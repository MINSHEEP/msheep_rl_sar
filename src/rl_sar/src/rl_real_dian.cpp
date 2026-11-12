/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rl_real_dian.hpp"
#include "fsm_dian.hpp"
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>

namespace
{
const char* kJoints[]={"FR_J1","FR_J2","FR_J3","FL_J1","FL_J2","FL_J3","RR_J1","RR_J2","RR_J3","RL_J1","RL_J2","RL_J3"};
}
struct DianTrace
{
    std::ofstream file;
    std::mutex mu;
    std::condition_variable cv;
    std::deque<std::string> lines;
    bool closing=false,idle=true;
    std::thread worker;
    explicit DianTrace(const std::filesystem::path& dir,const dian::Config& cfg)
    {
        std::filesystem::create_directories(dir);
        std::ofstream session(dir/"session.txt");
        if(!session)throw std::runtime_error("Cannot write "+(dir/"session.txt").string());
        session<<"policy "<<cfg.policy_name<<"\nmodel "<<cfg.model_path<<"\nperiod_s 0.005\njoint_order";
        for(const char* name:kJoints)session<<' '<<name;
        auto row=[&](const char* label,const dian::Joints& values){session<<'\n'<<label;for(double v:values)session<<' '<<v;};
        row("default_q",cfg.policy.default_q);row("target_lower",cfg.policy.lower);row("target_upper",cfg.policy.upper);
        row("action_scale",cfg.policy.scale);
        session<<"\n说明\n"
               <<"trace.csv 每个控制周期一行。state 为 RLFSMStateRLLocomotion 时策略在出力。\n"
               <<"q、dq、tau 是实测位置、速度、力矩。target 是发给电机的目标角。\n"
               <<"action 是网络原始输出。requested 是 default+scale*action，还没做软限位和滤波。\n"
               <<"gravity 是机体系重力，水平时接近 0 0 -1。bad_joint 为 -1 表示没有故障关节。\n"
               <<"stop.txt 是退出原因，completed 表示正常结束，interrupted 表示 Ctrl+C。\n";
        file.open(dir/"trace.csv");
        if(!file)throw std::runtime_error("Cannot write "+(dir/"trace.csv").string());
        file<<"t,state,vx,vy,wz";
        for(const char* prefix:{"q","dq","tau","target","action","requested"})
            for(const char* name:kJoints)file<<','<<prefix<<'_'<<name;
        file<<",gyro_x,gyro_y,gyro_z,grav_x,grav_y,grav_z,qw,qx,qy,qz,imu_age,bad_joint\n";
        file.flush();
        worker=std::thread([this]{Loop();});
    }
    ~DianTrace()
    {
        {std::lock_guard<std::mutex> lock(mu);closing=true;}
        cv.notify_all();
        if(worker.joinable())worker.join();
    }
    void Push(std::string line)
    {
        std::lock_guard<std::mutex> lock(mu);
        if(closing)return;
        if(lines.size()>=4000)lines.pop_front();
        lines.push_back(std::move(line));
        idle=false;cv.notify_all();
    }
    void Drain()
    {
        std::unique_lock<std::mutex> lock(mu);
        cv.wait(lock,[&]{return idle && lines.empty();});
    }
    void Loop()
    {
        try
        {
            std::unique_lock<std::mutex> lock(mu);
            while(true)
            {
                if(lines.empty())
                {
                    idle=true;cv.notify_all();
                    if(closing)break;
                    cv.wait(lock,[&]{return closing || !lines.empty();});
                    continue;
                }
                idle=false;
                if(!closing)cv.wait_for(lock,std::chrono::milliseconds(50),[&]{return closing;});
                std::deque<std::string> batch;batch.swap(lines);
                lock.unlock();
                for(const auto& line:batch)file<<line<<'\n';
                file.flush();
                lock.lock();
            }
        }
        catch(...)
        {
            std::lock_guard<std::mutex> lock(mu);
            lines.clear();idle=true;closing=true;cv.notify_all();
        }
    }
};

volatile std::sig_atomic_t RLRealDian::interrupted=0;

RLRealDian::RLRealDian(const std::filesystem::path& root,const std::string& mode,bool arm,dian::Vec3 cmd,
                       const std::string& selected_policy,const std::string& pad_path,const std::string& override_model,
                       const std::string& record_path)
    :run_mode(mode),config(dian::Config::Load(root,selected_policy)),contract(config.policy),command(cmd)
{
    robot_name="dian";config_name=selected_policy;
    params.config_node=YAML::LoadFile((root/"dian/base.yaml").string())["dian"];
    InitJointNum(12);
    dian::Finite(command,"command");
    for(int i=0;i<3;++i)if(std::abs(command[i])>config.command_limits[i])throw std::runtime_error("Command exceeds configured limit");
    interactive=mode=="keyboard" || mode=="keyboard-mock" || mode=="gamepad" || (mode=="mock" && !pad_path.empty());
    if(mode=="run" || mode=="stand")
    {
        if(!arm)throw std::runtime_error("Automatic motion requires --arm; gamepad mode enables with LB+A");
    }
    const bool needs_model=mode=="check" || mode=="mock" || mode=="run" || mode=="gamepad" || mode=="keyboard" || mode=="keyboard-mock";
    if(needs_model)
    {
        if(!override_model.empty())config.model_path=override_model;
        if(!std::filesystem::is_regular_file(config.model_path))
            throw std::runtime_error("Policy ONNX not installed: "+config.model_path);
        model=InferenceRuntime::ModelFactory::load_model(config.model_path);
        if(!model || model->get_model_type()!="onnx")throw std::runtime_error("DIAN requires ONNX Runtime");
        const auto test=model->forward({std::vector<float>(config.observation_size,0)});
        if(test.size()!=12)throw std::runtime_error("Model must output 12 actions");
        dian::Finite(test,"model output");
    }
    if(mode=="check")return;
    if(mode=="gamepad" || (mode=="mock" && !pad_path.empty()) || mode=="gamepad-read")
    {
        auto settings=dian::PadConfig::Load(root/"dian/gamepad.yaml");
        if(!pad_path.empty())settings.device=pad_path;
        gamepad=std::make_unique<dian::Gamepad>(settings);
    }
    if(mode=="keyboard" || mode=="keyboard-read" || mode=="keyboard-mock")
        keyboard=std::make_unique<dian::Keyboard>(config.command_limits);
    if(mode=="gamepad-read" || mode=="keyboard-read")return;
    if(mode=="mock" || mode=="keyboard-mock")
    {
        sample.q=config.policy.default_q;sample.temperature.fill(25);
        std::cout<<LOGGER::WARNING<<"MOCK: synthetic feedback, no CAN/IMU devices opened"<<std::endl;
    }
    else
    {
        hardware=std::make_unique<dian::Hardware>(config);
        hardware->cancelled=[this]
        {
            return interrupted || (gamepad && gamepad->Poll().request==dian::PadRequest::Disable)
                || (keyboard && keyboard->Poll().request==dian::PadRequest::Disable);
        };
    }
    auto created=FSMManager::GetInstance().CreateFSM(robot_name,this);
    if(!created)throw std::runtime_error("DIAN FSM registration missing");
    fsm=*created;
    if(!record_path.empty())
    {
        record_directory=record_path;
        trace=std::make_unique<DianTrace>(record_directory,config);
        trace_t0=dian::Now();
        std::cout<<"Recording "<<(record_directory/"trace.csv").string()<<'\n';
    }
}

RLRealDian::~RLRealDian()
{
    try{Stop();}catch(const std::exception& e){std::cerr<<"STOP FAILED: "<<e.what()<<"; use physical emergency stop\n";}
}
void RLRealDian::Refresh()
{
    if(hardware)sample=hardware->Snapshot();
    else
    {
        const double now=dian::Now();sample.motor_time.fill(now);sample.imu_time=now;
        sample.mode.fill(enabled?2:0);
    }
    GetState(&robot_state);
}
void RLRealDian::GetState(RobotState<float>* state)
{
    for(int i=0;i<12;++i)
    {
        state->motor_state.q[i]=sample.q[i];state->motor_state.dq[i]=sample.dq[i];
        state->motor_state.tau_est[i]=sample.torque[i];
    }
    for(int i=0;i<3;++i)
    {
        state->imu.gyroscope[i]=sample.gyro[i];state->imu.accelerometer[i]=sample.acceleration[i];
    }
    for(int i=0;i<4;++i)state->imu.quaternion[i]=sample.quaternion[i];
}
void RLRealDian::SetCommand(const RobotCommand<float>* cmd)
{
    dian::Joints target{};
    for(int i=0;i<12;++i)
    {
        target[i]=cmd->motor_command.q[i];
        if(!std::isfinite(target[i]))throw std::runtime_error("Invalid command target");
    }
    if(hardware)hardware->Targets(target);else sample.q=target;
}
std::vector<float> RLRealDian::Forward()
{
    const auto observation=contract.Observe(sample,command);
    const auto action=model->forward({observation});
    SetTarget(contract.Process(action,command));return action;
}
void RLRealDian::SetTarget(const dian::Joints& target)
{
    for(int i=0;i<12;++i)
    {
        auto& c=robot_command.motor_command;
        c.q[i]=target[i];c.dq[i]=0;c.tau[i]=0;c.kp[i]=config.joints[i].kp;c.kd[i]=config.joints[i].kd;
    }
}
void RLRealDian::ApplyOperator(const dian::PadOutput& pad)
{
    operator_request=pad.request;
    for(int i=0;i<3;++i)command[i]=pad.demand[i]*config.command_limits[i];
    if(operator_request==dian::PadRequest::Disable)
    {
        Stop();
        // Cancel a pending enable transition as well as an already running state.
        fsm.mode_=FSM::Mode::NORMAL;fsm.next_state_=fsm.current_state_;
        fsm.RequestStateChange("RLFSMStatePassive");
    }
}
void RLRealDian::BeginHold()
{
    if(enabled)return;
    send_authorized=true;
    if(hardware)hardware->Prepare();
    const double deadline=dian::Now()+config.startup_timeout;
    do
    {
        Refresh();
        if(sample.imu_time>0 && *std::min_element(sample.motor_time.begin(),sample.motor_time.end())>0)break;
        if(interrupted)return;
        if(dian::Now()>deadline)throw std::runtime_error("Startup feedback timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }while(true);
    dian::CheckState(config,sample,dian::Now(),false);
    SetTarget(sample.q);stand_done=false;
    if(interrupted)return;
    if(hardware)hardware->Enable();
    enabled=true;enabled_time=dian::Now();
    std::cout<<"Motors enabled; holding measured pose. "<<(keyboard?"Press 0 to stand.":"Press A to stand.")<<'\n';
}
void RLRealDian::BeginStand()
{
    if(!enabled)BeginHold();
    Refresh();dian::CheckState(config,sample,dian::Now(),false);
    stand_start=sample.q;stand_time=dian::Now();stand_done=false;
    SetTarget(stand_start);
}
void RLRealDian::StandStep()
{
    const double fraction=std::clamp((dian::Now()-stand_time)/config.stand_duration,0.,1.);
    dian::Joints target{};
    for(int i=0;i<12;++i)target[i]=stand_start[i]+fraction*(config.policy.default_q[i]-stand_start[i]);
    SetTarget(target);
    if(!stand_done && fraction>=1.)
        std::cout<<"Standing ready. "<<(keyboard?"Press 1 to start RL.":"Release shoulders, then RB+Up starts policy.")<<'\n';
    stand_done=fraction>=1.;
}
void RLRealDian::BeginPolicy()
{
    dian::Joints previous_target{};
    for(int i=0;i<12;++i)previous_target[i]=robot_command.motor_command.q[i];
    // A completed standing transition has already held a stationary command for its duration.
    const double standing_age=dian::Norm(command)<=0.1?std::max(0.0,dian::Now()-stand_time):0.0;
    contract.Reset(previous_target,standing_age);control_tick=0;
}
void RLRealDian::PolicyStep()
{
    if(control_tick++%4==0)Forward();
}
void RLRealDian::Stop()
{
    if(send_authorized && hardware)hardware->Disable();
    enabled=false;send_authorized=false;stand_done=false;
}
void RLRealDian::WriteTrace()
{
    if(!trace)return;
    const std::string state=fsm.current_state_?fsm.current_state_->GetStateName():"none";
    dian::Joints sent{};
    for(int i=0;i<12;++i)sent[i]=robot_command.motor_command.q[i];
    int bad=-1;
    for(int i=0;i<12 && bad<0;++i)if(sample.fault[i] || (enabled && sample.mode[i]!=2))bad=i;
    std::ostringstream line;
    line.setf(std::ios::fixed);line<<std::setprecision(4);
    line<<dian::Now()-trace_t0<<','<<state<<','<<command[0]<<','<<command[1]<<','<<command[2];
    auto put=[&](const auto& values){for(double v:values)line<<','<<v;};
    put(sample.q);put(sample.dq);put(sample.torque);put(sent);
    put(contract.RawAction());put(contract.Requested());
    put(sample.gyro);put(sample.gravity);
    for(double v:sample.quaternion)line<<','<<v;
    line<<','<<(sample.imu_time>0?dian::Now()-sample.imu_time:-1)<<','<<bad;
    trace->Push(line.str());
}
void RLRealDian::NoteStop(const std::string& reason)
{
    if(record_directory.empty())return;
    WriteTrace();
    if(trace){trace->Push("# stop "+reason);trace->Drain();}
    std::ofstream stop(record_directory/"stop.txt");
    if(!stop)return;
    stop<<reason<<'\n';
    if(fsm.current_state_)stop<<"state "<<fsm.current_state_->GetStateName()<<'\n';
    stop<<"gravity";for(double v:sample.gravity)stop<<' '<<v;
    stop<<"\nq";for(double v:sample.q)stop<<' '<<v;
    stop<<"\ntarget";for(int i=0;i<12;++i)stop<<' '<<robot_command.motor_command.q[i];
    stop<<"\ntorque";for(double v:sample.torque)stop<<' '<<v;
    stop<<'\n';
}
void RLRealDian::Run(double duration)
{
    if(!std::isfinite(duration) || duration<0)throw std::runtime_error("Duration must be nonnegative");
    if(run_mode=="check")
    {std::cout<<"DIAN "<<config_name<<": ONNX [1,"<<config.observation_size<<"] -> [1,12], no devices opened\n";return;}
    if(run_mode=="keyboard-read")
    {
        dian::Keyboard::Help();const double started=dian::Now();
        while(!interrupted && !keyboard->QuitRequested() && (duration==0 || dian::Now()-started<duration))
        {
            keyboard->Poll();std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return;
    }
    if(run_mode=="gamepad-read")
    {
        const double started=dian::Now();
        while(!interrupted && (duration==0 || dian::Now()-started<duration))
        {
            auto pad=gamepad->Poll();gamepad->Print();
            std::cout<<"ready="<<pad.ready<<" demand="<<pad.demand[0]<<','<<pad.demand[1]<<','<<pad.demand[2]<<'\n';
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return;
    }
    if(run_mode=="read" || run_mode=="read-parameters")
    {
        const bool parameters=run_mode=="read-parameters";
        const auto readings=hardware->Query(parameters);
        const std::array<std::string,4> legs{"FR","FL","RR","RL"};
        for(int i=0;i<12;++i)
        {
            const auto& j=config.joints[i];
            std::cout<<legs[i/3]<<"_J"<<i%3+1<<" bus="<<config.devices[j.device]<<" id="<<j.id
                     <<" motor_q="<<readings.motor_q[i]<<" joint_q="<<j.sign*(readings.motor_q[i]-j.zero)
                     <<" delta_default="<<j.sign*(readings.motor_q[i]-j.zero)-config.policy.default_q[i]
                     <<" joint_dq="<<j.sign*readings.motor_dq[i]<<" run_mode="<<readings.run_mode[i];
            if(parameters)
                std::cout<<" watchdog_raw="<<readings.watchdog[i]
                         <<" watchdog_hex=0x"<<std::hex<<readings.watchdog[i]<<std::dec
                         <<" torque_limit_nm="<<readings.torque_limit[i];
            std::cout<<'\n';
        }
        std::cout<<"Type17 read only. run_mode is a control-mode parameter, NOT proof of motor disable.\n";
        return;
    }
    if(run_mode=="monitor")
    {
        const double end=dian::Now()+duration;
        while(!interrupted && (duration==0 || dian::Now()<end))
        {
            Refresh();std::cout<<"q:";for(auto q:sample.q)std::cout<<' '<<q;
            std::cout<<" imu_age="<<dian::Now()-sample.imu_time<<"\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return;
    }
    start_requested=!interactive;
    if(keyboard)dian::Keyboard::Help();
    else if(interactive)std::cout<<"Disabled. Center sticks, release buttons. LB+A enable; A stand; RB+Up policy; B zero; LB+B disable.\n";
    const double start=dian::Now();double next=start,max_elapsed=0;unsigned long ticks=0;
    try
    {
        while(!interrupted && (duration==0 || dian::Now()-start<duration))
        {
            const double now=dian::Now();
            if(enabled && now-next>config.max_cycle)throw std::runtime_error("Control scheduling deadline missed");
            operator_request=dian::PadRequest::None;
            if(gamepad)ApplyOperator(gamepad->Poll());
            if(keyboard)
            {
                ApplyOperator(keyboard->Poll());
                if(keyboard->QuitRequested())break;
            }
            Refresh();
            if(enabled)dian::CheckState(config,sample,dian::Now(),true);
            const bool was_enabled=enabled;
            StateController(&robot_state,&robot_command);
            const double cycle_start=(!was_enabled && enabled)?dian::Now():now;
            Refresh();
            if(enabled)dian::CheckState(config,sample,dian::Now(),true);
            if(was_enabled && enabled && dian::Now()-cycle_start>config.max_cycle)
                throw std::runtime_error("Inference/control deadline missed");
            if(enabled && !interrupted)SetCommand(&robot_command);
            WriteTrace();
            const double elapsed=dian::Now()-cycle_start;max_elapsed=std::max(max_elapsed,elapsed);
            if(enabled && elapsed>config.max_cycle)throw std::runtime_error("CAN send deadline missed");
            ++ticks;next+=.005;next=std::max(next,dian::Now());
            std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.,next-dian::Now())));
        }
        fsm.RequestStateChange("RLFSMStateStop");StateController(&robot_state,&robot_command);
        NoteStop(interrupted?"interrupted":"completed");
    }
    catch(const std::exception& error)
    {
        try{NoteStop(error.what());}catch(...){}
        throw;
    }
    std::cout<<"Completed "<<ticks<<" control ticks, max work "<<max_elapsed*1000<<" ms\n";
}
#ifndef DIAN_LIBRARY_ONLY
int main(int argc,char** argv)
{
    try
    {
        std::filesystem::path root=POLICY_DIR;std::string mode="check",selected_policy="student227",pad_path,model_path,record_path;bool arm=false;double duration=0;
        dian::Vec3 command{};
        for(int i=1;i<argc;++i)
        {
            const std::string arg=argv[i];
            auto value=[&](){if(++i>=argc)throw std::runtime_error("Missing value for "+arg);return std::string(argv[i]);};
            if(arg=="--policy-root")root=value();else if(arg=="--mode")mode=value();
            else if(arg=="--policy")selected_policy=value();else if(arg=="--gamepad")pad_path=value();
            else if(arg=="--model")model_path=value();
            else if(arg=="--duration")duration=std::stod(value());else if(arg=="--record")record_path=value();
            else if(arg=="--arm")arm=true;
            else if(arg=="--command"){for(auto& x:command)x=std::stod(value());}
            else if(arg=="--help")
            {
                std::cout<<"rl_real_dian [--policy-root PATH] [--mode check|mock|read|read-parameters|monitor|stand|run|gamepad|gamepad-read|keyboard|keyboard-read|keyboard-mock]\n"
                         <<"  [--policy student227] [--model ONNX] [--gamepad /dev/input/js0] [--record DIR]\n"
                         <<"  [--arm] [--duration SECONDS] [--command VX VY WZ]\n"
                         <<"Default: check, student227. Duration 0 runs until Ctrl+C.\n"
                         <<"Keyboard: m enable, 0 stand, 1 RL, w/s/a/d/q/e velocity, Space zero, k disable, x exit.\n"
                         <<"Gamepad: LB+A enable, A stand, RB+Up policy, B zero, LB+B or LB+RB disable.\n"
                         <<"Automatic run/stand requires --arm; interactive modes enable only from operator input.\n";
                return 0;
            }
            else throw std::runtime_error("Unknown argument: "+arg);
        }
        if(mode!="check" && mode!="mock" && mode!="read" && mode!="read-parameters" && mode!="monitor" && mode!="stand" && mode!="run" && mode!="gamepad" && mode!="gamepad-read" && mode!="keyboard" && mode!="keyboard-read" && mode!="keyboard-mock")throw std::runtime_error("Invalid mode");
        if(!std::isfinite(duration)||duration<0)throw std::runtime_error("Invalid duration");
        std::signal(SIGINT,[](int){RLRealDian::interrupted=1;});std::signal(SIGTERM,[](int){RLRealDian::interrupted=1;});
        std::signal(SIGHUP,[](int){RLRealDian::interrupted=1;});
        RLRealDian app(root,mode,arm,command,selected_policy,pad_path,model_path,record_path);app.Run(duration);return 0;
    }
    catch(const std::exception& e){std::cerr<<LOGGER::ERROR<<"DIAN stopped: "<<e.what()<<std::endl;return 1;}
}

#endif
