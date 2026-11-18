/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "dian_config.hpp"
#include <Eigen/Geometry>
#include "hipnuc/custom_quad_imu.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <cstring>
#include <functional>
#include <atomic>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <memory>
#include <thread>
#include <sstream>
#include <unistd.h>

namespace dian
{
class Hardware
{
public:
    std::function<bool()> cancelled;
    explicit Hardware(const Config& config):cfg(config)
    {
        try
        {
            custom_quad::imu::Config imu;
            imu.device=cfg.imu_device;imu.baudrate=cfg.imu_baud;
            hi91=std::make_unique<custom_quad::imu::SerialReader>(imu);
            for(size_t i=0;i<cfg.devices.size();++i)buses.push_back(OpenSocketCAN(cfg.devices[i],cfg.usb_paths[i]));
            for(size_t i=0;i<buses.size();++i)readers.emplace_back([this,i]{Guard([&]{ReadCAN(i);});});
            readers.emplace_back([this]{Guard([&]{ReadIMU();});});
        }
        catch(...){Close();throw;}
    }
    ~Hardware(){Close();}
    Hardware(const Hardware&)=delete;
    Hardware& operator=(const Hardware&)=delete;
    Sample Snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex);
        if(error)std::rethrow_exception(error);
        return state;
    }
    struct Readback
    {
        Joints motor_q{},motor_dq{},torque_limit{};
        std::array<int,12> run_mode{};
        std::array<uint32_t,12> watchdog{};
    };
    Readback Query(bool limits=false,bool include_torque=true)
    {
        BeginRead(limits,include_torque);
        while(!Advance())
        {
            CancelCheck();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return readback;
    }
    void BeginBringup()
    {
        CancelBringup();
        const float torque=25.f;std::memcpy(&torque_bits,&torque,sizeof(torque_bits));
        FillReads(true,false);
        read_only=false;phase=Phase::Stop;cursor_joint=0;cursor_step=0;
    }
    void CancelBringup()
    {
        std::lock_guard<std::mutex> lock(mutex);
        phase=Phase::Idle;waiting=false;reply_ready=false;read_only=false;enable_armed=false;
    }
    // One CAN exchange per call. The control loop keeps running between calls.
    bool Advance()
    {
        CancelCheck();
        {
            std::lock_guard<std::mutex> lock(mutex);
            if(error)std::rethrow_exception(error);
            if(phase==Phase::Ready)return true;
            if(phase==Phase::Idle)return false;
        }
        if(phase==Phase::Stop)
        {
            Send(cfg.joints[cursor_joint],cfg.protocol.Service(cfg.joints[cursor_joint].id,4));
            if(++cursor_joint==12){phase=Phase::Write;cursor_joint=0;cursor_step=0;}
            return false;
        }
        if(phase==Phase::Write)
        {
            const auto& j=cfg.joints[cursor_joint];
            if(cursor_step==0)Send(j,cfg.protocol.MotionMode(j.id));
            else if(cursor_step==1)Send(j,cfg.protocol.WriteParameter(j.id,0x7028,2000));
            else Send(j,cfg.protocol.WriteParameter(j.id,0x700b,torque_bits));
            if(++cursor_step==3){cursor_step=0;if(++cursor_joint==12){phase=Phase::Read;cursor_joint=0;}}
            return false;
        }
        if(phase==Phase::Read)
        {
            if(!waiting){ArmRead();return false;}
            bool ready=false;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if(error)std::rethrow_exception(error);
                ready=reply_ready;
            }
            if(!ready)
            {
                if(Now()>read_deadline)ThrowMissing();
                return false;
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                waiting=false;reply_ready=false;
            }
            if(++cursor_step==read_count)
            {
                cursor_step=0;
                if(++cursor_joint==12)return FinishReads();
            }
            ArmRead();
            return false;
        }
        if(phase==Phase::Feedback)
        {
            const auto s=Snapshot();
            const bool motors=*std::min_element(s.motor_time.begin(),s.motor_time.end())>0;
            if(s.imu_time>0 && motors){phase=Phase::Enable;cursor_joint=0;enable_armed=false;return false;}
            if(Now()>feedback_deadline)throw std::runtime_error("Startup feedback timeout");
            return false;
        }
        if(!enable_armed)
        {
            const auto& j=cfg.joints[cursor_joint];
            Send(j,cfg.protocol.Command(j.id,0,0,0,0));
            Send(j,cfg.protocol.Service(j.id,3));
            if(++cursor_joint==12){enable_armed=true;enable_started=Now();cursor_joint=0;}
            return false;
        }
        const auto s=Snapshot();bool ready=true;
        for(int i=0;i<12;++i)
        {
            if(s.fault[i])throw std::runtime_error("Motor fault during enable");
            ready=ready && s.mode[i]==2 && s.motor_time[i]>enable_started;
        }
        if(ready){phase=Phase::Ready;return true;}
        if(Now()-enable_started>cfg.startup_timeout)throw std::runtime_error("Incomplete motor enable feedback");
        Send(cfg.joints[cursor_joint%12],cfg.protocol.Command(cfg.joints[cursor_joint%12].id,0,0,0,0));
        ++cursor_joint;
        return false;
    }
    void Disable()
    {
        std::exception_ptr first;
        for(const auto& j:cfg.joints)
        {
            try{Send(j,cfg.protocol.Service(j.id,4));}
            catch(...){if(!first)first=std::current_exception();}
        }
        if(first)std::rethrow_exception(first);
    }
    void Targets(const Joints& q)
    {
        std::array<Frame,12> frames;
        for(int i=0;i<12;++i)
        {
            const auto& j=cfg.joints[i];
            frames[i]=cfg.protocol.Command(j.id,j.zero+j.sign*q[i],0,j.kp,j.kd);
        }
        for(int i=0;i<12;++i)Send(cfg.joints[i],frames[i]);
    }
private:
    void CancelCheck()
    {
        if(cancelled && cancelled())throw std::runtime_error("Operator cancelled motor startup");
    }
    enum class Phase { Idle, Stop, Write, Read, Feedback, Enable, Ready };
    void FillReads(bool limits,bool include_torque)
    {
        read_count=0;
        for(uint16_t index:{uint16_t{0x7019},uint16_t{0x701b},uint16_t{0x7005},uint16_t{0x700b},uint16_t{0x7028}})
            if((limits || (index!=0x700b && index!=0x7028)) && (include_torque || index!=0x700b))
                read_list[read_count++]=index;
    }
    void BeginRead(bool limits,bool include_torque)
    {
        CancelBringup();
        FillReads(limits,include_torque);
        read_only=true;phase=Phase::Read;cursor_joint=0;cursor_step=0;
    }
    void ArmRead()
    {
        const auto index=read_list[cursor_step];
        const auto& j=cfg.joints[cursor_joint];
        {
            std::lock_guard<std::mutex> lock(mutex);
            if(error)std::rethrow_exception(error);
            expect_joint=cursor_joint;expect_index=index;reply_ready=false;waiting=true;
            read_deadline=Now()+0.5;
        }
        Send(j,cfg.protocol.ReadParameter(j.id,index));
    }
    bool FinishReads()
    {
        Readback initial;
        {
            std::lock_guard<std::mutex> lock(mutex);
            initial=readback;
        }
        if(read_only){phase=Phase::Ready;return true;}
        std::ostringstream mismatch;
        for(int i=0;i<12;++i)
            if(initial.run_mode[i]!=0 || initial.watchdog[i]!=2000)
                mismatch<<" ["<<cfg.devices[cfg.joints[i].device]<<" id="<<cfg.joints[i].id
                        <<" run_mode="<<initial.run_mode[i]<<" watchdog_raw="<<initial.watchdog[i]<<']';
        if(!mismatch.str().empty())
            throw std::runtime_error("Motor parameter mismatch (expected mode=0, watchdog_raw=2000):"+mismatch.str());
        phase=Phase::Feedback;feedback_deadline=Now()+cfg.startup_timeout;
        return false;
    }
    void ThrowMissing()
    {
        const auto& j=cfg.joints[expect_joint];
        std::ostringstream message;
        message<<"Type17 readback incomplete: ["<<cfg.devices[j.device]<<" id="<<j.id
               <<" missing=0x"<<std::hex<<expect_index<<std::dec<<']';
        throw std::runtime_error(message.str());
    }
    int OpenSocketCAN(const std::string& name,const std::string& usb_path)
    {
        const auto sysfs=std::filesystem::path("/sys/class/net")/name/"device";
        if(std::filesystem::canonical(sysfs).filename()!=usb_path)
            throw std::runtime_error("CAN USB port mismatch for "+name);
        if(name.size()>=IFNAMSIZ)throw std::runtime_error("Invalid CAN interface name");
        const int fd=socket(PF_CAN,SOCK_RAW|SOCK_CLOEXEC|SOCK_NONBLOCK,CAN_RAW);
        if(fd<0)throw std::runtime_error("Cannot create SocketCAN socket");
        ifreq request{};std::strncpy(request.ifr_name,name.c_str(),IFNAMSIZ-1);
        if(ioctl(fd,SIOCGIFINDEX,&request)<0){close(fd);throw std::runtime_error("CAN interface not found: "+name);}
        const auto lockpath="/tmp/custom-quad-can-"+std::to_string(request.ifr_ifindex)+".lock";
        const int lockfd=open(lockpath.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0660);
        if(lockfd<0 || flock(lockfd,LOCK_EX|LOCK_NB)<0)
        {
            if(lockfd>=0)close(lockfd);close(fd);
            throw std::runtime_error("CAN is held by another custom_quad/DIAN controller: "+name);
        }
        can_locks.push_back(lockfd);
        sockaddr_can address{};address.can_family=AF_CAN;address.can_ifindex=request.ifr_ifindex;
        if(bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0)
        {close(fd);throw std::runtime_error("Cannot bind SocketCAN: "+name);}
        return fd;
    }
    template <class Function> void Guard(Function function)
    {
        try{function();}
        catch(...){std::lock_guard<std::mutex> lock(mutex);error=std::current_exception();stopping=true;}
    }
    void Send(const Joint& j,Frame frame)
    {
        can_frame raw{};raw.can_id=frame.id|CAN_EFF_FLAG;raw.can_dlc=8;
        std::copy(frame.data.begin(),frame.data.end(),raw.data);
        if(write(buses[j.device],&raw,sizeof(raw))!=sizeof(raw))throw std::runtime_error("SocketCAN send failed");
    }
    void ReadCAN(int device)
    {
        while(!stopping)
        {
            pollfd descriptor{buses[device],POLLIN,0};
            const int ready=poll(&descriptor,1,2);
            if(ready<0){if(errno==EINTR)continue;throw std::runtime_error("CAN poll failed");}
            if(descriptor.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("CAN interface failed");
            if(!ready)continue;
            can_frame raw{};const auto count=read(buses[device],&raw,sizeof(raw));
            if(count<0 && (errno==EAGAIN || errno==EINTR))continue;
            if(count!=sizeof(raw))throw std::runtime_error("CAN receive failed");
            if(!(raw.can_id&CAN_EFF_FLAG) || (raw.can_id&(CAN_RTR_FLAG|CAN_ERR_FLAG)) || raw.can_dlc!=8)continue;
            Frame frame;frame.id=raw.can_id&CAN_EFF_MASK;
            std::copy(raw.data,raw.data+8,frame.data.begin());
            if(((frame.id>>24)&31)==17 && (frame.id&255)==static_cast<uint32_t>(cfg.protocol.master))
            {
                const int motor=(frame.id>>8)&255;
                if(((frame.id>>16)&255)!=0 || frame.data[2] || frame.data[3])
                    throw std::runtime_error("RS03 parameter read failed");
                const uint16_t index=frame.data[0]|(frame.data[1]<<8);
                uint32_t bits=0;for(int k=0;k<4;++k)bits|=uint32_t(frame.data[4+k])<<(8*k);
                float value;std::memcpy(&value,&bits,sizeof(value));
                for(int i=0;i<12;++i)
                {
                    const auto& j=cfg.joints[i];if(j.device!=device || j.id!=motor)continue;
                    std::lock_guard<std::mutex> lock(mutex);
                    if(index==0x7019 || index==0x701b)
                    {
                        if(!std::isfinite(value))throw std::runtime_error("Invalid parameter feedback");
                        if(index==0x7019){readback.motor_q[i]=value;query_seen[i]|=1;}
                        else{readback.motor_dq[i]=value;query_seen[i]|=2;}
                    }
                    else if(index==0x7005){readback.run_mode[i]=frame.data[4];query_seen[i]|=4;}
                    else if(index==0x700b){readback.torque_limit[i]=value;query_seen[i]|=8;}
                    else if(index==0x7028){readback.watchdog[i]=bits;query_seen[i]|=16;}
                    if(waiting && i==expect_joint && index==expect_index)reply_ready=true;
                }
                continue;
            }
            Feedback feedback;
            if(!cfg.protocol.Parse(frame,feedback))continue;
            for(int i=0;i<12;++i)
            {
                const auto& j=cfg.joints[i];
                if(j.device!=device || j.id!=feedback.id)continue;
                std::lock_guard<std::mutex> lock(mutex);
                state.q[i]=j.sign*(feedback.q-j.zero);state.dq[i]=j.sign*feedback.dq;
                state.torque[i]=j.sign*feedback.torque;state.temperature[i]=feedback.temperature;
                state.fault[i]=feedback.fault;state.mode[i]=feedback.mode;state.motor_time[i]=Now();
            }
        }
    }
    void ReadIMU()
    {
        Eigen::Matrix3d mount;
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)mount(i,j)=cfg.mount[i*3+j];
        const Eigen::Quaterniond mounting(mount);
        const custom_quad::imu::Quaternion qmount{mounting.w(),mounting.x(),mounting.y(),mounting.z()};
        while(!stopping)
        {
            if(!hi91->poll(20))continue;
            const auto& packet=*hi91->decoder().latest();
            const auto body=custom_quad::imu::to_body(packet,qmount);
            const double time=std::chrono::duration<double>(packet.received_at.time_since_epoch()).count();
            std::lock_guard<std::mutex> lock(mutex);
            imu_filter.Update(body.angular_velocity_rad_s,body.projected_gravity,time);
            state.gravity=imu_filter.gravity;state.gyro=imu_filter.gyro;
            state.quaternion=body.quaternion_wxyz;state.acceleration=body.acceleration_m_s2;
            state.imu_time=time;
        }
    }
    void Close()
    {
        stopping=true;for(auto& reader:readers)if(reader.joinable())reader.join();readers.clear();
        hi91.reset();
        for(int bus:buses)close(bus);
        buses.clear();
        for(int fd:can_locks)close(fd);can_locks.clear();
    }
    Config cfg;Sample state;Readback readback;ImuFilter imu_filter;std::array<int,12> query_seen{};
    Phase phase=Phase::Idle;bool read_only=false,waiting=false,reply_ready=false,enable_armed=false;
    int cursor_joint=0,cursor_step=0,read_count=0,expect_joint=-1;
    uint16_t expect_index=0;uint32_t torque_bits=0;
    double read_deadline=0,feedback_deadline=0,enable_started=0;
    std::array<uint16_t,5> read_list{};
    std::unique_ptr<custom_quad::imu::SerialReader> hi91;
    std::vector<int> buses,can_locks;
    std::atomic<bool> stopping{false};std::vector<std::thread> readers;
    std::mutex mutex;std::exception_ptr error;
};
}
