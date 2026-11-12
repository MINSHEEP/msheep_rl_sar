/* Copyright (c) 2024-2026 Ziqi Fan; SPDX-License-Identifier: Apache-2.0 */
#include "rl_real_dian.hpp"
#include <iostream>
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
int main(int argc,char** argv)
{
    try
    {
        require(argc==3,"Expected policy root and untrained FSM test graph");
        RLRealDian app(argv[1],"mock",false,{},"student227","",argv[2]);app.interactive=true;
        auto tick=[&]{app.StateController(&app.robot_state,&app.robot_command);};
        auto request=[&](dian::PadRequest value){app.ApplyOperator({{},value,true});};
        request(dian::PadRequest::Enable);tick();
        require(app.fsm.mode_==FSM::Mode::CHANGE,"Enable should request Hold");
        request(dian::PadRequest::Disable);tick();
        require(app.fsm.current_state_->GetStateName()=="RLFSMStatePassive","Emergency must cancel pending enable");
        request(dian::PadRequest::Enable);tick();tick();
        require(app.fsm.current_state_->GetStateName()=="RLFSMStateHold","Enable to Hold");
        app.config.stand_duration=.001;
        request(dian::PadRequest::Stand);tick();tick();
        request(dian::PadRequest::None);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));tick();
        require(app.stand_done,"Stand transition should complete");
        // Use a short configured settle time to exercise the real standing-to-policy path.
        app.contract.cfg.settle=.001;
        request(dian::PadRequest::Policy);tick();tick();
        require(app.fsm.current_state_->GetStateName()=="RLFSMStateRLLocomotion","Policy transition");
        const auto previous=app.contract.target;
        const auto quiet=app.contract.Process(std::vector<float>(12,1.0f),{});
        require(std::abs(quiet[0]-previous[0])<=app.contract.cfg.target_velocity[0]*app.contract.cfg.dt*
                app.contract.cfg.quiet_scale+1e-9,"Policy takeover reset standing filter");
        request(dian::PadRequest::Zero);tick();
        require(app.fsm.current_state_->GetStateName()=="RLFSMStateRLLocomotion","Zero demand keeps policy active");
        request(dian::PadRequest::Disable);tick();
        require(app.fsm.current_state_->GetStateName()=="RLFSMStatePassive","Disable returns Passive");
        request(dian::PadRequest::Enable);tick();tick();
        require(app.fsm.current_state_->GetStateName()=="RLFSMStateHold","Explicit re-enable after stop");
        std::cout<<"PASS: real DIAN FSM transitions, emergency cancels queued enable, re-enable, no hardware\n";
        return 0;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
