/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dian_config.hpp"
#include <iostream>
#include <limits>

void Require(bool condition,const char* message)
{if(!condition)throw std::runtime_error(message);}
template<class Function>void Reject(Function function)
{
    bool rejected=false;try{function();}catch(const std::exception&){rejected=true;}
    Require(rejected,"Invalid input unexpectedly accepted");
}
int main(int argc,char** argv)
{
    try
    {
        Require(argc==2,"Expected policy root");
        auto cfg=dian::Config::Load(argv[1]);
        Require(cfg.protocol.position==12.57,"RS03 recent firmware range mismatch");
        const std::array<int,12> expected_ids{5,6,7,1,2,3,13,14,15,9,10,11};
        const std::array<int,4> expected_devices{1,0,3,2};
        for(int i=0;i<12;++i)
            Require(cfg.joints[i].id==expected_ids[i] && cfg.joints[i].device==expected_devices[i/3] &&
                    cfg.joints[i].sign==-1 && cfg.joints[i].zero==0,"DIAN remote motor mapping mismatch");
        Require(cfg.protocol.ReadParameter(5,0x7019).id==0x1100fd05,"Read-only parameter ID mismatch");
        Require(cfg.protocol.ReadParameter(5,0x7019).data==std::array<uint8_t,8>{0x19,0x70,0,0,0,0,0,0},"Read-only payload mismatch");
        const auto frame=cfg.protocol.Command(1,0,0,0,0);
        Require(frame.id==0x017fff01,"CAN arbitration ID mismatch");
        Require(frame.data==std::array<uint8_t,8>{0x7f,0xff,0x7f,0xff,0,0,0,0},"CAN command bytes mismatch");
        Require(cfg.protocol.Service(1,4).data==std::array<uint8_t,8>{},"Stop must not clear faults");
        Require(cfg.protocol.MotionMode(1).data==std::array<uint8_t,8>{5,0x70,0,0,0,0,0,0},"Mode bytes mismatch");
        Reject([&]{cfg.protocol.Command(1,std::numeric_limits<double>::quiet_NaN(),0,0,0);});
        Reject([&]{cfg.protocol.Command(1,13,0,0,0);});
        dian::Feedback feedback;
        dian::Frame response{(2u<<24)|(2u<<22)|(4u<<16)|(3u<<8)|253,{0,0,255,255,127,255,1,164}};
        Require(cfg.protocol.Parse(response,feedback),"Feedback not parsed");
        Require(feedback.id==3 && feedback.fault==4 && feedback.mode==2 && feedback.q==-cfg.protocol.position &&
                feedback.dq==20 && feedback.temperature==42,"Feedback decoding mismatch");
        dian::Sample sample;sample.q=cfg.policy.default_q;sample.motor_time.fill(dian::Now());sample.imu_time=dian::Now();
        dian::CheckState(cfg,sample,dian::Now(),false);
        auto outside=sample;outside.q[8]=cfg.policy.upper[8]+0.5;outside.temperature[0]=100;
        outside.torque[0]=61;outside.dq[0]=30;outside.gravity={0,0.6,-0.8};
        dian::CheckState(cfg,outside,dian::Now(),false);
        for(int test=0;test<4;++test)
        {
            auto bad=sample;
            if(test==0)bad.imu_time=0;if(test==1)bad.motor_time[0]=0;if(test==2)bad.fault[0]=1;
            if(test==3)bad.gravity={0,0,0};
            Reject([&]{dian::CheckState(cfg,bad,dian::Now(),false);});
        }
        std::cout<<"PASS: RS03 protocol, remote joint mapping, read/write frames, invalid data and state limits\n";
        return 0;
    }
    catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
