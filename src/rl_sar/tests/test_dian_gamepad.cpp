/* Copyright (c) 2024-2026 Ziqi Fan; SPDX-License-Identifier: Apache-2.0 */
#include "dian_gamepad.hpp"
#include <iostream>
void require(bool valid,const char* message){if(!valid)throw std::runtime_error(message);}
int main()
{
    try
    {
        dian::PadConfig cfg;dian::PadMapper mapper(cfg);
        std::vector<double> axes(8,0);std::vector<int> buttons(11,0);
        buttons[4]=buttons[0]=1;
        require(!mapper.Update(axes,buttons).ready,"Held startup buttons must not enable");
        buttons[4]=buttons[0]=0;require(mapper.Update(axes,buttons).ready,"Neutral startup");
        buttons[4]=1;mapper.Update(axes,buttons);buttons[0]=1;
        require(mapper.Update(axes,buttons).request==dian::PadRequest::Enable,"LB+A enable");
        require(mapper.Update(axes,buttons).request==dian::PadRequest::None,"Enable must be edge-triggered");
        buttons.assign(11,0);mapper.Update(axes,buttons);buttons[0]=1;
        require(mapper.Update(axes,buttons).request==dian::PadRequest::Stand,"A stand");
        buttons.assign(11,0);mapper.Update(axes,buttons);buttons[5]=1;axes[7]=-1;
        require(mapper.Update(axes,buttons).request==dian::PadRequest::Policy,"RB+Up policy");
        buttons.assign(11,0);axes[7]=0;axes[1]=-1;axes[0]=-.55;axes[3]=-.55;
        const auto drive=mapper.Update(axes,buttons);
        require(std::abs(drive.demand[0]-1)<1e-8 && std::abs(drive.demand[1]-.5)<1e-8 &&
                std::abs(drive.demand[2]-.5)<1e-8,"Joystick polarity and deadzone");
        buttons[1]=1;require(dian::Norm(mapper.Update(axes,buttons).demand)==0,"B zero");
        buttons[1]=0;require(dian::Norm(mapper.Update(axes,buttons).demand)==0,"B remains zero until sticks center");
        axes.assign(8,0);mapper.Update(axes,buttons);axes[1]=-1;
        require(mapper.Update(axes,buttons).demand[0]==1,"Resume after neutral");
        buttons[4]=buttons[5]=buttons[0]=1;
        require(mapper.Update(axes,buttons).request==dian::PadRequest::Disable,"Emergency stop priority");
        buttons[5]=0;buttons[1]=1;
        require(mapper.Update(axes,buttons).request==dian::PadRequest::Disable,"LB+B disable");
        std::cout<<"PASS: neutral startup, button edges, direction/scaling, zero latch, emergency stop priority\n";
        return 0;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
