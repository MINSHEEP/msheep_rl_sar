/* Copyright (c) 2024-2026 Ziqi Fan; SPDX-License-Identifier: Apache-2.0 */
#include "dian_keyboard.hpp"
#include <iostream>
void require(bool valid,const char* message){if(!valid)throw std::runtime_error(message);}
int main()
{
    try
    {
        dian::KeyboardMapper keyboard({.5,.2,.5});
        require(keyboard.Input('m').request==dian::PadRequest::Enable,"Enable key");
        require(keyboard.Input('0').request==dian::PadRequest::Stand,"Stand key");
        require(keyboard.Input('1').request==dian::PadRequest::Policy,"Policy key");
        require(std::abs(keyboard.Input('w').demand[0]-.2)<1e-9,"Forward step is 0.1 m/s");
        for(int i=0;i<20;++i)keyboard.Input('w');
        require(keyboard.Output().demand[0]==1,"Forward saturation");
        require(keyboard.Input(' ').request==dian::PadRequest::Zero && dian::Norm(keyboard.velocity)==0,"Space zero");
        keyboard.Input('s');keyboard.Input('a');keyboard.Input('q');
        require(keyboard.velocity==dian::Vec3{-.1,.05,.1},"Reverse/lateral/yaw steps");
        keyboard.Input('d');keyboard.Input('e');
        require(keyboard.velocity==dian::Vec3{-.1,0,0},"Opposite directions");
        require(keyboard.Input('k').request==dian::PadRequest::Disable && dian::Norm(keyboard.velocity)==0,"Disable resets speed");
        for(char c:std::string("\x1b[1~\x1b[0m\x1bOA"))
            require(keyboard.Input(c).request==dian::PadRequest::None,"Escape sequence must not trigger controls");
        require(dian::Norm(keyboard.velocity)==0,"Arrow/escape sequence must not move");
        require(keyboard.Input('x').request==dian::PadRequest::Disable && keyboard.quit,"Exit disables");
        std::cout<<"PASS: keyboard control mapping, velocity steps/limits, zero/disable and escape sequences\n";
        return 0;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
