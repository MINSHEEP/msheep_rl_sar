/* Copyright (c) 2024-2026 Ziqi Fan; SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "dian_gamepad.hpp"
#include <termios.h>
#include <iomanip>

namespace dian
{
class KeyboardMapper
{
public:
    explicit KeyboardMapper(Vec3 command_limits):limits(command_limits){}
    PadOutput Input(char key)
    {
        PadRequest request=PadRequest::None;
        // Ignore terminal escape sequences (arrows/function keys), including their digits.
        if(escape==1){escape=(key=='[' || key=='O')?2:0;return Output(request);}
        if(escape==2){if(key>='@' && key<='~')escape=0;return Output(request);}
        if(key==27){escape=1;return Output(request);}
        if(key>='A' && key<='Z')key=key-'A'+'a';
        switch(key)
        {
        case 'm': velocity={};request=PadRequest::Enable;break;
        case '0': velocity={};request=PadRequest::Stand;break;
        case '1': velocity={};request=PadRequest::Policy;break;
        case 'w': velocity[0]+=.1;break;
        case 's': velocity[0]-=.1;break;
        case 'a': velocity[1]+=.05;break;
        case 'd': velocity[1]-=.05;break;
        case 'q': velocity[2]+=.1;break;
        case 'e': velocity[2]-=.1;break;
        case ' ': velocity={};request=PadRequest::Zero;break;
        case 'k': velocity={};request=PadRequest::Disable;break;
        case 'x': case 4: velocity={};request=PadRequest::Disable;quit=true;break;
        default:return Output(request);
        }
        changed=true;
        for(int i=0;i<3;++i)velocity[i]=std::clamp(velocity[i],-limits[i],limits[i]);
        return Output(request);
    }
    PadOutput Output(PadRequest request=PadRequest::None) const
    {
        PadOutput out;out.ready=true;out.request=request;
        for(int i=0;i<3;++i)out.demand[i]=velocity[i]/limits[i];
        return out;
    }
    Vec3 velocity{};
    bool quit=false,changed=false;
private:
    Vec3 limits;
    int escape=0;
};
class Keyboard
{
public:
    explicit Keyboard(Vec3 limits,int input=STDIN_FILENO):mapper(limits),fd(input)
    {
        if(!isatty(fd) || tcgetattr(fd,&original)<0)
            throw std::runtime_error("Keyboard mode needs a terminal; connect with ssh -t");
        original_flags=fcntl(fd,F_GETFL,0);
        if(original_flags<0)throw std::runtime_error("Cannot read terminal flags");
        auto settings=original;
        settings.c_lflag&=~(ICANON|ECHO);
        settings.c_lflag|=ISIG;
        settings.c_cc[VMIN]=0;settings.c_cc[VTIME]=0;
        settings.c_cc[VSUSP]=_POSIX_VDISABLE; // Do not suspend a live motor controller with Ctrl+Z.
        if(tcsetattr(fd,TCSANOW,&settings)<0)throw std::runtime_error("Cannot configure keyboard terminal");
        if(fcntl(fd,F_SETFL,original_flags|O_NONBLOCK)<0)
        {
            tcsetattr(fd,TCSANOW,&original);
            throw std::runtime_error("Cannot configure nonblocking keyboard input");
        }
        tcflush(fd,TCIFLUSH);
    }
    ~Keyboard()
    {
        tcsetattr(fd,TCSANOW,&original);
        fcntl(fd,F_SETFL,original_flags);
    }
    Keyboard(const Keyboard&)=delete;
    Keyboard& operator=(const Keyboard&)=delete;
    PadOutput Poll()
    {
        pollfd descriptor{fd,POLLIN,0};
        if(poll(&descriptor,1,0)<0 && errno!=EINTR)throw std::runtime_error("Keyboard terminal poll failed");
        if(descriptor.revents&(POLLHUP|POLLERR|POLLNVAL))throw std::runtime_error("SSH/keyboard terminal disconnected");
        auto result=mapper.Output();
        if(!(descriptor.revents&POLLIN))return result;
        char bytes[64];const auto count=read(fd,bytes,sizeof(bytes));
        if(count<0 && (errno==EAGAIN||errno==EINTR))return result;
        if(count<=0)throw std::runtime_error("SSH/keyboard terminal closed");
        bool stop=false,zero=false;
        mapper.changed=false;
        for(int i=0;i<count;++i)
        {
            auto next=mapper.Input(bytes[i]);
            stop=stop || next.request==PadRequest::Disable;
            zero=zero || next.request==PadRequest::Zero;
            if(next.request==PadRequest::None)next.request=result.request;
            result=next;
        }
        // Stop/zero takes precedence over other keys arriving in the same SSH packet.
        if(stop)result=mapper.Input('k');
        else if(zero)result=mapper.Input(' ');
        if(mapper.changed)
            std::cout<<std::fixed<<std::setprecision(3)<<"Keyboard command: vx="<<mapper.velocity[0]
                     <<" vy="<<mapper.velocity[1]<<" yaw="<<mapper.velocity[2]<<'\n';
        return result;
    }
    bool QuitRequested() const{return mapper.quit;}
    static void Help()
    {
        std::cout<<"Keyboard: m=check/enable, 0=stand, 1=RL, w/s=vx, a/d=vy, q/e=yaw, Space=zero, k=disable, x=exit.\n"
                 <<"No Enter needed. Speed persists after key release; Space stops movement demand.\n";
    }
private:
    KeyboardMapper mapper;
    int fd,original_flags;
    termios original{};
};
}
