/* Copyright (c) 2024-2026 Ziqi Fan; SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "dian_config.hpp"
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <iostream>

namespace dian
{
enum class PadRequest {None,Enable,Stand,Policy,Zero,Disable};
struct PadOutput {Vec3 demand{};PadRequest request=PadRequest::None;bool ready=false;};
struct PadConfig
{
    std::string device;
    std::array<int,3> axes{1,0,3},signs{-1,-1,-1};
    std::array<int,6> buttons{0,1,2,3,4,5};
    int dpad_vertical=7;
    double deadzone=.1;
    static PadConfig Load(const std::filesystem::path& path)
    {
        auto n=YAML::LoadFile(path.string());PadConfig c;
        c.device=n["device"].as<std::string>();c.deadzone=n["deadzone"].as<double>();
        auto axes=n["axes"].as<std::vector<int>>(),signs=n["signs"].as<std::vector<int>>();
        auto buttons=n["buttons"].as<std::vector<int>>();
        if(axes.size()!=3 || signs.size()!=3 || buttons.size()!=6)throw std::runtime_error("Invalid gamepad mapping size");
        std::copy(axes.begin(),axes.end(),c.axes.begin());std::copy(signs.begin(),signs.end(),c.signs.begin());
        std::copy(buttons.begin(),buttons.end(),c.buttons.begin());c.dpad_vertical=n["dpad_vertical"].as<int>();
        if(!std::isfinite(c.deadzone)||c.deadzone<0||c.deadzone>=.5)throw std::runtime_error("Invalid gamepad deadzone");
        for(int v:c.axes)if(v<0||v>63)throw std::runtime_error("Invalid gamepad axis");
        for(int v:c.signs)if(std::abs(v)!=1)throw std::runtime_error("Invalid gamepad sign");
        for(int v:c.buttons)if(v<0||v>63)throw std::runtime_error("Invalid gamepad button");
        if(c.dpad_vertical<0||c.dpad_vertical>63)throw std::runtime_error("Invalid dpad axis");
        return c;
    }
};
class PadMapper
{
public:
    explicit PadMapper(PadConfig config):cfg(config){}
    PadOutput Update(const std::vector<double>& axes,const std::vector<int>& buttons)
    {
        Finite(axes,"gamepad");
        for(auto v:axes)if(std::abs(v)>1.001)throw std::runtime_error("Gamepad axis outside range");
        std::array<bool,7> held{};
        for(int i=0;i<6;++i)held[i]=buttons.at(cfg.buttons[i])!=0;
        held[6]=axes.at(cfg.dpad_vertical)<-.5;
        Vec3 demand{};
        for(int i=0;i<3;++i)
        {
            const double v=axes.at(cfg.axes[i])*cfg.signs[i];
            demand[i]=std::abs(v)<=cfg.deadzone?0:std::copysign((std::abs(v)-cfg.deadzone)/(1-cfg.deadzone),v);
        }
        std::array<bool,7> rising{};for(int i=0;i<7;++i)rising[i]=held[i]&&!previous[i];previous=held;
        if(!ready)
        {
            ready=Norm(demand)==0 && std::none_of(held.begin(),held.end(),[](bool x){return x;});
            return {{},PadRequest::None,ready};
        }
        PadOutput out{demand,PadRequest::None,true};
        const bool lb=held[4],rb=held[5];
        if((lb&&rb) || (lb&&held[1])) {out.request=PadRequest::Disable;out.demand={};}
        else if(lb&&rising[0])out.request=PadRequest::Enable;
        else if(rb&&rising[6])out.request=PadRequest::Policy;
        else if(!lb&&!rb&&rising[0])out.request=PadRequest::Stand;
        else if(!lb&&!rb&&rising[1]){zero_latched=true;out.request=PadRequest::Zero;}
        if(Norm(demand)==0)zero_latched=false;
        if(zero_latched)out.demand={};
        return out;
    }
private:
    PadConfig cfg;bool ready=false,zero_latched=false;std::array<bool,7> previous{};
};
class Gamepad
{
public:
    explicit Gamepad(PadConfig settings):cfg(settings),mapper(settings)
    {
        fd=open(cfg.device.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
        if(fd<0)throw std::runtime_error("Cannot open gamepad: "+cfg.device);
        unsigned char na=0,nb=0;
        if(ioctl(fd,JSIOCGAXES,&na)<0 || ioctl(fd,JSIOCGBUTTONS,&nb)<0)
        {close(fd);fd=-1;throw std::runtime_error("Invalid joystick device");}
        axes.resize(na);buttons.resize(nb);axes_seen.resize(na);buttons_seen.resize(nb);
        try
        {
            for(auto index:cfg.axes)axes.at(index);axes.at(cfg.dpad_vertical);
            for(auto index:cfg.buttons)buttons.at(index);
        }
        catch(...){close(fd);fd=-1;throw std::runtime_error("Gamepad layout does not match config");}
    }
    ~Gamepad(){if(fd>=0)close(fd);}
    PadOutput Poll()
    {
        pollfd descriptor{fd,POLLIN,0};
        if(poll(&descriptor,1,0)<0 && errno!=EINTR)throw std::runtime_error("Gamepad poll failed");
        if(descriptor.revents&(POLLHUP|POLLERR|POLLNVAL))throw std::runtime_error("Gamepad disconnected");
        js_event event{};PadOutput result;
        bool got=false;
        for(int count=0;count<256;++count)
        {
            const auto n=read(fd,&event,sizeof(event));
            if(n<0 && (errno==EAGAIN||errno==EINTR))break;
            if(n!=sizeof(event))throw std::runtime_error("Gamepad disconnected/read failed");
            const auto type=event.type&~JS_EVENT_INIT;
            if(type==JS_EVENT_AXIS && event.number<axes.size()){axes[event.number]=std::clamp(event.value/32767.,-1.,1.);axes_seen[event.number]=true;}
            if(type==JS_EVENT_BUTTON && event.number<buttons.size()){buttons[event.number]=event.value?1:0;buttons_seen[event.number]=true;}
            // Startup initialization updates levels only, never requests motor motion.
            if((event.type&JS_EVENT_INIT)||!Initialized())continue;
            auto next=mapper.Update(axes,buttons);got=true;
            if(result.request==PadRequest::Disable)next.request=PadRequest::Disable;
            else if(next.request==PadRequest::None)next.request=result.request;
            result=next;
        }
        if(!Initialized())return {};
        if(!got)result=mapper.Update(axes,buttons);
        return result;
    }
    void Print() const
    {
        std::cout<<"axes:";for(auto v:axes)std::cout<<' '<<v;
        std::cout<<" buttons:";for(size_t i=0;i<buttons.size();++i)if(buttons[i])std::cout<<' '<<i;
        std::cout<<'\n';
    }
private:
    bool Initialized() const
    {
        for(int i:cfg.axes)if(!axes_seen[i])return false;
        for(int i:cfg.buttons)if(!buttons_seen[i])return false;
        return axes_seen[cfg.dpad_vertical];
    }
    std::vector<bool> axes_seen,buttons_seen;
    PadConfig cfg;PadMapper mapper;int fd=-1;std::vector<double> axes;std::vector<int> buttons;
};
}
