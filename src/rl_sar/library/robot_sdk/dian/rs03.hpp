/* SPDX-License-Identifier: Apache-2.0
 * RobStride RS03 extended CAN protocol. Ranges must match the installed firmware. */
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace dian
{
struct Frame { uint32_t id=0; std::array<uint8_t,8> data{}; };
struct Feedback { int id=0, fault=0, mode=0; double q=0,dq=0,torque=0,temperature=0; };
class RS03
{
public:
    int master=253;
    double position=12.57,velocity=20,torque=60,kp=5000,kd=100;
    static uint16_t Encode(double x,double low,double high)
    {
        if (!std::isfinite(x) || x<low || x>high) throw std::runtime_error("RS03 command outside protocol range");
        return static_cast<uint16_t>((x-low)*65535/(high-low));
    }
    static double Decode(uint16_t x,double limit) { return x*(2*limit)/65535-limit; }
    Frame Command(int id,double q,double dq,double k,double d,double tau=0) const
    {
        Frame f; f.id=(1u<<24) | (static_cast<uint32_t>(Encode(tau,-torque,torque))<<8) | id;
        const std::array<uint16_t,4> words{Encode(q,-position,position),Encode(dq,-velocity,velocity),
                                          Encode(k,0,kp),Encode(d,0,kd)};
        for (int i=0;i<4;++i) { f.data[2*i]=words[i]>>8; f.data[2*i+1]=words[i]&255; }
        return f;
    }
    Frame Service(int id,int kind) const { Frame f; f.id=(kind<<24)|(master<<8)|id; return f; }
    Frame ReadParameter(int id,uint16_t index) const
    {
        auto f=Service(id,17);f.data[0]=index&255;f.data[1]=index>>8;return f;
    }
    Frame WriteParameter(int id,uint16_t index,uint32_t bits) const
    {
        auto f=ReadParameter(id,index);f.id=(f.id&0xffffff)|(18u<<24);
        for(int i=0;i<4;++i)f.data[4+i]=(bits>>(i*8))&255;
        return f;
    }
    Frame MotionMode(int id) const { auto f=Service(id,18); f.data[0]=5;f.data[1]=0x70;return f; }
    bool Parse(const Frame& f, Feedback& out) const
    {
        if ((f.id&255)!=static_cast<uint32_t>(master)) return false;
        if (((f.id>>24)&31)==21) throw std::runtime_error("RS03 fault response type 21");
        if (((f.id>>24)&31)!=2) return false;
        out.id=(f.id>>8)&255;out.fault=(f.id>>16)&63;out.mode=(f.id>>22)&3;
        auto word=[&](int i){ return static_cast<uint16_t>((f.data[2*i]<<8)|f.data[2*i+1]); };
        out.q=Decode(word(0),position);out.dq=Decode(word(1),velocity);
        out.torque=Decode(word(2),torque);out.temperature=word(3)/10.;return true;
    }
};
}
