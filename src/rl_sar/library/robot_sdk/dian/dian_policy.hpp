/* Copyright (c) 2024-2026 Ziqi Fan; SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "observation_buffer.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace dian
{
using Joints = std::array<double, 12>;
using Vec3 = std::array<double, 3>;
inline double Now()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
template <typename Range> inline void Finite(const Range& values, const char* name)
{
    for(auto value:values)if(!std::isfinite(value))throw std::runtime_error(std::string("Non-finite ")+name);
}
template <typename Range> inline double Norm(const Range& values)
{
    double sum=0;for(auto value:values)sum+=value*value;return std::sqrt(sum);
}
struct Sample
{
    Joints q{},dq{},torque{},temperature{},motor_time{};
    Vec3 gyro{},gravity{0,0,-1},acceleration{}; // Body m/s², includes gravity.
    std::array<double,4> quaternion{1,0,0,0};
    std::array<int,12> fault{},mode{};
    double imu_time=0;
};
struct PolicyConfig
{
    Joints default_q{},scale{},lower{},upper{};
    Vec3 target_velocity{4,8,6};
    double dt=.02,settle=1.,quiet_scale=.1,time_constant=.04;
};
class PolicyContract
{
public:
    explicit PolicyContract(PolicyConfig config):cfg(config),target(config.default_q){}
    void Reset()
    {
        Reset(cfg.default_q, 0.0);
    }
    void Reset(const Joints& previous_target, double previous_standing_age)
    {
        target=previous_target;histories.clear();tick=0;standing_age=previous_standing_age;last_action.fill(0);
    }
    std::vector<float> Observe(const Sample& state,const Vec3& command)
    {
        std::vector<float> out;
        AppendHistory(out,"gyro",state.gyro,100.,.25);
        AppendHistory(out,"gravity",state.gravity,100.,1.);
        AppendHistory(out,"command",command,100.,1.);
        Joints relative{};
        for(int i=0;i<12;++i)relative[i]=state.q[i]-cfg.default_q[i];
        AppendHistory(out,"q",relative,100.,1.);
        AppendHistory(out,"dq",state.dq,100.,.05);
        AppendHistory(out,"action",last_action,100.,1.);
        // Commanded stopping changes the output phase, not the episode clock.
        const double angle=2*std::acos(-1.)*tick*cfg.dt/.6;
        out.push_back(Norm(command)>=.1?std::sin(angle):0);
        out.push_back(Norm(command)>=.1?std::cos(angle):1);
        ++tick;Finite(out,"observation");return out;
    }
    Joints Process(const std::vector<float>& action,const Vec3& command)
    {
        if(action.size()!=12)throw std::runtime_error("DIAN action must have 12 elements");
        Finite(action,"action");
        standing_age=Norm(command)<=.1?standing_age+cfg.dt:0;
        const double factor=standing_age>=cfg.settle-1e-6?cfg.quiet_scale:1.;
        const double alpha=cfg.dt/(cfg.time_constant/factor+cfg.dt);
        for(int i=0;i<12;++i)
        {
            last_action[i]=action[i];
            last_requested[i]=cfg.default_q[i]+cfg.scale[i]*action[i];
            const double limit=cfg.target_velocity[i%3]*cfg.dt*factor;
            const double requested=std::clamp(last_requested[i],cfg.lower[i],cfg.upper[i]);
            target[i]+=std::clamp((requested-target[i])*alpha,-limit,limit);
        }
        return target;
    }
    PolicyConfig cfg;
    Joints target;
    const Joints& RawAction() const { return last_action; }
    const Joints& Requested() const { return last_requested; }
private:
    template <typename Range>void AppendHistory(std::vector<float>& out,const std::string& key,
                                               const Range& values,double clip,double scale)
    {
        std::vector<float> current;
        for(double v:values)current.push_back(std::clamp(v,-clip,clip)*scale);
        auto found=histories.find(key);
        if(found==histories.end())
        {
            found=histories.emplace(key,ObservationBuffer(1,{static_cast<int>(current.size())},5,"time")).first;
            found->second.reset({0},current);
        }
        else found->second.insert(current);
        auto history=found->second.get_obs_vec({4,3,2,1,0});
        out.insert(out.end(),history.begin(),history.end());
    }
    std::map<std::string,ObservationBuffer> histories;
    Joints last_action{},last_requested{};
    unsigned long tick=0;
    double standing_age=0;
};
}
