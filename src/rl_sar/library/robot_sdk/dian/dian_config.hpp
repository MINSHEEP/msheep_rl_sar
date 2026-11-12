/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "dian_policy.hpp"
#include "rs03.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <set>
#include <utility>

namespace dian
{
template <size_t N> std::array<double,N> Array(const YAML::Node& node)
{
    if (!node.IsSequence() || node.size()!=N) throw std::runtime_error("Wrong configuration array size");
    std::array<double,N> out{};
    for (size_t i=0;i<N;++i) out[i]=node[i].as<double>();
    Finite(out,"configuration");return out;
}
struct Joint { int device=0,id=1,sign=1;double zero=0,kp=50,kd=1; };
struct Config
{
    PolicyConfig policy;
    RS03 protocol;
    std::array<Joint,12> joints;
    std::vector<std::string> devices;
    std::vector<std::string> usb_paths;
    std::string imu_device,model_path;
    int imu_baud=115200;
    std::array<double,9> mount{1,0,0,0,1,0,0,0,1};
    double timeout=.1, max_temperature=65, max_velocity=15,max_torque=25,max_tilt=.6981317;
    double startup_timeout=3,stand_duration=3,max_cycle=.04;
    Vec3 command_limits{.5,.2,.5};
    int observation_size=227;
    std::string policy_name;
    static Config Load(const std::filesystem::path& policy_root,const std::string& policy_name="student227")
    {
        Config c;c.policy_name=policy_name;
        const auto base=YAML::LoadFile((policy_root/"dian/base.yaml").string())["dian"];
        const auto p=YAML::LoadFile((policy_root/"dian"/policy_name/"config.yaml").string())["dian/"+policy_name];
        const auto h=YAML::LoadFile((policy_root/"dian/hardware.yaml").string())["dian"];
        const auto profile=p["observation_profile"].as<std::string>();
        if(profile!="proprio227_v1" ||
           p["num_observations"].as<int>()!=c.observation_size || base["num_of_dofs"].as<int>()!=12 ||
           std::abs(base["dt"].as<double>()*base["decimation"].as<int>()-.02)>1e-9)
            throw std::runtime_error("Unsupported DIAN policy contract");
        if(p["standing_contact_aware"].as<bool>() || p["standing_layout_tolerance"].as<double>()!=0 || p["standing_level_tolerance"].as<double>()!=0)
            throw std::runtime_error("227 policy must use contact-independent filtering");
        c.policy.default_q=Array<12>(base["default_dof_pos"]);c.policy.scale=Array<12>(p["action_scale"]);
        c.policy.lower=Array<12>(p["target_lower"]);c.policy.upper=Array<12>(p["target_upper"]);
        c.policy.target_velocity=Array<3>(p["max_target_velocity"]);
        c.policy.settle=p["standing_settle_time"].as<double>();c.policy.quiet_scale=p["standing_velocity_scale"].as<double>();
        c.policy.time_constant=p["target_time_constant"].as<double>();
        c.model_path=(policy_root/"dian"/policy_name/p["model_name"].as<std::string>()).string();
        const auto r=h["rs03"];
        c.protocol.master=r["master_id"].as<int>();c.protocol.position=r["position_range"].as<double>();
        c.protocol.velocity=r["velocity_range"].as<double>();c.protocol.torque=r["torque_range"].as<double>();
        Finite(std::array<double,3>{c.protocol.position,c.protocol.velocity,c.protocol.torque},"RS03 ranges");
        if(c.protocol.master<0 || c.protocol.master>255 || c.protocol.position<=0 || c.protocol.velocity<=0 || c.protocol.torque<=0)
            throw std::runtime_error("Invalid RS03 ranges/master ID");
        c.devices=h["devices"].as<std::vector<std::string>>();
        if(c.devices.empty() || std::set<std::string>(c.devices.begin(),c.devices.end()).size()!=c.devices.size())
            throw std::runtime_error("Missing/duplicate CAN interfaces");
        c.usb_paths=h["usb_paths"].as<std::vector<std::string>>();
        if(c.usb_paths.size()!=c.devices.size())throw std::runtime_error("USB port mapping is required");
        c.imu_device=h["imu"]["device"].as<std::string>();c.imu_baud=h["imu"]["baud"].as<int>();
        c.mount=Array<9>(h["imu"]["R_body_sensor"]);
        if(h["imu"]["protocol"].as<std::string>()!="hi91_serial")throw std::runtime_error("Expected HIPNUC HI91");
        const auto safety=h["safety"];
        c.timeout=safety["feedback_timeout"].as<double>();c.startup_timeout=safety["startup_timeout"].as<double>();
        c.stand_duration=safety["stand_duration"].as<double>();
        c.max_cycle=safety["max_cycle"].as<double>();c.max_temperature=safety["max_temperature"].as<double>();
        c.max_velocity=safety["max_velocity"].as<double>();c.max_torque=safety["max_torque"].as<double>();
        c.max_tilt=safety["max_tilt"].as<double>();c.command_limits=Array<3>(safety["command_limits"]);
        const auto gains=Array<12>(p["rl_kp"]), damping=Array<12>(p["rl_kd"]);
        std::set<std::pair<int,int>> addresses;
        const std::array<std::string,4> legs{"FR","FL","RR","RL"};
        if (h["joints"].size()!=12 || base["joint_names"].size()!=12) throw std::runtime_error("Expected 12 joint mappings");
        for (int i=0;i<12;++i)
        {
            const auto node=h["joints"][i];auto& j=c.joints[i];
            const auto expected=legs[i/3]+"_J"+std::to_string(i%3+1);
            if (node["name"].as<std::string>()!=expected || base["joint_names"][i].as<std::string>()!=expected)
                throw std::runtime_error("DIAN joint order mismatch");
            j.device=node["device"].as<int>();j.id=node["id"].as<int>();
            j.sign=node["sign"].as<int>();j.zero=node["zero"].as<double>();j.kp=gains[i];j.kd=damping[i];
            if (j.device<0 || j.device>=static_cast<int>(c.devices.size()) ||
                j.id<1 || j.id>127 || std::abs(j.sign)!=1 || !addresses.emplace(j.device,j.id).second)
                throw std::runtime_error("Invalid/duplicate CAN address or sign");
            if (!(c.policy.lower[i]<c.policy.default_q[i] && c.policy.default_q[i]<c.policy.upper[i]) || c.policy.scale[i]<=0)
                throw std::runtime_error("Invalid joint target limits/scale");
            c.protocol.Command(j.id,j.zero+j.sign*c.policy.lower[i],0,j.kp,j.kd);
            c.protocol.Command(j.id,j.zero+j.sign*c.policy.upper[i],0,j.kp,j.kd);
        }
        for (double x : {c.timeout,c.startup_timeout,c.stand_duration,c.max_cycle,c.max_temperature,
                         c.max_velocity,c.max_torque,c.max_tilt,c.policy.settle,c.policy.quiet_scale,
                         c.policy.time_constant})
            if (!std::isfinite(x) || x<=0) throw std::runtime_error("Invalid timing/safety/filter setting");
        if(c.policy.quiet_scale>1 || c.max_tilt>=1.57)throw std::runtime_error("Invalid tilt/quiet factor");
        if(c.protocol.master<0 || c.protocol.master>255 || c.protocol.position<=0 || c.protocol.velocity<=0 || c.protocol.torque<=0)
            throw std::runtime_error("Invalid RS03 ranges/master ID");
        for(double x:c.policy.target_velocity) if(x<=0) throw std::runtime_error("Invalid target velocity");
        for(double x:c.command_limits) if(x<=0) throw std::runtime_error("Invalid command limit");
        for(int i=0;i<3;++i) for(int j=0;j<3;++j)
        {
            double dot=0;for(int k=0;k<3;++k)dot+=c.mount[3*i+k]*c.mount[3*j+k];
            if(std::abs(dot-(i==j?1:0))>1e-6)throw std::runtime_error("Invalid IMU rotation");
        }
        const auto& m=c.mount;
        const double det=m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6]);
        if(std::abs(det-1)>1e-6)throw std::runtime_error("IMU rotation must be right-handed");
        return c;
    }

};
inline void CheckState(const Config& c,const Sample& s,double now,bool running)
{
    Finite(s.q,"q");Finite(s.dq,"dq");Finite(s.torque,"torque");Finite(s.temperature,"temperature");
    Finite(s.motor_time,"motor time");Finite(s.gyro,"gyro");Finite(s.gravity,"gravity");
    if(!std::isfinite(s.imu_time) || s.imu_time<=0 || now-s.imu_time>c.timeout || now<s.imu_time)
        throw std::runtime_error("Stale IMU");
    // Projected gravity is a unit vector. Attitude, joint position, temperature, speed and
    // torque are not interlocks: policy target bounds stay a command clamp only.
    if(std::abs(Norm(s.gravity)-1)>0.02)throw std::runtime_error("Invalid gravity");
    for(int i=0;i<12;++i)
    {
        if(s.motor_time[i]<=0 || now-s.motor_time[i]>c.timeout || now<s.motor_time[i])throw std::runtime_error("Stale motor feedback");
        if(s.fault[i] || (running && s.mode[i]!=2))throw std::runtime_error("Motor fault/not running");
    }
}
}
