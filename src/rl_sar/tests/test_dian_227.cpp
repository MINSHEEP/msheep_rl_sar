/* Copyright (c) 2024-2026 Ziqi Fan; SPDX-License-Identifier: Apache-2.0 */
#include "dian_config.hpp"
#include <iostream>
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
template<class Range>double compare(const Range& got,const YAML::Node& expected)
{
    require(got.size()==expected.size(),"Shape mismatch");double error=0;
    for(size_t i=0;i<got.size();++i)
    {
        const double delta=std::abs(got[i]-expected[i].as<double>());
        if(!std::isfinite(got[i])||delta>2e-6)throw std::runtime_error("227 reference mismatch at "+std::to_string(i));
        error=std::max(error,delta);
    }
    return error;
}
int main(int argc,char** argv)
{
    try
    {
        require(argc==3,"Expected policy root and reference fixture");
        auto cfg=dian::Config::Load(argv[1],"student227");
        require(cfg.observation_size==227,"Wrong dimension");
        dian::PolicyContract contract(cfg.policy);dian::Sample sample;
        // A policy entered after standing inherits the last target and the existing quiet filter.
        auto previous=cfg.policy.default_q;previous[0]=0.1;
        contract.Reset(previous,cfg.policy.settle);
        const auto quiet=contract.Process(std::vector<float>(12,1.0f),{});
        require(std::abs(quiet[0]-previous[0])<=cfg.policy.target_velocity[0]*cfg.policy.dt*cfg.policy.quiet_scale+1e-9,
                "Standing takeover ignored quiet slew limit");
        require(quiet[0]>previous[0],"Standing takeover did not continue from last target");
        const auto moving=contract.Process(std::vector<float>(12,1.0f),{0.2,0,0});
        require(moving[0]-quiet[0]>quiet[0]-previous[0],"Moving command did not restore normal response");
        contract.Reset();
        const auto frames=YAML::LoadFile(argv[2])["frames"];double obs_error=0,target_error=0;
        for(const auto& n:frames)
        {
            sample.q=dian::Array<12>(n["q"]);sample.dq=dian::Array<12>(n["dq"]);
            sample.gyro=dian::Array<3>(n["gyro"]);sample.gravity=dian::Array<3>(n["gravity"]);
            auto command=dian::Array<3>(n["command"]);
            obs_error=std::max(obs_error,compare(contract.Observe(sample,command),n["observation"]));
            target_error=std::max(target_error,compare(contract.Process(n["action"].as<std::vector<float>>(),command),n["target"]));
        }
        contract.Reset();sample.q=cfg.policy.default_q;sample.dq.fill(0);sample.gyro.fill(0);sample.gravity={0,0,-1};
        const auto initial=contract.Observe(sample,{});
        for(int i=165;i<225;++i)require(initial[i]==0,"Reset raw action history");
        auto watchdog=cfg.protocol.WriteParameter(5,0x7028,2000);
        require(watchdog.id==0x1200fd05 && watchdog.data==std::array<uint8_t,8>{0x28,0x70,0,0,0xd0,7,0,0},"Watchdog frame");
        std::cout<<"PASS: "<<frames.size()<<" 227 reference frames, no estimator inputs; obs error="<<obs_error
                 <<" target error="<<target_error<<'\n';return 0;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
