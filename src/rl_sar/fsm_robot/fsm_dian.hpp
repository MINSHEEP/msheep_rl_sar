/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "rl_real_dian.hpp"

namespace dian_fsm
{
class State : public RLFSMState
{
public:
    State(RLRealDian* app,const std::string& name):RLFSMState(*app,name),app(*app){}
    void Enter() override
    {
        std::cout<<LOGGER::INFO<<"[DIAN FSM] "<<state_name_<<std::endl;
        if(state_name_=="RLFSMStatePassive")app.Stop();
        if(state_name_=="RLFSMStateHold")app.BeginHold();
        if(state_name_=="RLFSMStateGetUp")app.BeginStand();
        if(state_name_=="RLFSMStateRLLocomotion")app.BeginPolicy();
        if(state_name_=="RLFSMStateStop")app.Stop();
    }
    void Run() override
    {
        if(state_name_=="RLFSMStateGetUp")app.StandStep();
        if(state_name_=="RLFSMStateRLLocomotion")app.PolicyStep();
    }
    void Exit() override{}
    std::string CheckChange() override
    {
        if(app.interactive)
        {
            if(app.operator_request==dian::PadRequest::Disable)return "RLFSMStatePassive";
            if(state_name_=="RLFSMStatePassive" && app.operator_request==dian::PadRequest::Enable)
                return "RLFSMStateHold";
            if(state_name_!="RLFSMStatePassive" && app.operator_request==dian::PadRequest::Stand)
                return "RLFSMStateGetUp";
            if(state_name_=="RLFSMStateGetUp" && app.stand_done && app.operator_request==dian::PadRequest::Policy)
                return "RLFSMStateRLLocomotion";
            return state_name_;
        }
        if(state_name_=="RLFSMStatePassive" && app.start_requested)return "RLFSMStateGetUp";
        if(state_name_=="RLFSMStateGetUp" && app.stand_done && app.run_mode!="stand")return "RLFSMStateRLLocomotion";
        return state_name_;
    }
private:
    RLRealDian& app;
};
class Factory : public FSMFactory
{
public:
    explicit Factory(const std::string& initial):initial(initial){}
    std::shared_ptr<FSMState> CreateState(void* context,const std::string& name) override
    {return std::make_shared<State>(static_cast<RLRealDian*>(context),name);}
    std::string GetType() const override{return "dian";}
    std::vector<std::string> GetSupportedStates() const override
    {return {"RLFSMStatePassive","RLFSMStateHold","RLFSMStateGetUp","RLFSMStateRLLocomotion","RLFSMStateStop"};}
    std::string GetInitialState() const override{return initial;}
private:
    std::string initial;
};
REGISTER_FSM_FACTORY(Factory,"RLFSMStatePassive")
}
