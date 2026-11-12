/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "rl_sdk.hpp"
#include "dian_hardware.hpp"
#include "dian_gamepad.hpp"
#include "dian_keyboard.hpp"
#include <csignal>

struct DianTrace;
class RLRealDian : public RL
{
public:
    RLRealDian(const std::filesystem::path& root,const std::string& mode,bool arm,dian::Vec3 command,
               const std::string& policy_name="student227",const std::string& gamepad_path="",
               const std::string& model_path="",const std::string& record_path="");
    ~RLRealDian();
    void Run(double duration);
    std::vector<float> Forward() override;
    void GetState(RobotState<float>* state) override;
    void SetCommand(const RobotCommand<float>* command) override;
    void ApplyOperator(const dian::PadOutput& pad);
    void BeginHold();
    void BeginStand();
    void StandStep();
    void BeginPolicy();
    void PolicyStep();
    void Stop();
    bool stand_done=false, start_requested=false, interactive=false;
    dian::PadRequest operator_request=dian::PadRequest::None;
    std::string run_mode;
    dian::Config config;
    dian::PolicyContract contract;
    static volatile std::sig_atomic_t interrupted;
private:
    void Refresh();
    void SetTarget(const dian::Joints& target);
    void WriteTrace();
    void NoteStop(const std::string& reason);
    std::unique_ptr<DianTrace> trace;
    std::filesystem::path record_directory;
    double trace_t0=0;
    std::unique_ptr<dian::Gamepad> gamepad;
    std::unique_ptr<dian::Keyboard> keyboard;
    std::unique_ptr<dian::Hardware> hardware;
    dian::Sample sample;
    dian::Vec3 command;
    dian::Joints stand_start{};
    double stand_time=0,enabled_time=0;
    unsigned long control_tick=0;
    bool enabled=false,send_authorized=false;
};
