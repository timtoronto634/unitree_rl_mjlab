// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <mutex>
#include <chrono>
#include <unistd.h>

#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/LowCmd.h>

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

// ---------------------------------------------------------------------------
// K1LowStateMsg
//
// Thin wrapper around booster_interface::msg::LowState that adds a
// motor_state() alias (required by the shared State_Passive header which
// calls lowstate->msg_.motor_state()[i].q()).
// ---------------------------------------------------------------------------
struct K1LowStateMsg {
    booster_interface::msg::LowState inner;

    const booster_interface::msg::ImuState& imu_state() const { return inner.imu_state(); }
    booster_interface::msg::ImuState&       imu_state()       { return inner.imu_state(); }

    const std::vector<booster_interface::msg::MotorState>& motor_state_parallel() const { return inner.motor_state_parallel(); }
    std::vector<booster_interface::msg::MotorState>&       motor_state_parallel()       { return inner.motor_state_parallel(); }

    // Compatibility alias used by shared FSM headers (State_Passive::run)
    const std::vector<booster_interface::msg::MotorState>& motor_state() const { return inner.motor_state_parallel(); }
    std::vector<booster_interface::msg::MotorState>&       motor_state()       { return inner.motor_state_parallel(); }
};

// ---------------------------------------------------------------------------
// K1LowState
// ---------------------------------------------------------------------------
class K1LowState {
public:
    std::mutex   mutex_;
    K1LowStateMsg msg_;

    // Stub joystick — always zero (transitions driven by keyboard instead).
    UnitreeJoystick joystick{};

    using SharedPtr = std::shared_ptr<K1LowState>;

    K1LowState()
    : subscriber_(booster::robot::b1::kTopicLowState,
                  [this](const void* raw) {
                      auto* ls = static_cast<const booster_interface::msg::LowState*>(raw);
                      std::lock_guard<std::mutex> lock(mutex_);
                      msg_.inner = *ls;
                      last_msg_time_ = std::chrono::steady_clock::now();
                  })
    {
        subscriber_.InitChannel();
    }

    void wait_for_connection() {
        while (isTimeout()) {
            usleep(10000); // 10 ms
        }
    }

    bool isTimeout() {
        std::lock_guard<std::mutex> lock(mutex_);
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_msg_time_).count();
        return elapsed > 1000; // 1 second
    }

    // No-op: state is updated continuously by DDS callback.
    void update() {}

private:
    booster::robot::ChannelSubscriber<booster_interface::msg::LowState> subscriber_;
    std::chrono::steady_clock::time_point last_msg_time_{};
};

// ---------------------------------------------------------------------------
// K1LowCmd
// ---------------------------------------------------------------------------
class K1LowCmd {
public:
    booster_interface::msg::LowCmd msg_;

    K1LowCmd()
    : publisher_(booster::robot::b1::kTopicJointCtrl)
    {
        // Pre-allocate 23 motor command slots (kJointCnt from b1_api_const.hpp)
        msg_.motor_cmd().resize(booster::robot::b1::kJointCnt);
        msg_.cmd_type() = booster_interface::msg::PARALLEL;
        publisher_.InitChannel();
    }

    void unlockAndPublish() {
        publisher_.Write(&msg_);
    }

    // No mode_machine concept in Booster SDK — always succeed.
    bool check_mode_machine(const std::shared_ptr<K1LowState>& /*lowstate*/) {
        return true;
    }

private:
    booster::robot::ChannelPublisher<booster_interface::msg::LowCmd> publisher_;
};
