// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <cmath>
#include "isaaclab/assets/articulation/articulation.h"

namespace booster
{

template <typename LowStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_)
    : lowstate(lowstate_)
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        // base_angular_velocity
        for(int i(0); i < 3; i++) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyro()[i];
        }
        // RPY -> quaternion (ZYX Euler convention)
        float r = lowstate->msg_.imu_state().rpy()[0];
        float p = lowstate->msg_.imu_state().rpy()[1];
        float y = lowstate->msg_.imu_state().rpy()[2];
        float qw = std::cos(r/2)*std::cos(p/2)*std::cos(y/2) + std::sin(r/2)*std::sin(p/2)*std::sin(y/2);
        float qx = std::sin(r/2)*std::cos(p/2)*std::cos(y/2) - std::cos(r/2)*std::sin(p/2)*std::sin(y/2);
        float qy = std::cos(r/2)*std::sin(p/2)*std::cos(y/2) + std::sin(r/2)*std::cos(p/2)*std::sin(y/2);
        float qz = std::cos(r/2)*std::cos(p/2)*std::sin(y/2) - std::sin(r/2)*std::sin(p/2)*std::cos(y/2);
        data.root_quat_w = Eigen::Quaternionf(qw, qx, qy, qz);
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        // joint positions and velocities from parallel motor array
        for(int i(0); i < (int)data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state_parallel()[data.joint_ids_map[i]].q();
            data.joint_vel[i] = lowstate->msg_.motor_state_parallel()[data.joint_ids_map[i]].dq();
        }
    }

    LowStatePtr lowstate;
};

} // namespace booster
