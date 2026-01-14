// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>  // 新增头文件
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class CartesianPoseExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  // 订阅器
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_move_subscriber_;

  Eigen::Quaterniond orientation_;
  Eigen::Quaterniond target_orientation_;
  Eigen::Vector3d position_;
  Eigen::Vector3d final_pos_;
  Eigen::Quaterniond final_orientation_;

  const bool k_elbow_activated_{false};
  bool init_flag_{true};

  double elapsed_time_{0.0};
  double start_time_{0.0};
  double current_time_{0.0};
  std::string robot_description_;
  std::string arm_id_;

  // 消息回调函数
  void cartesianMoveCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  bool checkForNewTarget();

  double target_x_;
  double target_y_;
  double target_z_;
  double target_quat_w_;
  double target_quat_x_;
  double target_quat_y_;
  double target_quat_z_;

  bool is_moving_;
  bool move_complete_;
  double movement_duration_;

  // 线程安全的目标缓冲区
  std::mutex target_mutex_;
  bool pending_target_{false};
  double pending_x_;
  double pending_y_;
  double pending_z_;
  double pending_quat_w_;
  double pending_quat_x_;
  double pending_quat_y_;
  double pending_quat_z_;
  double pending_duration_;
};

}  // namespace franka_example_controllers