
#include <franka_example_controllers/cartesian_pose_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <random>  // 新增头文件
#include <string>

#include <algorithm>
#include <fstream>
#include <sstream>

#include <std_msgs/msg/float64_multi_array.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

void CartesianPoseExampleController::cartesianMoveCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  // 期望消息格式: [x, y, z, qw, qx, qy, qz, duration]
  if (msg->data.size() >= 8) {
    double x = msg->data[0];
    double y = msg->data[1];
    double z = msg->data[2];
    double qw = msg->data[3];
    double qx = msg->data[4];
    double qy = msg->data[5];
    double qz = msg->data[6];
    double duration = msg->data[7];

    // 将数据存储到缓冲区，等待运动完成后处理
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      pending_target_ = true;
      pending_x_ = x;
      pending_y_ = y;
      pending_z_ = z;
      pending_quat_w_ = qw;
      pending_quat_x_ = qx;
      pending_quat_y_ = qy;
      pending_quat_z_ = qz;
      pending_duration_ = duration;
    }

    RCLCPP_INFO(get_node()->get_logger(),
                "Received target via topic: Pos[%.3f, %.3f, %.3f], "
                "Ori[%.3f, %.3f, %.3f, %.3f], Duration: %.3f",
                x, y, z, qw, qx, qy, qz, duration);
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Received message with invalid size: %zu (expected at least 8)", msg->data.size());
  }
}

bool CartesianPoseExampleController::checkForNewTarget() {
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (pending_target_) {
    pending_target_ = false;
    target_x_ = pending_x_;
    target_y_ = pending_y_;
    target_z_ = pending_z_;
    target_quat_w_ = pending_quat_w_;
    target_quat_x_ = pending_quat_x_;
    target_quat_y_ = pending_quat_y_;
    target_quat_z_ = pending_quat_z_;
    movement_duration_ = pending_duration_;
    return true;
  }
  return false;
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (init_flag_) {
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    start_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = 0.0;

    RCLCPP_INFO(get_node()->get_logger(),
                "Initial position: [%.3f, %.3f, %.3f], orientation: [%.3f, %.3f, %.3f, %.3f]",
                position_.x(), position_.y(), position_.z(), orientation_.w(), orientation_.x(),
                orientation_.y(), orientation_.z());

    // 检查是否有初始目标
    if (checkForNewTarget()) {
      target_orientation_ =
          Eigen::Quaterniond(target_quat_w_, target_quat_x_, target_quat_y_, target_quat_z_);
      if (target_orientation_.norm() > 0) {
        target_orientation_.normalize();
      }
    } else {
      // 使用随机偏移的默认目标 [0.3, 0.3, 0.6, 0.0, 0.9, 0.0, 0.1, 5.0]
      // 创建随机数生成器
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis(-0.05, 0.05);

      // 基础目标
      target_x_ = 0.3 + dis(gen);  // 随机偏移 -0.05 到 +0.05
      target_y_ = 0.3 + dis(gen);  // 随机偏移 -0.05 到 +0.05
      target_z_ = 0.6 + dis(gen);  // 随机偏移 -0.05 到 +0.05
      target_quat_w_ = 0.0;
      target_quat_x_ = 0.9;
      target_quat_y_ = 0.0;
      target_quat_z_ = 0.1;
      movement_duration_ = 5.0;

      target_orientation_ =
          Eigen::Quaterniond(target_quat_w_, target_quat_x_, target_quat_y_, target_quat_z_);
      if (target_orientation_.norm() > 0) {
        target_orientation_.normalize();
      }

      RCLCPP_INFO(get_node()->get_logger(),
                  "Using randomized default target: Pos[%.3f, %.3f, %.3f], "
                  "Ori[%.3f, %.3f, %.3f, %.3f], Duration: %.3f",
                  target_x_, target_y_, target_z_, target_orientation_.w(), target_orientation_.x(),
                  target_orientation_.y(), target_orientation_.z(), movement_duration_);
    }

    is_moving_ = true;
    move_complete_ = false;
    init_flag_ = false;

    RCLCPP_INFO(get_node()->get_logger(),
                "Starting movement to target: Pos[%.3f, %.3f, %.3f], "
                "Ori[%.3f, %.3f, %.3f, %.3f], Duration: %.3f",
                target_x_, target_y_, target_z_, target_orientation_.w(), target_orientation_.x(),
                target_orientation_.y(), target_orientation_.z(), movement_duration_);
  } else {
    current_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = current_time_ - start_time_;
  }

  if (is_moving_) {
    double progress = std::min(elapsed_time_ / movement_duration_, 1.0);
    double s = progress * progress * (3.0 - 2.0 * progress);

    Eigen::Vector3d current_pos = position_;
    current_pos(0) += (target_x_ - position_(0)) * s;
    current_pos(1) += (target_y_ - position_(1)) * s;
    current_pos(2) += (target_z_ - position_(2)) * s;

    Eigen::Quaterniond current_ori = orientation_.slerp(s, target_orientation_);

    if (progress >= 1.0) {
      is_moving_ = false;
      move_complete_ = true;
      final_pos_ = current_pos;
      final_orientation_ = current_ori;
      RCLCPP_INFO(get_node()->get_logger(), "Movement completed at time: %.3f", elapsed_time_);
    }

    if (franka_cartesian_pose_->setCommand(current_ori, current_pos)) {
      return controller_interface::return_type::OK;
    } else {
      RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }
  } else if (move_complete_) {
    if (checkForNewTarget()) {
      target_orientation_ =
          Eigen::Quaterniond(target_quat_w_, target_quat_x_, target_quat_y_, target_quat_z_);
      if (target_orientation_.norm() > 0) {
        target_orientation_.normalize();
      }

      position_ = final_pos_;
      orientation_ = final_orientation_;
      start_time_ = current_time_;
      elapsed_time_ = 0.0;
      is_moving_ = true;
      move_complete_ = false;

      RCLCPP_INFO(get_node()->get_logger(),
                  "New target detected. Starting movement to: Pos[%.3f, %.3f, %.3f], "
                  "Ori[%.3f, %.3f, %.3f, %.3f], Duration: %.3f",
                  target_x_, target_y_, target_z_, target_orientation_.w(), target_orientation_.x(),
                  target_orientation_.y(), target_orientation_.z(), movement_duration_);
    }

    if (franka_cartesian_pose_->setCommand(final_orientation_, final_pos_)) {
      return controller_interface::return_type::OK;
    } else {
      RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn CartesianPoseExampleController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  movement_duration_ = 5.0;
  target_quat_w_ = 0.0;
  target_quat_x_ = 0.9;
  target_quat_y_ = 0.0;
  target_quat_z_ = 0.1;
  target_x_ = 0.3;
  target_y_ = 0.3;
  target_z_ = 0.6;

  pending_target_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // 创建订阅器
  cartesian_move_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/NS_1/cartesian_move",
      rclcpp::QoS(10),  // 队列大小
      std::bind(&CartesianPoseExampleController::cartesianMoveCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(),
              "Subscribed to topic: /NS_1/cartesian_move for receiving target poses");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  init_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)