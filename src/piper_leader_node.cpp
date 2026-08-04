// Copyright 2026 Physical AI Runtime contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <piper/control_types.h>
#include <piper/mit_config.h>
#include <piper/model.h>
#include <piper/robot.h>
#include <piper/teaching_pendant.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace piper_leader_teleop {

using namespace std::chrono_literals;

namespace {
// PiperGripperInterface joint position is one finger travel; libpiper pendant
// width is full opening. Match piper_hardware_interface conversion.
constexpr double kFingerTravelPerOpeningWidth = 0.5;
constexpr double kMaxOpeningWidthM = 0.08;
}  // namespace

class PiperLeaderNode final : public rclcpp::Node {
public:
  PiperLeaderNode() : Node("piper_leader") {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    init_can_ = declare_parameter<bool>("init_can", true);
    leader_robot_description_ =
        declare_parameter<std::string>("leader_robot_description", "");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
    feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s", 0.1);
    mit_kd_effort_damping_ =
        declare_parameter<double>("mit_kd_effort_damping", 0.0);
    restore_pendant_servo_on_stop_ =
        declare_parameter<bool>("restore_pendant_servo_on_stop", false);
    joint_topic_ = declare_parameter<std::string>(
        "joint_reference_topic",
        "/action_sources/piper_leader/arm/joint_reference");
    pendant_topic_ = declare_parameter<std::string>(
        "pendant_state_topic", "/teleop/piper_leader/pendant_state");
    gripper_topic_ = declare_parameter<std::string>(
        "gripper_reference_topic",
        "/action_sources/piper_leader/end_effector/joint_reference");
    status_topic_ = declare_parameter<std::string>(
        "status_topic", "/teleop/piper_leader/status");
    gripper_joint_name_ =
        declare_parameter<std::string>("gripper_joint_name", "gripper_joint1");
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});

    if (joint_names_.size() != piper::kNumJoints || publish_rate_hz_ <= 0.0 ||
        !std::isfinite(publish_rate_hz_) || feedback_timeout_s_ <= 0.0 ||
        !std::isfinite(feedback_timeout_s_) || mit_kd_effort_damping_ < 0.0 ||
        !std::isfinite(mit_kd_effort_damping_)) {
      throw std::invalid_argument("invalid Piper leader parameters");
    }

    const auto command_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        joint_topic_, command_qos);
    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        gripper_topic_, command_qos);
    pendant_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        pendant_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "~/enable", std::bind(&PiperLeaderNode::onEnable, this,
                              std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PiperLeaderNode::publishLatest, this));

    RCLCPP_INFO(get_logger(),
                "Piper leader ready but disabled. Call '%s/enable' with "
                "data=true after hardware checks.",
                get_fully_qualified_name());
  }

  ~PiperLeaderNode() override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stopLocked();
  }

private:
  void onEnable(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    try {
      if (request->data) {
        startLocked();
        response->message = "Piper leader gravity compensation active";
      } else {
        stopLocked();
        response->message = "Piper leader stopped";
      }
      response->success = true;
    } catch (const std::exception &error) {
      stopLocked();
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Piper leader transition failed: %s",
                   error.what());
    }
  }

  void startLocked() {
    if (active_.load()) {
      return;
    }

    if (leader_robot_description_.empty() ||
        leader_robot_description_.find("<robot") == std::string::npos) {
      throw std::runtime_error(
          "leader_robot_description is empty or is not an expanded URDF");
    }

    const auto unique_suffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    temporary_model_path_ = std::filesystem::temp_directory_path() /
                            ("piper_leader_model_" + unique_suffix + ".urdf");
    {
      std::ofstream model_file(temporary_model_path_,
                               std::ios::out | std::ios::trunc);
      if (!model_file) {
        throw std::runtime_error("failed to create temporary leader URDF");
      }
      model_file << leader_robot_description_;
      if (!model_file) {
        throw std::runtime_error("failed to write temporary leader URDF");
      }
    }

    robot_ = std::make_unique<piper::Robot>(
        can_interface_, piper::RealtimeConfig::kIgnore, init_can_);
    model_ = std::make_unique<piper::Model>(
        robot_->loadModel(temporary_model_path_.string()));
    if (!model_->isLoaded()) {
      throw std::runtime_error("Piper leader gravity model failed to load");
    }

    piper::MitGains gains{};
    gains.kd_effort_damping = mit_kd_effort_damping_;
    robot_->setMitGains(gains);

    const auto initial_state = robot_->readOnce();
    if (!initial_state.valid || !initial_state.coherent ||
        !isFresh(initial_state.feedback_receive_time)) {
      throw std::runtime_error(
          "Piper leader has no fresh coherent joint feedback");
    }
    robot_->enableArm();

    pendant_ = std::make_unique<piper::TeachingPendant>(can_interface_, false);
    pendant_->enableFingerFreeMove();

    last_arm_sequence_ = 0;
    last_pendant_sequence_ = 0;
    active_.store(true);
    robot_->controlAsync([this](const piper::RobotState &state,
                                piper::Duration) -> piper::Torques {
      piper::Torques command{model_->gravity(state)};
      if (!active_.load(std::memory_order_relaxed)) {
        return piper::MotionFinished(command);
      }
      return command;
    });

    RCLCPP_INFO(get_logger(),
                "Piper leader enabled on %s using leader_robot_description",
                can_interface_.c_str());
  }

  void stopLocked() noexcept {
    active_.store(false);
    if (robot_) {
      try {
        robot_->stop();
      } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "Piper leader stop failed: %s", error.what());
      }
    }
    if (pendant_ && restore_pendant_servo_on_stop_) {
      try {
        pendant_->disableFingerFreeMove();
      } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "Pendant servo restore failed: %s",
                    error.what());
      }
    }
    pendant_.reset();
    model_.reset();
    robot_.reset();
    if (!temporary_model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(temporary_model_path_, error);
      temporary_model_path_.clear();
    }
  }

  template <typename TimePoint> bool isFresh(TimePoint receive_time) const {
    if (receive_time == TimePoint{}) {
      return false;
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         receive_time)
               .count() <= feedback_timeout_s_;
  }

  void publishLatest() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!active_.load() || !robot_ || !pendant_) {
      publishStatus(nullptr, nullptr);
      return;
    }

    const auto arm = robot_->readLatest();
    const auto pendant = pendant_->readLatest();
    const bool arm_usable =
        arm.valid && arm.coherent && isFresh(arm.feedback_receive_time) &&
        arm.device_status != piper::FeedbackDeviceStatus::kError;
    const bool pendant_usable =
        pendant.valid && isFresh(pendant.feedback_receive_time) &&
        pendant.device_status != piper::FeedbackDeviceStatus::kError;
    const auto stamp = now();

    if (arm_usable && arm.feedback_sequence != last_arm_sequence_) {
      trajectory_msgs::msg::JointTrajectory message;
      message.header.stamp = stamp;
      message.joint_names = joint_names_;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.assign(arm.q.begin(), arm.q.end());
      point.velocities.assign(arm.dq.begin(), arm.dq.end());
      point.effort.assign(arm.tau_J.begin(), arm.tau_J.end());
      message.points.push_back(std::move(point));
      joint_pub_->publish(message);
      last_arm_sequence_ = arm.feedback_sequence;
    }

    if (pendant_usable && pendant.feedback_sequence != last_pendant_sequence_) {
      const double opening_width =
          std::clamp(pendant.width, 0.0, kMaxOpeningWidthM);
      const double finger_position =
          opening_width * kFingerTravelPerOpeningWidth;

      sensor_msgs::msg::JointState raw_message;
      raw_message.header.stamp = stamp;
      raw_message.name = {gripper_joint_name_};
      // Raw pendant topic keeps SDK full opening width.
      raw_message.position = {pendant.width};
      raw_message.effort = {pendant.force};
      pendant_pub_->publish(raw_message);

      // EM end_effector contract: joint position = one finger travel.
      trajectory_msgs::msg::JointTrajectory gripper_message;
      gripper_message.header.stamp = stamp;
      gripper_message.joint_names = {gripper_joint_name_};
      trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
      gripper_point.positions = {finger_position};
      gripper_message.points.push_back(std::move(gripper_point));
      gripper_pub_->publish(gripper_message);

      last_pendant_sequence_ = pendant.feedback_sequence;
    }

    publishStatus(&arm, &pendant);
  }

  void publishStatus(const piper::RobotState *arm,
                     const piper::TeachingPendantState *pendant) {
    std::ostringstream json;
    json << "{\"schema_version\":1,\"active\":"
         << (active_.load() ? "true" : "false");
    if (arm) {
      const double age =
          arm->feedback_receive_time == std::chrono::steady_clock::time_point{}
              ? -1.0
              : std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              arm->feedback_receive_time)
                    .count();
      json << ",\"arm_sequence\":" << arm->feedback_sequence
           << ",\"arm_valid\":" << (arm->valid ? "true" : "false")
           << ",\"arm_coherent\":" << (arm->coherent ? "true" : "false")
           << ",\"arm_source_age_s\":" << age;
    }
    if (pendant) {
      const double age =
          pendant->feedback_receive_time ==
                  std::chrono::steady_clock::time_point{}
              ? -1.0
              : std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              pendant->feedback_receive_time)
                    .count();
      json << ",\"pendant_sequence\":" << pendant->feedback_sequence
           << ",\"pendant_valid\":" << (pendant->valid ? "true" : "false")
           << ",\"pendant_source_age_s\":" << age;
    }
    json << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    status_pub_->publish(message);
  }

  std::mutex state_mutex_;
  std::atomic<bool> active_{false};
  std::unique_ptr<piper::Robot> robot_;
  std::unique_ptr<piper::Model> model_;
  std::unique_ptr<piper::TeachingPendant> pendant_;

  std::string can_interface_;
  bool init_can_{true};
  std::string leader_robot_description_;
  std::filesystem::path temporary_model_path_;
  double publish_rate_hz_{200.0};
  double feedback_timeout_s_{0.1};
  double mit_kd_effort_damping_{0.0};
  bool restore_pendant_servo_on_stop_{false};
  std::string joint_topic_;
  std::string pendant_topic_;
  std::string gripper_topic_;
  std::string status_topic_;
  std::string gripper_joint_name_;
  std::vector<std::string> joint_names_;
  uint64_t last_arm_sequence_{0};
  uint64_t last_pendant_sequence_{0};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pendant_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace piper_leader_teleop

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<piper_leader_teleop::PiperLeaderNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("piper_leader"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
