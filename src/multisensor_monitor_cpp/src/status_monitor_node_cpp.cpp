// C++ status_monitor_cpp using NodeStatusMsg/AlertMsg and new topics
#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "multisensor_interfaces/msg/node_status_msg.hpp"
#include "multisensor_interfaces/msg/alert_msg.hpp"
#include "multisensor_interfaces/msg/vision_msg.hpp"
#include "multisensor_interfaces/msg/audio_msg.hpp"
#include "multisensor_interfaces/msg/lidar_msg.hpp"
#include "multisensor_interfaces/msg/fusion_msg.hpp"

#include "multisensor_interfaces/srv/get_status.hpp"
#include "multisensor_interfaces/action/adjust_node_params.hpp"

class StatusMonitorNodeCpp : public rclcpp::Node
{
public:
  using NodeStatusMsg = multisensor_interfaces::msg::NodeStatusMsg;
  using AlertMsg = multisensor_interfaces::msg::AlertMsg;
  using Vision = multisensor_interfaces::msg::VisionMsg;
  using Audio = multisensor_interfaces::msg::AudioMsg;
  using Lidar = multisensor_interfaces::msg::LidarMsg;
  using FusionMsg = multisensor_interfaces::msg::FusionMsg;
  using GetStatus = multisensor_interfaces::srv::GetStatus;
  using AdjustNodeParams = multisensor_interfaces::action::AdjustNodeParams;
  using GoalHandleAdjustNodeParams = rclcpp_action::ServerGoalHandle<AdjustNodeParams>;

  StatusMonitorNodeCpp()
  : Node("status_monitor_cpp")
  {
    timeout_sec_ = this->declare_parameter<double>("timeout_sec", 3.0);
    status_publish_period_ = this->declare_parameter<double>("status_publish_period", 1.0);
    alert_cooldown_sec_ = this->declare_parameter<double>("alert_cooldown_sec", 5.0);

    rclcpp::QoS qos_status(rclcpp::KeepLast(10));
    qos_status.reliable();

    rclcpp::QoS qos_sensor(rclcpp::KeepLast(10));
    qos_sensor.best_effort();

    last_update_ = {
      {"vision_node_cpp", rclcpp::Time(0, 0, get_clock()->get_clock_type())},
      {"audio_node_cpp", rclcpp::Time(0, 0, get_clock()->get_clock_type())},
      {"lidar_node_cpp", rclcpp::Time(0, 0, get_clock()->get_clock_type())},
      {"fusion_node_cpp", rclcpp::Time(0, 0, get_clock()->get_clock_type())},
    };

    for (const auto & kv : last_update_) {
      last_alert_time_[kv.first] = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    sub_vision_ = this->create_subscription<Vision>(
      "/vision/detection", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_vision, this, std::placeholders::_1));
    sub_audio_ = this->create_subscription<Audio>(
      "/audio/source_angle", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_audio, this, std::placeholders::_1));
    sub_lidar_ = this->create_subscription<Lidar>(
      "/lidar/scan", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_lidar, this, std::placeholders::_1));
    sub_fusion_ = this->create_subscription<FusionMsg>(
      "/fusion/result", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_fusion, this, std::placeholders::_1));

    pub_status_ = this->create_publisher<NodeStatusMsg>("/system/status_cpp", qos_status);
    pub_alert_ = this->create_publisher<AlertMsg>("/system/alert_cpp", qos_status);

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(status_publish_period_)),
      std::bind(&StatusMonitorNodeCpp::on_timer, this));

    srv_get_status_ = this->create_service<GetStatus>(
      "/system/get_status_cpp",
      std::bind(&StatusMonitorNodeCpp::handle_get_status, this,
        std::placeholders::_1, std::placeholders::_2));

    adjust_action_server_ = rclcpp_action::create_server<AdjustNodeParams>(
      this,
      "/system/adjust_node_params",
      std::bind(&StatusMonitorNodeCpp::handle_adjust_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&StatusMonitorNodeCpp::handle_adjust_cancel, this, std::placeholders::_1),
      std::bind(&StatusMonitorNodeCpp::handle_adjust_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "status_monitor_cpp started: timeout=%.2fs status_period=%.2fs",
      timeout_sec_, status_publish_period_);
  }

private:
  void on_vision(const Vision::SharedPtr)
  {
    last_update_["vision_node_cpp"] = this->now();
  }

  void on_audio(const Audio::SharedPtr)
  {
    last_update_["audio_node_cpp"] = this->now();
  }

  void on_lidar(const Lidar::SharedPtr)
  {
    last_update_["lidar_node_cpp"] = this->now();
  }

  void on_fusion(const FusionMsg::SharedPtr)
  {
    last_update_["fusion_node_cpp"] = this->now();
  }

  void on_timer()
  {
    auto now = this->now();
    for (auto & kv : last_update_) {
      const std::string & node_name = kv.first;
      const rclcpp::Time & last_time = kv.second;

      double age = (now.nanoseconds() - last_time.nanoseconds()) / 1e9;
      bool has_ever_seen = last_time.nanoseconds() != 0;
      if (!has_ever_seen) {
        age = std::numeric_limits<double>::infinity();
      }

      bool alive = has_ever_seen && age <= timeout_sec_;

      NodeStatusMsg status_msg;
      status_msg.header.stamp = now;
      status_msg.header.frame_id = "";
      status_msg.node_name = node_name;
      status_msg.is_online = alive;
      status_msg.last_update_time = static_cast<float>(
        std::isinf(age) ? -1.0 : age);
      pub_status_->publish(status_msg);

      if (!alive) {
        maybe_publish_alert(node_name, now, age);
      }
    }
  }

  void maybe_publish_alert(const std::string & node_name, const rclcpp::Time & now, double age)
  {
    auto it = last_alert_time_.find(node_name);
    if (it != last_alert_time_.end()) {
      double dt = (now.nanoseconds() - it->second.nanoseconds()) / 1e9;
      if (dt < alert_cooldown_sec_) {
        return;
      }
    }

    AlertMsg alert;
    alert.header.stamp = now;
    alert.header.frame_id = "";
    alert.message = "WARN [" + node_name + "]: no update for " + std::to_string(age) +
      "s (> " + std::to_string(timeout_sec_) + "s)";
    pub_alert_->publish(alert);

    last_alert_time_[node_name] = now;
  }

  void handle_get_status(
    const std::shared_ptr<GetStatus::Request> request,
    std::shared_ptr<GetStatus::Response> response)
  {
    auto now = this->now();
    std::string target = request->node_name;
    if (!target.empty()) {
      auto it = last_update_.find(target);
      if (it != last_update_.end()) {
        append_status_for_node(target, it->second, now, response);
      }
    } else {
      for (auto & kv : last_update_) {
        append_status_for_node(kv.first, kv.second, now, response);
      }
    }
  }

  void append_status_for_node(
    const std::string & node_name,
    const rclcpp::Time & last_time,
    const rclcpp::Time & now,
    const std::shared_ptr<GetStatus::Response> & response)
  {
    double age = (now.nanoseconds() - last_time.nanoseconds()) / 1e9;
    bool has_ever_seen = last_time.nanoseconds() != 0;
    if (!has_ever_seen) {
      age = std::numeric_limits<double>::infinity();
    }
    bool alive = has_ever_seen && age <= timeout_sec_;

    NodeStatusMsg status_msg;
    status_msg.header.stamp = now;
    status_msg.header.frame_id = "";
    status_msg.node_name = node_name;
    status_msg.is_online = alive;
    status_msg.last_update_time = static_cast<float>(
      std::isinf(age) ? -1.0 : age);
    response->statuses.push_back(status_msg);
  }

  rclcpp_action::GoalResponse handle_adjust_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const AdjustNodeParams::Goal> goal)
  {
    std::string key = goal->key;
    if (key == "timeout_sec" || key == "status_publish_period" || key == "alert_cooldown_sec") {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    RCLCPP_WARN(this->get_logger(), "Reject AdjustNodeParams goal with unknown key: %s", key.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_adjust_cancel(
    const std::shared_ptr<GoalHandleAdjustNodeParams> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_adjust_accepted(const std::shared_ptr<GoalHandleAdjustNodeParams> goal_handle)
  {
    std::thread{std::bind(&StatusMonitorNodeCpp::execute_adjust, this, goal_handle)}.detach();
  }

  void execute_adjust(const std::shared_ptr<GoalHandleAdjustNodeParams> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    std::string key = goal->key;
    double value = goal->value;

    auto feedback = std::make_shared<AdjustNodeParams::Feedback>();
    feedback->progress = 0.0f;
    feedback->current_state = "validating";
    goal_handle->publish_feedback(feedback);

    bool success = true;
    std::string message;

    if (key == "timeout_sec") {
      timeout_sec_ = value;
      message = "Updated timeout_sec";
    } else if (key == "status_publish_period") {
      status_publish_period_ = value;
      timer_->cancel();
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(status_publish_period_)),
        std::bind(&StatusMonitorNodeCpp::on_timer, this));
      message = "Updated status_publish_period";
    } else if (key == "alert_cooldown_sec") {
      alert_cooldown_sec_ = value;
      message = "Updated alert_cooldown_sec";
    } else {
      success = false;
      message = "Unknown parameter key: " + key;
    }

    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<AdjustNodeParams::Result>();
      result->success = false;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      return;
    }

    feedback->progress = 1.0f;
    feedback->current_state = success ? "done" : "failed";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<AdjustNodeParams::Result>();
    result->success = success;
    result->message = message;

    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  double timeout_sec_;
  double status_publish_period_;
  double alert_cooldown_sec_;

  std::map<std::string, rclcpp::Time> last_update_;
  std::map<std::string, rclcpp::Time> last_alert_time_;

  rclcpp::Subscription<Vision>::SharedPtr sub_vision_;
  rclcpp::Subscription<Audio>::SharedPtr sub_audio_;
  rclcpp::Subscription<Lidar>::SharedPtr sub_lidar_;
  rclcpp::Subscription<FusionMsg>::SharedPtr sub_fusion_;

  rclcpp::Publisher<NodeStatusMsg>::SharedPtr pub_status_;
  rclcpp::Publisher<AlertMsg>::SharedPtr pub_alert_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<GetStatus>::SharedPtr srv_get_status_;
  rclcpp_action::Server<AdjustNodeParams>::SharedPtr adjust_action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StatusMonitorNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

