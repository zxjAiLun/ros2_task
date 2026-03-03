#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "multisensor_interfaces/msg/node_status.hpp"
#include "multisensor_interfaces/msg/system_alert.hpp"
#include "multisensor_interfaces/msg/vision.hpp"
#include "multisensor_interfaces/msg/audio.hpp"
#include "multisensor_interfaces/msg/lidar.hpp"
#include "multisensor_interfaces/msg/fusion_result.hpp"

#include "multisensor_interfaces/srv/get_status.hpp"
#include "multisensor_interfaces/srv/set_monitor_param.hpp"

class StatusMonitorNodeCpp : public rclcpp::Node
{
public:
  using NodeStatus = multisensor_interfaces::msg::NodeStatus;
  using SystemAlert = multisensor_interfaces::msg::SystemAlert;
  using Vision = multisensor_interfaces::msg::Vision;
  using Audio = multisensor_interfaces::msg::Audio;
  using Lidar = multisensor_interfaces::msg::Lidar;
  using FusionResult = multisensor_interfaces::msg::FusionResult;
  using GetStatus = multisensor_interfaces::srv::GetStatus;
  using SetMonitorParam = multisensor_interfaces::srv::SetMonitorParam;

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
      "/vision_cpp/data", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_vision, this, std::placeholders::_1));
    sub_audio_ = this->create_subscription<Audio>(
      "/audio_cpp/data", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_audio, this, std::placeholders::_1));
    sub_lidar_ = this->create_subscription<Lidar>(
      "/lidar_cpp/data", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_lidar, this, std::placeholders::_1));
    sub_fusion_ = this->create_subscription<FusionResult>(
      "/fusion/result_cpp", qos_sensor,
      std::bind(&StatusMonitorNodeCpp::on_fusion, this, std::placeholders::_1));

    pub_status_ = this->create_publisher<NodeStatus>("/system/status_cpp", qos_status);
    pub_alert_ = this->create_publisher<SystemAlert>("/system/alert_cpp", qos_status);

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(status_publish_period_)),
      std::bind(&StatusMonitorNodeCpp::on_timer, this));

    srv_get_status_ = this->create_service<GetStatus>(
      "/system/get_status_cpp",
      std::bind(&StatusMonitorNodeCpp::handle_get_status, this,
        std::placeholders::_1, std::placeholders::_2));

    srv_set_param_ = this->create_service<SetMonitorParam>(
      "/system/set_monitor_param_cpp",
      std::bind(&StatusMonitorNodeCpp::handle_set_param, this,
        std::placeholders::_1, std::placeholders::_2));

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

  void on_fusion(const FusionResult::SharedPtr)
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

      NodeStatus status_msg;
      status_msg.header.stamp = now;
      status_msg.header.frame_id = "";
      status_msg.node_name = node_name;
      status_msg.alive = alive;
      status_msg.last_update_age = static_cast<float>(
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

    SystemAlert alert;
    alert.header.stamp = now;
    alert.header.frame_id = "";
    alert.level = "WARN";
    alert.source_node = node_name;
    alert.message = "Node " + node_name + " no update for " + std::to_string(age) +
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

    NodeStatus status_msg;
    status_msg.header.stamp = now;
    status_msg.header.frame_id = "";
    status_msg.node_name = node_name;
    status_msg.alive = alive;
    status_msg.last_update_age = static_cast<float>(
      std::isinf(age) ? -1.0 : age);
    response->statuses.push_back(status_msg);
  }

  void handle_set_param(
    const std::shared_ptr<SetMonitorParam::Request> request,
    std::shared_ptr<SetMonitorParam::Response> response)
  {
    const std::string key = request->key;
    double value = request->value;

    if (key == "timeout_sec") {
      timeout_sec_ = value;
      response->success = true;
      response->message = "Updated timeout_sec";
    } else if (key == "status_publish_period") {
      status_publish_period_ = value;
      timer_->cancel();
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(status_publish_period_)),
        std::bind(&StatusMonitorNodeCpp::on_timer, this));
      response->success = true;
      response->message = "Updated status_publish_period";
    } else if (key == "alert_cooldown_sec") {
      alert_cooldown_sec_ = value;
      response->success = true;
      response->message = "Updated alert_cooldown_sec";
    } else {
      response->success = false;
      response->message = "Unknown parameter key: " + key;
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
  rclcpp::Subscription<FusionResult>::SharedPtr sub_fusion_;

  rclcpp::Publisher<NodeStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<SystemAlert>::SharedPtr pub_alert_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<GetStatus>::SharedPtr srv_get_status_;
  rclcpp::Service<SetMonitorParam>::SharedPtr srv_set_param_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StatusMonitorNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

