#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "multisensor_interfaces/msg/vision.hpp"
#include "multisensor_interfaces/msg/audio.hpp"
#include "multisensor_interfaces/msg/lidar.hpp"
#include "multisensor_interfaces/msg/fusion_result.hpp"

class FusionNodeCpp : public rclcpp::Node
{
public:
  using Vision = multisensor_interfaces::msg::Vision;
  using Audio = multisensor_interfaces::msg::Audio;
  using Lidar = multisensor_interfaces::msg::Lidar;
  using FusionResult = multisensor_interfaces::msg::FusionResult;

  using Policy = message_filters::sync_policies::ApproximateTime<Vision, Audio, Lidar>;

  FusionNodeCpp()
  : Node("fusion_node_cpp")
  {
    angle_tolerance_deg_ = this->declare_parameter<double>("angle_tolerance_deg", 30.0);
    vision_weight_ = this->declare_parameter<double>("vision_weight", 0.6);
    audio_weight_ = this->declare_parameter<double>("audio_weight", 0.4);

    int queue_size = this->declare_parameter<int>("sync_queue_size", 10);
    double slop = this->declare_parameter<double>("sync_slop", 0.1);
    (void)slop;  // ApproximateTime policy slop is template parameter; keep simple here

    auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    sub_vision_.subscribe(this, "/vision_cpp/data", qos_sensor.get_rmw_qos_profile());
    sub_audio_.subscribe(this, "/audio_cpp/data", qos_sensor.get_rmw_qos_profile());
    sub_lidar_.subscribe(this, "/lidar_cpp/data", qos_sensor.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
      Policy(queue_size), sub_vision_, sub_audio_, sub_lidar_);
    sync_->registerCallback(std::bind(&FusionNodeCpp::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    auto qos_fusion = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pub_fusion_ = this->create_publisher<FusionResult>("/fusion/result_cpp", qos_fusion);

    RCLCPP_INFO(
      this->get_logger(),
      "fusion_node_cpp started: angle_tolerance=%.2fdeg queue_size=%d",
      angle_tolerance_deg_, queue_size);
  }

private:
  void sync_callback(
    const Vision::ConstSharedPtr vision,
    const Audio::ConstSharedPtr audio,
    const Lidar::ConstSharedPtr lidar)
  {
    (void)lidar;
    FusionResult result;
    result.header.stamp = vision->header.stamp;
    result.header.frame_id = "fusion_cpp_base";

    std::string status;
    double confidence = 0.0;
    double target_angle = std::numeric_limits<double>::quiet_NaN();

    compute_fusion(*vision, *audio, status, confidence, target_angle);

    result.fusion_status = status;
    result.fusion_confidence = static_cast<float>(confidence);
    result.target_angle = static_cast<float>(target_angle);

    pub_fusion_->publish(result);
  }

  void compute_fusion(
    const Vision & vision,
    const Audio & audio,
    std::string & status,
    double & confidence,
    double & target_angle) const
  {
    bool v = vision.detected;
    bool a = audio.voice_detected;

    if (!v && !a) {
      status = "none";
      confidence = 0.0;
      target_angle = std::numeric_limits<double>::quiet_NaN();
      return;
    }
    if (v && !a) {
      status = "vision_only";
      confidence = std::max(static_cast<double>(vision.confidence), 0.0);
      target_angle = vision.angle;
      return;
    }
    if (a && !v) {
      status = "audio_only";
      confidence = std::max(static_cast<double>(audio.energy), 0.0);
      target_angle = audio.direction;
      return;
    }

    if (std::isnan(vision.angle) || std::isnan(audio.direction)) {
      status = "conflict";
      confidence = 0.0;
      target_angle = std::numeric_limits<double>::quiet_NaN();
      return;
    }

    double diff = std::fabs(normalize_angle_deg(
      static_cast<double>(vision.angle) - static_cast<double>(audio.direction)));
    if (diff <= angle_tolerance_deg_) {
      double v_conf = std::max(static_cast<double>(vision.confidence), 0.0);
      double a_conf = std::max(static_cast<double>(audio.energy), 0.0);
      double norm = v_conf + a_conf;
      double v_w;
      double a_w;
      if (norm > 0.0) {
        v_w = vision_weight_ * v_conf / norm;
        a_w = audio_weight_ * a_conf / norm;
      } else {
        v_w = vision_weight_;
        a_w = audio_weight_;
      }
      double fused_angle = v_w * static_cast<double>(vision.angle) +
        a_w * static_cast<double>(audio.direction);
      double fused_conf = std::min(
        1.0, v_conf * vision_weight_ + a_conf * audio_weight_);

      status = "both";
      confidence = fused_conf;
      target_angle = fused_angle;
      return;
    }

    status = "conflict";
    confidence = 0.0;
    target_angle = std::numeric_limits<double>::quiet_NaN();
  }

  static double normalize_angle_deg(double angle)
  {
    double value = std::fmod(angle, 360.0);
    if (value <= -180.0) {
      value += 360.0;
    } else if (value > 180.0) {
      value -= 360.0;
    }
    return value;
  }

  double angle_tolerance_deg_;
  double vision_weight_;
  double audio_weight_;

  message_filters::Subscriber<Vision> sub_vision_;
  message_filters::Subscriber<Audio> sub_audio_;
  message_filters::Subscriber<Lidar> sub_lidar_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;

  rclcpp::Publisher<FusionResult>::SharedPtr pub_fusion_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

