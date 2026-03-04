// C++ fusion_node_cpp using VisionMsg/AudioMsg/LidarMsg/FusionMsg
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "multisensor_interfaces/msg/vision_msg.hpp"
#include "multisensor_interfaces/msg/audio_msg.hpp"
#include "multisensor_interfaces/msg/lidar_msg.hpp"
#include "multisensor_interfaces/msg/fusion_msg.hpp"

class FusionNodeCpp : public rclcpp::Node
{
public:
  using Vision = multisensor_interfaces::msg::VisionMsg;
  using Audio = multisensor_interfaces::msg::AudioMsg;
  using Lidar = multisensor_interfaces::msg::LidarMsg;
  using FusionMsg = multisensor_interfaces::msg::FusionMsg;

  using Policy = message_filters::sync_policies::ApproximateTime<Vision, Audio, Lidar>;

  FusionNodeCpp()
  : Node("fusion_node_cpp")
  {
    int queue_size = this->declare_parameter<int>("sync_queue_size", 10);
    double slop = this->declare_parameter<double>("sync_slop", 0.1);
    (void)slop;

    auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    sub_vision_.subscribe(this, "/vision/detection", qos_sensor.get_rmw_qos_profile());
    sub_audio_.subscribe(this, "/audio/source_angle", qos_sensor.get_rmw_qos_profile());
    sub_lidar_.subscribe(this, "/lidar/scan", qos_sensor.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
      Policy(queue_size), sub_vision_, sub_audio_, sub_lidar_);
    sync_->registerCallback(std::bind(&FusionNodeCpp::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    auto qos_fusion = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pub_fusion_ = this->create_publisher<FusionMsg>("/fusion/result", qos_fusion);

    RCLCPP_INFO(
      this->get_logger(),
      "fusion_node_cpp started: queue_size=%d",
      queue_size);
  }

private:
  void sync_callback(
    const Vision::ConstSharedPtr vision,
    const Audio::ConstSharedPtr audio,
    const Lidar::ConstSharedPtr lidar)
  {
    (void)lidar;
    FusionMsg result;
    result.header.stamp = vision->header.stamp;
    result.header.frame_id = "fusion_cpp_base";

    std::string status;
    double vision_conf = 0.0;
    double audio_angle = std::numeric_limits<double>::quiet_NaN();
    double lidar_distance = 0.0;

    compute_fusion(*vision, *audio, *lidar, status, vision_conf, audio_angle, lidar_distance);

    result.vision_person_detected = vision->person_detected;
    result.vision_confidence = static_cast<float>(vision_conf);
    result.audio_source_angle = static_cast<float>(audio_angle);
    result.lidar_distance = static_cast<float>(lidar_distance);
    result.fusion_status = status;

    pub_fusion_->publish(result);
  }

  void compute_fusion(
    const Vision & vision,
    const Audio & audio,
    const Lidar & lidar,
    std::string & status,
    double & vision_conf,
    double & audio_angle,
    double & lidar_distance) const
  {
    bool v = vision.person_detected;
    audio_angle = static_cast<double>(audio.audio_source_angle);
    lidar_distance = static_cast<double>(lidar.lidar_distance);
    vision_conf = static_cast<double>(vision.vision_confidence);

    if (!v && std::isnan(audio_angle)) {
      status = "none";
      vision_conf = 0.0;
      return;
    }
    if (v && std::isnan(audio_angle)) {
      status = "vision_only";
      return;
    }
    if (!v && !std::isnan(audio_angle)) {
      status = "audio_only";
      vision_conf = 0.0;
      return;
    }

    status = "both";
  }

  message_filters::Subscriber<Vision> sub_vision_;
  message_filters::Subscriber<Audio> sub_audio_;
  message_filters::Subscriber<Lidar> sub_lidar_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;

  rclcpp::Publisher<FusionMsg>::SharedPtr pub_fusion_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

