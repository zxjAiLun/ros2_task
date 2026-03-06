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
    FusionMsg result;
    result.header.stamp = vision->header.stamp;
    result.header.frame_id = "fusion_cpp_base";
    result.frame_id = "fusion_cpp_base";

    // 直接打包原始多模态数据，供下游多模态模型使用
    result.vision = *vision;
    result.audio = *audio;
    result.lidar = *lidar;
    result.is_aligned = true;

    pub_fusion_->publish(result);
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

