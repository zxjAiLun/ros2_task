// C++ audio_node_cpp using AudioMsg (only angle)
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "multisensor_interfaces/msg/audio_msg.hpp"

using namespace std::chrono_literals;

class AudioNodeCpp : public rclcpp::Node
{
public:
  AudioNodeCpp()
  : Node("audio_node_cpp"),
    rng_(std::random_device{}())
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "mic_link");
    double publish_rate = this->declare_parameter<double>("publish_rate", 10.0);

    voice_probability_ = this->declare_parameter<double>("voice_probability", 0.5);
    angle_mean_deg_ = this->declare_parameter<double>("angle_mean_deg", 0.0);
    angle_sigma_deg_ = this->declare_parameter<double>("angle_sigma_deg", 45.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    publisher_ = this->create_publisher<multisensor_interfaces::msg::AudioMsg>("/audio/source_angle", qos);

    std::chrono::duration<double> period{publish_rate > 0.0 ? 1.0 / publish_rate : 0.1};
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&AudioNodeCpp::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "audio_node_cpp started: rate=%.2fHz frame_id=%s",
      publish_rate, frame_id_.c_str());
  }

private:
  void on_timer()
  {
    multisensor_interfaces::msg::AudioMsg msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    std::uniform_real_distribution<double> uni(0.0, 1.0);
    bool voice_detected = uni(rng_) < voice_probability_;

    if (voice_detected) {
      std::normal_distribution<double> angle_dist(angle_mean_deg_, angle_sigma_deg_);
      double angle = normalize_angle_deg(angle_dist(rng_));
      msg.audio_source_angle = static_cast<float>(angle);
    } else {
      msg.audio_source_angle = std::numeric_limits<float>::quiet_NaN();
    }

    publisher_->publish(msg);
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

  std::string frame_id_;
  double voice_probability_;
  double angle_mean_deg_;
  double angle_sigma_deg_;

  rclcpp::Publisher<multisensor_interfaces::msg::AudioMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AudioNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

