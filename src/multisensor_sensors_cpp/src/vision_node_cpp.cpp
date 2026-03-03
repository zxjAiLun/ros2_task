#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "multisensor_interfaces/msg/vision.hpp"

using namespace std::chrono_literals;

class VisionNodeCpp : public rclcpp::Node
{
public:
  VisionNodeCpp()
  : Node("vision_node_cpp"),
    rng_(std::random_device{}())
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_link");
    double publish_rate = this->declare_parameter<double>("publish_rate", 10.0);

    detect_probability_ = this->declare_parameter<double>("detect_probability", 0.6);
    angle_mean_deg_ = this->declare_parameter<double>("angle_mean_deg", 0.0);
    angle_sigma_deg_ = this->declare_parameter<double>("angle_sigma_deg", 30.0);
    confidence_min_ = this->declare_parameter<double>("confidence_min", 0.5);
    confidence_max_ = this->declare_parameter<double>("confidence_max", 1.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    publisher_ = this->create_publisher<multisensor_interfaces::msg::Vision>("/vision_cpp/data", qos);

    std::chrono::duration<double> period{publish_rate > 0.0 ? 1.0 / publish_rate : 0.1};
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&VisionNodeCpp::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "vision_node_cpp started: rate=%.2fHz frame_id=%s",
      publish_rate, frame_id_.c_str());
  }

private:
  void on_timer()
  {
    multisensor_interfaces::msg::Vision msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    std::uniform_real_distribution<double> uni(0.0, 1.0);
    bool detected = uni(rng_) < detect_probability_;
    msg.detected = detected;

    if (detected) {
      std::normal_distribution<double> angle_dist(angle_mean_deg_, angle_sigma_deg_);
      double angle = normalize_angle_deg(angle_dist(rng_));
      std::uniform_real_distribution<double> conf_dist(confidence_min_, confidence_max_);
      msg.angle = static_cast<float>(angle);
      msg.confidence = static_cast<float>(conf_dist(rng_));
    } else {
      msg.angle = std::numeric_limits<float>::quiet_NaN();
      msg.confidence = 0.0f;
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
  double detect_probability_;
  double angle_mean_deg_;
  double angle_sigma_deg_;
  double confidence_min_;
  double confidence_max_;

  rclcpp::Publisher<multisensor_interfaces::msg::Vision>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

