// C++ lidar_node_cpp using LidarMsg (single distance)
#include <chrono>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "multisensor_interfaces/msg/lidar_msg.hpp"

using namespace std::chrono_literals;

class LidarNodeCpp : public rclcpp::Node
{
public:
  LidarNodeCpp()
  : Node("lidar_node_cpp"),
    rng_(std::random_device{}())
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "lidar_link");
    double publish_rate = this->declare_parameter<double>("publish_rate", 5.0);

    obstacle_probability_ = this->declare_parameter<double>("obstacle_probability", 0.4);
    distance_min_m_ = this->declare_parameter<double>("distance_min_m", 0.3);
    distance_max_m_ = this->declare_parameter<double>("distance_max_m", 5.0);
    no_obstacle_distance_m_ = this->declare_parameter<double>("no_obstacle_distance_m", 10.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    publisher_ = this->create_publisher<multisensor_interfaces::msg::LidarMsg>("/lidar/scan", qos);

    std::chrono::duration<double> period{publish_rate > 0.0 ? 1.0 / publish_rate : 0.2};
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LidarNodeCpp::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "lidar_node_cpp started: rate=%.2fHz frame_id=%s",
      publish_rate, frame_id_.c_str());
  }

private:
  void on_timer()
  {
    multisensor_interfaces::msg::LidarMsg msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.type = "2d_circle";

    std::uniform_real_distribution<double> uni(0.0, 1.0);
    bool obstacle = uni(rng_) < obstacle_probability_;

    double radius;
    if (obstacle) {
      std::uniform_real_distribution<double> dist(distance_min_m_, distance_max_m_);
      radius = dist(rng_);
    } else {
      radius = no_obstacle_distance_m_;
    }

    const int points_per_scan = 360;
    msg.point_cloud.clear();
    msg.point_cloud.reserve(static_cast<size_t>(points_per_scan * 3));

    for (int i = 0; i < points_per_scan; ++i) {
      double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(points_per_scan);
      float x = static_cast<float>(radius * std::cos(angle));
      float y = static_cast<float>(radius * std::sin(angle));
      float z = 0.0f;
      msg.point_cloud.push_back(x);
      msg.point_cloud.push_back(y);
      msg.point_cloud.push_back(z);
    }
    msg.point_count = static_cast<int32_t>(points_per_scan);

    publisher_->publish(msg);
  }

  std::string frame_id_;
  double obstacle_probability_;
  double distance_min_m_;
  double distance_max_m_;
  double no_obstacle_distance_m_;

  rclcpp::Publisher<multisensor_interfaces::msg::LidarMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarNodeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

