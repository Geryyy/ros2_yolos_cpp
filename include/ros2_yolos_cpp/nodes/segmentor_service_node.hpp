#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "ros2_yolos_cpp/adapters/segmentor_adapter.hpp"
#include "ros2_yolos_cpp/srv/segment_image.hpp"

namespace ros2_yolos_cpp
{

class YolosSegmentorServiceNode : public rclcpp::Node
{
public:
  explicit YolosSegmentorServiceNode(
    const rclcpp::NodeOptions & options);

private:
  void declareParameters();
  YolosConfig loadConfig();

  void handleRequest(
    const std::shared_ptr<srv::SegmentImage::Request> request,
    std::shared_ptr<srv::SegmentImage::Response> response);

  std::shared_ptr<SegmentorAdapter> segmentor_;
  rclcpp::Service<srv::SegmentImage>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_segmentations_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_total_ms_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_convert_ms_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_infer_ms_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pack_ms_pub_;
};

}  // namespace ros2_yolos_cpp
