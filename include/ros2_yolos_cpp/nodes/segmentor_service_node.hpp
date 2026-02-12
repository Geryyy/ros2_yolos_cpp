#pragma once

#include <rclcpp/rclcpp.hpp>
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
};

}  // namespace ros2_yolos_cpp
