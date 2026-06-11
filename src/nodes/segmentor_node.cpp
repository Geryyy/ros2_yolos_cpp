// Copyright 2024 YOLOs-CPP Team
// SPDX-License-Identifier: AGPL-3.0

#include "ros2_yolos_cpp/nodes/segmentor_node.hpp"
#include "ros2_yolos_cpp/conversion/segmentation_converter.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <algorithm>
#include <chrono>

namespace ros2_yolos_cpp {

YolosSegmentorNode::YolosSegmentorNode(const rclcpp::NodeOptions& o) : rclcpp_lifecycle::LifecycleNode("yolos_segmentor", o) { declareParameters(); }

void YolosSegmentorNode::declareParameters() {
  declare_parameter("model_path", rclcpp::PARAMETER_STRING);
  declare_parameter("labels_path", rclcpp::PARAMETER_STRING);
  declare_parameter("use_gpu", false);
  declare_parameter("conf_threshold", 0.4);
  declare_parameter("nms_threshold", 0.45);
  declare_parameter("publish_debug_image", false);
  declare_parameter("publish_timing", true);
  declare_parameter("timing_log_every_n_frames", 30);
}

YolosConfig YolosSegmentorNode::loadConfig() {
  YolosConfig c;
  c.model_path = get_parameter("model_path").as_string();
  c.labels_path = get_parameter("labels_path").as_string();
  c.use_gpu = get_parameter("use_gpu").as_bool();
  c.conf_threshold = static_cast<float>(get_parameter("conf_threshold").as_double());
  c.nms_threshold = static_cast<float>(get_parameter("nms_threshold").as_double());
  conf_ = c.conf_threshold; nms_ = c.nms_threshold;
  debug_ = get_parameter("publish_debug_image").as_bool();
  publish_timing_ = get_parameter("publish_timing").as_bool();
  timing_log_every_n_frames_ =
    std::max(1, static_cast<int>(get_parameter("timing_log_every_n_frames").as_int()));
  return c;
}

YolosSegmentorNode::CallbackReturn YolosSegmentorNode::on_configure(const rclcpp_lifecycle::State&) {
  auto c = loadConfig();
  if (c.model_path.empty() || c.labels_path.empty()) return CallbackReturn::FAILURE;
  segmentor_ = createSegmentorAdapter();
  if (!segmentor_->initialize(c)) return CallbackReturn::FAILURE;
  cb_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("~/detections", 10);
  mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/mask", 1);
  if (debug_) debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);
  if (publish_timing_) {
    timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/timing", 10);
    timing_frame_count_pub_ = create_publisher<std_msgs::msg::Float64>("~/timing/frame_count", 10);
    timing_segmentations_pub_ = create_publisher<std_msgs::msg::Float64>("~/timing/segmentations", 10);
    timing_total_ms_pub_ = create_publisher<std_msgs::msg::Float64>("~/timing/total_ms", 10);
    timing_convert_ms_pub_ = create_publisher<std_msgs::msg::Float64>("~/timing/convert_ms", 10);
    timing_infer_ms_pub_ = create_publisher<std_msgs::msg::Float64>("~/timing/infer_ms", 10);
    timing_publish_ms_pub_ = create_publisher<std_msgs::msg::Float64>("~/timing/publish_ms", 10);
    timing_debug_ms_pub_ = create_publisher<std_msgs::msg::Float64>("~/timing/debug_ms", 10);
  }
  return CallbackReturn::SUCCESS;
}

YolosSegmentorNode::CallbackReturn YolosSegmentorNode::on_activate(const rclcpp_lifecycle::State&) {
  det_pub_->on_activate(); mask_pub_->on_activate();
  if (debug_pub_) debug_pub_->on_activate();
  if (timing_pub_) timing_pub_->on_activate();
  if (timing_frame_count_pub_) timing_frame_count_pub_->on_activate();
  if (timing_segmentations_pub_) timing_segmentations_pub_->on_activate();
  if (timing_total_ms_pub_) timing_total_ms_pub_->on_activate();
  if (timing_convert_ms_pub_) timing_convert_ms_pub_->on_activate();
  if (timing_infer_ms_pub_) timing_infer_ms_pub_->on_activate();
  if (timing_publish_ms_pub_) timing_publish_ms_pub_->on_activate();
  if (timing_debug_ms_pub_) timing_debug_ms_pub_->on_activate();
  auto opt = rclcpp::SubscriptionOptions(); opt.callback_group = cb_;
  sub_ = create_subscription<sensor_msgs::msg::Image>("~/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&YolosSegmentorNode::imageCallback, this, std::placeholders::_1), opt);
  return CallbackReturn::SUCCESS;
}

YolosSegmentorNode::CallbackReturn YolosSegmentorNode::on_deactivate(const rclcpp_lifecycle::State&) {
  sub_.reset(); det_pub_->on_deactivate(); mask_pub_->on_deactivate();
  if (debug_pub_) debug_pub_->on_deactivate();
  if (timing_pub_) timing_pub_->on_deactivate();
  if (timing_frame_count_pub_) timing_frame_count_pub_->on_deactivate();
  if (timing_segmentations_pub_) timing_segmentations_pub_->on_deactivate();
  if (timing_total_ms_pub_) timing_total_ms_pub_->on_deactivate();
  if (timing_convert_ms_pub_) timing_convert_ms_pub_->on_deactivate();
  if (timing_infer_ms_pub_) timing_infer_ms_pub_->on_deactivate();
  if (timing_publish_ms_pub_) timing_publish_ms_pub_->on_deactivate();
  if (timing_debug_ms_pub_) timing_debug_ms_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

YolosSegmentorNode::CallbackReturn YolosSegmentorNode::on_cleanup(const rclcpp_lifecycle::State&) {
  if (segmentor_) { segmentor_->shutdown(); segmentor_.reset(); }
  det_pub_.reset(); mask_pub_.reset(); debug_pub_.reset(); timing_pub_.reset();
  timing_frame_count_pub_.reset();
  timing_segmentations_pub_.reset();
  timing_total_ms_pub_.reset();
  timing_convert_ms_pub_.reset();
  timing_infer_ms_pub_.reset();
  timing_publish_ms_pub_.reset();
  timing_debug_ms_pub_.reset();
  return CallbackReturn::SUCCESS;
}

void YolosSegmentorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (!segmentor_ || !segmentor_->isInitialized()) return;
  const auto t_start = std::chrono::steady_clock::now();
  try {
    auto cv = cv_bridge::toCvShare(msg, "bgr8");
    const auto t_after_convert = std::chrono::steady_clock::now();
    auto segs = segmentor_->segment(cv->image, conf_, nms_);
    const auto t_after_infer = std::chrono::steady_clock::now();
    det_pub_->publish(conversion::toDetection2DArray(segs, msg->header, msg->width, msg->height));
    mask_pub_->publish(conversion::toCombinedMaskImage(segs, msg->header, msg->width, msg->height));
    const auto t_after_publish = std::chrono::steady_clock::now();
    if (debug_ && debug_pub_ && debug_pub_->is_activated()) {
      cv::Mat d = cv->image.clone(); segmentor_->drawSegmentations(d, segs);
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", d).toImageMsg());
    }
    const auto t_end = std::chrono::steady_clock::now();
    ++frame_count_;
    const auto convert_ms =
      std::chrono::duration<double, std::milli>(t_after_convert - t_start).count();
    const auto infer_ms =
      std::chrono::duration<double, std::milli>(t_after_infer - t_after_convert).count();
    const auto publish_ms =
      std::chrono::duration<double, std::milli>(t_after_publish - t_after_infer).count();
    const auto debug_ms =
      std::chrono::duration<double, std::milli>(t_end - t_after_publish).count();
    const auto total_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
    if (publish_timing_ &&
      (frame_count_ % static_cast<uint64_t>(timing_log_every_n_frames_) == 0U))
    {
      RCLCPP_INFO(
        get_logger(),
        "YOLO topic frame=%lu segmentations=%zu total=%.1f ms "
        "(convert=%.1f infer=%.1f publish=%.1f debug=%.1f)",
        frame_count_, segs.size(), total_ms, convert_ms, infer_ms, publish_ms, debug_ms);
    }
    if (publish_timing_ && timing_pub_ && timing_pub_->is_activated()) {
      std_msgs::msg::Float64MultiArray timing;
      timing.data = {
        static_cast<double>(frame_count_),
        static_cast<double>(segs.size()),
        total_ms,
        convert_ms,
        infer_ms,
        publish_ms,
        debug_ms
      };
      timing_pub_->publish(timing);

      std_msgs::msg::Float64 scalar;
      scalar.data = static_cast<double>(frame_count_);
      timing_frame_count_pub_->publish(scalar);
      scalar.data = static_cast<double>(segs.size());
      timing_segmentations_pub_->publish(scalar);
      scalar.data = total_ms;
      timing_total_ms_pub_->publish(scalar);
      scalar.data = convert_ms;
      timing_convert_ms_pub_->publish(scalar);
      scalar.data = infer_ms;
      timing_infer_ms_pub_->publish(scalar);
      scalar.data = publish_ms;
      timing_publish_ms_pub_->publish(scalar);
      scalar.data = debug_ms;
      timing_debug_ms_pub_->publish(scalar);
    }
  } catch (const std::exception& e) { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", e.what()); }
}

}  // namespace ros2_yolos_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_yolos_cpp::YolosSegmentorNode)
