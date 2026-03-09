#include "ros2_yolos_cpp/nodes/segmentor_service_node.hpp"
#include "ros2_yolos_cpp/conversion/segmentation_converter.hpp"

#include <cv_bridge/cv_bridge.h>
#include <mutex>

namespace ros2_yolos_cpp
{

YolosSegmentorServiceNode::YolosSegmentorServiceNode(
  const rclcpp::NodeOptions & options)
: Node("yolos_segmentor_service", options)
{
  declareParameters();
  auto config = loadConfig();

  if (config.model_path.empty() || config.labels_path.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "model_path and labels_path must be set");
    throw std::runtime_error("Invalid parameters");
  }

  segmentor_ = std::make_shared<SegmentorAdapter>();

  if (!segmentor_->initialize(config)) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to initialize segmentor");
    throw std::runtime_error("Segmentor init failed");
  }

  service_group_ =
    create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  service_ = create_service<srv::SegmentImage>(
    "~/segment",
    std::bind(
      &YolosSegmentorServiceNode::handleRequest,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_);

  RCLCPP_INFO(
    get_logger(),
    "Segmentation service ready.");
}

void YolosSegmentorServiceNode::declareParameters()
{
  declare_parameter("model_path", "");
  declare_parameter("labels_path", "");
  declare_parameter("use_gpu", false);
  declare_parameter("conf_threshold", 0.4);
  declare_parameter("nms_threshold", 0.45);
}

YolosConfig YolosSegmentorServiceNode::loadConfig()
{
  YolosConfig c;
  c.model_path = get_parameter("model_path").as_string();
  c.labels_path = get_parameter("labels_path").as_string();
  c.use_gpu = get_parameter("use_gpu").as_bool();
  c.conf_threshold =
    static_cast<float>(get_parameter("conf_threshold").as_double());
  c.nms_threshold =
    static_cast<float>(get_parameter("nms_threshold").as_double());
  return c;
}

void YolosSegmentorServiceNode::handleRequest(
  const std::shared_ptr<srv::SegmentImage::Request> req,
  std::shared_ptr<srv::SegmentImage::Response> res)
{
  RCLCPP_INFO(
    get_logger(),
    "Received segmentation request (conf_threshold=%.2f, nms_threshold=%.2f)",
    req->conf_threshold, req->nms_threshold);

  static std::mutex inference_mutex;  // GPU protection

  if (!segmentor_ || !segmentor_->isInitialized()) {
    res->success = false;
    res->message = "Segmentor not initialized";
    RCLCPP_ERROR(
      get_logger(),
      "Segmentor not initialized");
    return;
  }

  try {
    auto cv = cv_bridge::toCvCopy(req->image, "bgr8");

    float conf = req->conf_threshold > 0 ?
      req->conf_threshold :
      get_parameter("conf_threshold").as_double();

    float nms = req->nms_threshold > 0 ?
      req->nms_threshold :
      get_parameter("nms_threshold").as_double();

    std::lock_guard<std::mutex> lock(inference_mutex);

    auto segs =
      segmentor_->segment(cv->image, conf, nms);

    RCLCPP_INFO(
      get_logger(),
      "Segmentation completed: %zu segmentations",
      segs.size());

    res->detections =
      conversion::toDetection2DArray(
      segs,
      req->image.header,
      req->image.width,
      req->image.height);

    res->mask =
      conversion::toCombinedMaskImage(
      segs,
      req->image.header,
      req->image.width,
      req->image.height);

    if (req->return_debug) {
      cv::Mat debug = cv->image.clone();
      segmentor_->drawSegmentations(debug, segs);
      res->debug_image =
        *cv_bridge::CvImage(
        req->image.header,
        "bgr8",
        debug).toImageMsg();
    }

    res->success = true;
    res->message = "OK";

    RCLCPP_INFO(
      get_logger(),
      "Segmentation completed: %zu segmentations",
      segs.size());

  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

}  // namespace ros2_yolos_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  ros2_yolos_cpp::YolosSegmentorServiceNode)
