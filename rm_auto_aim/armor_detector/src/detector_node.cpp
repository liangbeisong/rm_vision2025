// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Detector
  detector_ = initDetector();

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS().keep_last(1));

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      if (camera_info->header.frame_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Camera info frame_id is empty");
      }
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  number_source_ = this->declare_parameter("number_source", std::string("mindvision"));
  max_frame_delta_ms_ = this->declare_parameter("max_frame_delta_ms", 30);
  main_calibration_width_ = this->declare_parameter("main_calibration_width", 1440);
  main_calibration_height_ = this->declare_parameter("main_calibration_height", 1080);
  number_roi_padding_ = this->declare_parameter("number_roi_padding", 1.0);
  if (number_source_ == "video4") {
    const auto mapping_path = this->declare_parameter("number_mapping_path", std::string(""));
    if (!mapping_path.empty() && !number_mapping_.load(mapping_path)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid number mapping: %s", mapping_path.c_str());
    }
    if (!number_mapping_.ready()) {
      RCLCPP_ERROR(this->get_logger(), "No fitted number mapping; all candidates will be rejected");
    }
    const auto qos = rclcpp::SensorDataQoS().keep_last(10).get_rmw_qos_profile();
    main_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_raw", qos);
    number_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/number_camera/image_raw", qos);
    image_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *main_filter_, *number_filter_);
    image_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_frame_delta_ms_ / 1000.0));
    image_sync_->registerCallback(
      std::bind(&ArmorDetectorNode::pairedImageCallback, this, std::placeholders::_1,
      std::placeholders::_2));
    last_pair_time_ = this->now();
    pair_watchdog_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
      if ((this->now() - last_pair_time_).seconds() > 2.0) {
        RCLCPP_WARN(this->get_logger(), "No synchronized number-camera frames within %d ms",
          max_frame_delta_ms_);
      }
    });
  } else {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
  }
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  processImage(img_msg, nullptr);
}

void ArmorDetectorNode::pairedImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr main_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr number_msg)
{
  last_pair_time_ = this->now();
  processImage(main_msg, number_msg);
}

void ArmorDetectorNode::processImage(
  const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr number_msg)
{
  auto armors = detectArmors(img_msg);
  const bool dual = number_source_ == "video4";
  cv::Mat number_image;
  cv::Mat number_debug;
  std::vector<cv::Rect2f> projected(armors.size());
  std::vector<std::string> reasons(armors.size());
  std::vector<cv::Mat> crops;
  if (dual && number_msg) {
    const auto delta_ns = std::llabs(
      (rclcpp::Time(img_msg->header.stamp) - rclcpp::Time(number_msg->header.stamp)).nanoseconds());
    if (delta_ns > static_cast<int64_t>(max_frame_delta_ms_) * 1000000) {
      std::fill(reasons.begin(), reasons.end(), "timestamp_mismatch");
    } else {
      number_image = cv_bridge::toCvShare(number_msg, "rgb8")->image;
      if (debug_) number_debug = number_image.clone();
    }
  }

  if (pnp_solver_ != nullptr) {
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    if (dual && cam_info_ &&
      (cam_info_->width != img_msg->width || cam_info_->height != img_msg->height ||
      main_calibration_width_ != static_cast<int>(img_msg->width) ||
      main_calibration_height_ != static_cast<int>(img_msg->height))) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MindVision calibration expected %dx%d, camera_info %ux%u, image %ux%u; rejecting mapping",
        main_calibration_width_, main_calibration_height_, cam_info_->width, cam_info_->height,
        img_msg->width, img_msg->height);
      std::fill(reasons.begin(), reasons.end(), "main_intrinsics_size_mismatch");
    }
    std::vector<cv::Mat> rvecs(armors.size()), tvecs(armors.size());
    for (size_t i = 0; i < armors.size(); ++i) {
      if (!pnp_solver_->solvePnP(armors[i], rvecs[i], tvecs[i])) {
        reasons[i] = "pnp_failed";
        continue;
      }
      if (dual && tvecs[i].at<double>(2) <= 0.0) {
        reasons[i] = "pnp_behind_camera";
        continue;
      }
      if (dual && reasons[i].empty() && !number_image.empty()) {
        number_mapping_.predict(
          armors[i].center, cv::norm(tvecs[i]),
          cv::Size(img_msg->width, img_msg->height), number_image.size(),
          number_roi_padding_, projected[i], reasons[i]);
      } else if (dual && reasons[i].empty()) {
        reasons[i] = "number_frame_unavailable";
      }
    }
    if (dual) {
      for (size_t i = 0; i < armors.size(); ++i) {
        if (!reasons[i].empty()) continue;
        for (size_t j = i + 1; j < armors.size(); ++j) {
          if (!reasons[j].empty()) continue;
          const auto intersection = projected[i] & projected[j];
          if (intersection.area() > 0.0f) {
            reasons[i] = reasons[j] = "ambiguous_number_roi";
          }
        }
      }
    }
    for (size_t i = 0; i < armors.size(); ++i) {
      auto armor = armors[i];
      if (!reasons[i].empty()) continue;
      if (dual) {
        const auto roi = projected[i];
        const cv::Rect pixel_roi(
          cv::Point(cvFloor(roi.x), cvFloor(roi.y)),
          cv::Point(cvCeil(roi.br().x), cvCeil(roi.br().y)));
        if ((pixel_roi & cv::Rect(0, 0, number_image.cols, number_image.rows)) != pixel_roi) {
          reasons[i] = "projection_outside_image";
          continue;
        }
        cv::Mat crop = number_image(pixel_roi);
        cv::resize(crop, crop, cv::Size(20, 28));
        cv::cvtColor(crop, armor.number_img, cv::COLOR_RGB2GRAY);
        cv::threshold(armor.number_img, armor.number_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        crops.emplace_back(armor.number_img);
        std::vector<Armor> classified{armor};
        detector_->classifier->classify(classified);
        if (classified.empty() || classified.front().number == "negative") {
          reasons[i] = "number_rejected";
          continue;
        }
        armor = classified.front();
      }
      const auto & rvec = rvecs[i];
      const auto & tvec = tvecs[i];
      if (!tvec.empty()) {
        // Fill basic info
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        // Fill the markers
        armor_marker_.id++;
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
        armor_marker_.pose = armor_msg.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      }
    }
    if (dual && debug_) {
      for (size_t i = 0; i < armors.size(); ++i) {
        if (!number_debug.empty() && projected[i].area() > 0) {
          cv::rectangle(number_debug, projected[i],
            reasons[i].empty() ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0), 2);
          cv::putText(number_debug, reasons[i].empty() ? "accepted" : reasons[i],
            projected[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
      }
      if (!number_debug.empty()) number_result_img_pub_.publish(
        cv_bridge::CvImage(number_msg->header, "rgb8", number_debug).toImageMsg());
      if (!crops.empty()) {
        cv::Mat mosaic;
        cv::hconcat(crops, mosaic);
        number_crop_img_pub_.publish(
          cv_bridge::CvImage(number_msg->header, "mono8", mosaic).toImageMsg());
      }
      std_msgs::msg::String status;
      for (size_t i = 0; i < reasons.size(); ++i) {
        status.data += std::to_string(i) + ":" + (reasons[i].empty() ? "accepted" : reasons[i]) + " ";
      }
      const auto total_latency_ms = (this->now() - img_msg->header.stamp).seconds() * 1000.0;
      status.data += "latency_ms:" + std::to_string(total_latency_ms);
      number_status_pub_->publish(status);
    }

    // Publishing detected armors
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    publishMarkers();
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  detector->use_number_classifier = this->declare_parameter("use_number_classifier", true);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Update params
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->detect_color = get_parameter("detect_color").as_int();
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();
  detector_->use_number_classifier =
    number_source_ != "video4" && get_parameter("use_number_classifier").as_bool();

  auto armors = detector_->detect(img);

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // Publish debug info
  if (debug_) {
    binary_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty() && detector_->use_number_classifier) {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    detector_->drawResults(img);
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

void ArmorDetectorNode::createDebugPublishers()
{
  auto image_qos = rmw_qos_profile_sensor_data;
  image_qos.depth = 1;
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img", image_qos);
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img", image_qos);
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img", image_qos);
  number_result_img_pub_ = image_transport::create_publisher(
    this, "/detector/number_camera_result_img", image_qos);
  number_crop_img_pub_ = image_transport::create_publisher(
    this, "/detector/number_camera_crop", image_qos);
  number_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/detector/number_camera_status", 10);
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
  number_result_img_pub_.shutdown();
  number_crop_img_pub_.shutdown();
  number_status_pub_.reset();
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
