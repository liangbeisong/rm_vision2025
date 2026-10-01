#include "armor_detector/coarse_number_mapping.hpp"

#include <cmath>

namespace rm_auto_aim
{
bool CoarseNumberMapping::load(const std::string & path)
{
  ready_ = false;
  cv::FileStorage file(path, cv::FileStorage::READ);
  if (!file.isOpened()) return false;

  file["main_width"] >> main_width_;
  file["main_height"] >> main_height_;
  file["number_width"] >> number_width_;
  file["number_height"] >> number_height_;
  file["min_depth"] >> min_depth_;
  file["max_depth"] >> max_depth_;
  file["min_u"] >> min_u_;
  file["max_u"] >> max_u_;
  file["min_v"] >> min_v_;
  file["max_v"] >> max_v_;
  file["center_coeff"] >> center_coeff_;
  file["size_coeff"] >> size_coeff_;

  if (
    main_width_ <= 0 || main_height_ <= 0 || number_width_ <= 0 || number_height_ <= 0 ||
    min_depth_ <= 0.0 || max_depth_ <= min_depth_ || min_u_ >= max_u_ || min_v_ >= max_v_ ||
    center_coeff_.rows != 2 || center_coeff_.cols != 4 || size_coeff_.rows != 2 ||
    size_coeff_.cols != 2) {
    return false;
  }
  center_coeff_.convertTo(center_coeff_, CV_64F);
  size_coeff_.convertTo(size_coeff_, CV_64F);
  ready_ = true;
  return true;
}

bool CoarseNumberMapping::predict(
  const cv::Point2f & main_center, double depth, const cv::Size & main_size,
  const cv::Size & number_size, double padding, cv::Rect2f & roi, std::string & reason) const
{
  if (!ready_) {
    reason = "mapping_unavailable";
    return false;
  }
  if (
    main_size.width != main_width_ || main_size.height != main_height_ ||
    number_size.width != number_width_ || number_size.height != number_height_) {
    reason = "image_size_mismatch";
    return false;
  }
  if (!std::isfinite(depth) || depth < min_depth_ || depth > max_depth_) {
    reason = "depth_out_of_range";
    return false;
  }

  const double u = main_center.x / main_width_;
  const double v = main_center.y / main_height_;
  if (u < min_u_ || u > max_u_ || v < min_v_ || v > max_v_) {
    reason = "position_out_of_range";
    return false;
  }

  const cv::Mat input = (cv::Mat_<double>(4, 1) << u, v, 1.0, 1.0 / depth);
  const cv::Mat size_input = (cv::Mat_<double>(2, 1) << 1.0, 1.0 / depth);
  const cv::Mat center = center_coeff_ * input;
  const cv::Mat size = size_coeff_ * size_input;
  const double x = center.at<double>(0) * number_width_;
  const double y = center.at<double>(1) * number_height_;
  const double width = size.at<double>(0) * number_width_ * padding;
  const double height = size.at<double>(1) * number_height_ * padding;

  if (
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(width) ||
    !std::isfinite(height) || width < 4.0 || height < 4.0) {
    reason = "invalid_projection";
    return false;
  }
  roi = cv::Rect2f(
    static_cast<float>(x - width / 2.0), static_cast<float>(y - height / 2.0),
    static_cast<float>(width), static_cast<float>(height));
  if (
    roi.x < 0.0f || roi.y < 0.0f || roi.br().x > number_width_ ||
    roi.br().y > number_height_) {
    reason = "projection_outside_image";
    return false;
  }
  reason.clear();
  return true;
}
}  // namespace rm_auto_aim
