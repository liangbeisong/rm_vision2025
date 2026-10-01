#ifndef ARMOR_DETECTOR__COARSE_NUMBER_MAPPING_HPP_
#define ARMOR_DETECTOR__COARSE_NUMBER_MAPPING_HPP_

#include <opencv2/core.hpp>

#include <string>

namespace rm_auto_aim
{
// Empirical image mapping used only until the two cameras have been calibrated.
class CoarseNumberMapping
{
public:
  bool load(const std::string & path);

  bool predict(
    const cv::Point2f & main_center, double depth, const cv::Size & main_size,
    const cv::Size & number_size, double padding, cv::Rect2f & roi, std::string & reason) const;

  bool ready() const { return ready_; }

private:
  bool ready_ = false;
  int main_width_ = 0;
  int main_height_ = 0;
  int number_width_ = 0;
  int number_height_ = 0;
  double min_depth_ = 0.0;
  double max_depth_ = 0.0;
  double min_u_ = 0.0;
  double max_u_ = 0.0;
  double min_v_ = 0.0;
  double max_v_ = 0.0;
  cv::Mat center_coeff_;  // 2 x 4: normalized u, v, 1, 1 / depth
  cv::Mat size_coeff_;    // 2 x 2: 1, 1 / depth
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__COARSE_NUMBER_MAPPING_HPP_
