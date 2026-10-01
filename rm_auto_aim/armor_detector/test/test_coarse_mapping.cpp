#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <cstdio>
#include <string>
#include <unistd.h>

#include "armor_detector/coarse_number_mapping.hpp"

TEST(CoarseNumberMapping, RejectsUnsafeProjections)
{
  const std::string path = "/tmp/test_coarse_mapping_" + std::to_string(getpid()) + ".yaml";
  {
    cv::FileStorage file(path, cv::FileStorage::WRITE);
    file << "main_width" << 100 << "main_height" << 100;
    file << "number_width" << 100 << "number_height" << 100;
    file << "min_depth" << 1.0 << "max_depth" << 8.0;
    file << "min_u" << 0.1 << "max_u" << 0.9;
    file << "min_v" << 0.1 << "max_v" << 0.9;
    file << "center_coeff" << (cv::Mat_<double>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    file << "size_coeff" << (cv::Mat_<double>(2, 2) << 0.2, 0, 0.2, 0);
  }
  rm_auto_aim::CoarseNumberMapping mapping;
  ASSERT_TRUE(mapping.load(path));
  std::remove(path.c_str());

  cv::Rect2f roi;
  std::string reason;
  EXPECT_TRUE(mapping.predict({50, 50}, 3, {100, 100}, {100, 100}, 1, roi, reason));
  EXPECT_NEAR(roi.x, 40, 0.01);
  EXPECT_NEAR(roi.y, 40, 0.01);
  EXPECT_FALSE(mapping.predict({50, 50}, 9, {100, 100}, {100, 100}, 1, roi, reason));
  EXPECT_EQ(reason, "depth_out_of_range");
  EXPECT_FALSE(mapping.predict({95, 50}, 3, {100, 100}, {100, 100}, 1, roi, reason));
  EXPECT_EQ(reason, "position_out_of_range");
  EXPECT_FALSE(mapping.predict({50, 50}, 3, {120, 100}, {100, 100}, 1, roi, reason));
  EXPECT_EQ(reason, "image_size_mismatch");
  EXPECT_FALSE(mapping.predict({85, 50}, 3, {100, 100}, {100, 100}, 2, roi, reason));
  EXPECT_EQ(reason, "projection_outside_image");
}
