// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_detector/detector.hpp"

namespace
{
rm_auto_aim::Detector makeDetector()
{
  rm_auto_aim::Detector::LightParams light_params{0.1, 0.4, 40.0};
  rm_auto_aim::Detector::ArmorParams armor_params{0.7, 0.8, 3.2, 3.2, 5.5, 35.0};
  return rm_auto_aim::Detector(80, rm_auto_aim::RED, light_params, armor_params);
}

void expectRectangularLights(const cv::Scalar & rgb_color, int expected_color)
{
  cv::Mat image = cv::Mat::zeros(120, 165, CV_8UC3);
  cv::rectangle(image, cv::Rect(30, 38, 5, 36), rgb_color, cv::FILLED);
  cv::rectangle(image, cv::Rect(115, 38, 5, 36), rgb_color, cv::FILLED);

  auto detector = makeDetector();
  auto binary = detector.preprocessImage(image);
  auto lights = detector.findLights(image, binary);

  ASSERT_EQ(lights.size(), 2u);
  for (const auto & light : lights) {
    EXPECT_EQ(light.color, expected_color);
  }
}
}  // namespace

TEST(DetectorTest, FindsFourPointBlueLights)
{
  expectRectangularLights(cv::Scalar(0, 255, 255), rm_auto_aim::BLUE);
}

TEST(DetectorTest, FindsRedLightsAtThresholdEighty)
{
  expectRectangularLights(cv::Scalar(255, 0, 0), rm_auto_aim::RED);
}

TEST(DetectorTest, KeepsGeometricArmorWithoutNumberClassifier)
{
  cv::Mat image = cv::Mat::zeros(120, 165, CV_8UC3);
  cv::rectangle(image, cv::Rect(30, 38, 5, 36), cv::Scalar(0, 255, 255), cv::FILLED);
  cv::rectangle(image, cv::Rect(115, 38, 5, 36), cv::Scalar(0, 255, 255), cv::FILLED);

  auto detector = makeDetector();
  detector.detect_color = rm_auto_aim::BLUE;
  detector.use_number_classifier = false;
  auto armors = detector.detect(image);

  ASSERT_EQ(armors.size(), 1u);
  EXPECT_TRUE(armors.front().number.empty());
  EXPECT_EQ(armors.front().classfication_result, "unclassified");
}
