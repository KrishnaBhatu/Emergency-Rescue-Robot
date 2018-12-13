/**
 * BSD 3-Clause License
 * @copyright (c) 2018, Krishna Bhatu, Siddhesh Rane
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @file    camera.cpp
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief Camera class implementation;
 *
 * @section DESCRIPTION
 *
 * C++ implementation for Camera which collects image data.
 */
#include "../include/camera.h"
Camera::Camera()
    : signDetected(false),
      nowTurn(0),
      count(0),
      countB(0) {
  // Subscribe to /camera/rgb/image_raw topic
    subImage = nh.subscribe < sensor_msgs::Image
        > ("/camera/rgb/image_raw", 10, &Camera::checkImage, this);
}
Camera::~Camera() {
}
void Camera::checkImage(const sensor_msgs::Image::ConstPtr& dataImage) {
  try {
    cv_ptr = cv_bridge::toCvCopy(dataImage, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    return;
  }
  lowerRed = cv::Scalar(0, 70, 50);
  higherRed = cv::Scalar(10, 255, 250);
  lowerGreen = cv::Scalar(45, 100, 50);
  higherGreen = cv::Scalar(65, 255, 255);
  lowerBlue = cv::Scalar(110, 50, 50);
  higherBlue = cv::Scalar(130, 255, 255);
  cv::Mat imageHSV;
  cv::Mat mask;
  cv::Mat res;
  cv::Mat res_gray;
  cv::Mat thresh;
  cv::cvtColor(cv_ptr->image, imageHSV, cv::COLOR_BGR2HSV);
  cv::inRange(imageHSV, lowerGreen, higherGreen, mask);
  cv::bitwise_and(imageHSV, imageHSV, res, mask);
  cv::cvtColor(res, res_gray, CV_BGR2GRAY);
  cv::threshold(res_gray, thresh, 40, 255, cv::THRESH_BINARY);
  for (auto i = 0; i < thresh.size().height; i++) {
    for (auto j = 0; j < thresh.size().width; j++) {
      if (thresh.at<int>(i, j) == 255) {
        signDetected = true;
        if (i < 2) {
          count = count + 1;
          ROS_INFO("count: %d", count);
          if (count > 45) {
            count = 0;
            nowTurn = 5;
          }
        }
      }
    }
  }
  cv::Mat imageHSVB;
  cv::Mat maskB;
  cv::Mat resB;
  cv::Mat res_grayB;
  cv::Mat threshB;
  cv::cvtColor(cv_ptr->image, imageHSVB, cv::COLOR_BGR2HSV);
  cv::inRange(imageHSVB, lowerBlue, higherBlue, maskB);
  cv::bitwise_and(imageHSVB, imageHSVB, resB, maskB);
  cv::cvtColor(resB, res_grayB, CV_BGR2GRAY);
  cv::threshold(res_grayB, threshB, 40, 255, cv::THRESH_BINARY);
  for (auto i = 0; i < threshB.size().height; i++) {
    for (auto j = 0; j < threshB.size().width; j++) {
      if (threshB.at<int>(i, j) == 255) {
        signDetected = true;
        if (i < 2) {
          countB = countB + 1;
          if (countB > 45) {
            countB = 0;
            nowTurn = 10;
          }
        }
        break;
      }
    }
  }
  cv::Mat imageHSVR;
  cv::Mat maskR;
  cv::Mat resR;
  cv::Mat res_grayR;
  cv::Mat threshR;
  cv::cvtColor(cv_ptr->image, imageHSVR, cv::COLOR_BGR2HSV);
  cv::inRange(imageHSVR, lowerRed, higherRed, maskR);
  cv::bitwise_and(imageHSVR, imageHSVR, resR, maskR);
  cv::cvtColor(resR, res_grayR, CV_BGR2GRAY);
  cv::threshold(res_grayR, threshR, 40, 255, cv::THRESH_BINARY);
  for (auto i = 0; i < threshR.size().height; i++) {
    for (auto j = 0; j < threshR.size().width; j++) {
      if (threshR.at<int>(i, j) == 255) {
        signDetected = true;
        if (i < 2) {
          nowTurn = 15;
        }
        break;
      }
    }
  }
}
int Camera::getNowTurn() {
  return nowTurn;
}
int Camera::getCount() {
  return count;
}
int Camera::getCountB() {
  return countB;
}
bool Camera::getSignDetected() {
  return signDetected;
}
void Camera::setNowTurn(int turn) {
  nowTurn = turn;
}
void Camera::setCount(int c) {
  count = c;
}
void Camera::setCountB(int cB) {
  countB = cB;
}
void Camera::setSignDetected(bool sD) {
  signDetected = sD;
}
