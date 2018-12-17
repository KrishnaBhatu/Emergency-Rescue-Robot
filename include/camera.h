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
 * @file    camera.h
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief Camera class source code;
 *
 * @section DESCRIPTION
 *
 * C++ header file for Camera class which collects image data.
 */
#ifndef INCLUDE_CAMERA_H_
#define INCLUDE_CAMERA_H_
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
class Camera {
 private:
  /// Subscriber to images
  ros::Subscriber subImage;
  /// Pointer to an image
  cv_bridge::CvImagePtr cv_ptr;
  /// HSV vlaue for Red lower bound
  cv::Scalar lowerRed;
  /// HSV vlaue for Red higher bound
  cv::Scalar higherRed;
  /// HSV vlaue for Green lower bound
  cv::Scalar lowerGreen;
  /// HSV vlaue for Green higher bound
  cv::Scalar higherGreen;
  /// HSV vlaue for Blue lower bound
  cv::Scalar lowerBlue;
  /// HSV vlaue for Blue higher bound
  cv::Scalar higherBlue;
  /// Indicate when to start to turn as per the sign
  int nowTurn;
  /// Sign position tracker for green arrow
  int count;
  /// Sign position tracker for bluw arrow
  int countB = 0;
  /// Check if sign is detected
  bool signDetected;
  /// Initiate NodeHandle object
  ros::NodeHandle nh;
 public:
  /**
    * @brief Default constructor for Camera
    */
  Camera();
  /**
    * @brief Default destructor for Camera
    */
  ~Camera();
  /**
    * @brief Callback function for camera subscriber
    * @param msg data from camera node
    * @return void
    */
  void checkImage(const sensor_msgs::Image::ConstPtr& dataImage);
  /**
    * @brief Get status for making turning decision
    * @return int
    */
  int getNowTurn();
  /**
    * @brief Get value of sign tracker for green arrow
    * @return int
    */
  int getCount();
  /**
    * @brief Get value of sign tracker for blue arrow
    * @return int
    */
  int getCountB();
  /**
    * @brief Check if sign is detected
    * @return bool
    */
  bool getSignDetected();
  /**
    * @brief Set value of the integer which represents the direction of turn
   * @param turn nowTurn value to be set
    * @return void
    */
  void setNowTurn(int turn);
  /**
    * @brief Set value of the counter which determines the time after which the
    *        robot has to take turn after detecting the left turning point
   * @param c blue count value to be set
    * @return void
    */
  void setCount(int c);
  /**
    * @brief Set value of the counter which determines the time after which the
    *        robot has to take turn after detecting the right turning point
   * @param cB green count value to be set
    * @return void
    */
  void setCountB(int cB);
  /**
    * @brief Set the flag when sign is detected
   * @param sD boolean value that is given to the flag
    * @return void
    */
  void setSignDetected(bool sD);
  /**
   * @brief Start processing the image received from img sensor
   * @param image openCv image
   * @return void
   */
  void imageProcessing(cv::Mat image);
};
#endif // INCLUDE_CAMERA_H_
