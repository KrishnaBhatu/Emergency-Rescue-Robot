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
 * @file    camera_test.cpp
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief walker class implementation;
 *
 * @section DESCRIPTION
 *
 * Implementation of google tests for Camera class.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "../include/camera.h"
/**
 * @brief Test to check if Camera class is initiailizing
 */
TEST(CameraTest, CameraInitializationTest) {
  Camera camera;
  EXPECT_EQ(0, camera.getNowTurn());
  EXPECT_EQ(0, camera.getCount());
  EXPECT_EQ(0, camera.getCountB());
  EXPECT_FALSE(camera.getSignDetected());
}
/**
 * @brief Test to check checkImage is detecting signs
 */
TEST(CameraTest, checkImageTest) {
  Camera camera;
  ros::WallDuration(5, 0).sleep();
  ros::spinOnce();
  EXPECT_FALSE(camera.getSignDetected());
}
/**
 * @brief Test to check if nowTurn is being set
 */
TEST(CameraTest, setNowTurnTest) {
  Camera camera;
  camera.setNowTurn(15);
  EXPECT_EQ(15, camera.getNowTurn());
}
/**
 * @brief Test to check if setCount is being set
 */
TEST(CameraTest, setCountTest) {
  Camera camera;
  camera.setCount(30);
  EXPECT_EQ(30, camera.getCount());
}
/**
 * @brief Test to check if setCountB is being set
 */
TEST(CameraTest, setCountBTest) {
  Camera camera;
  camera.setCountB(10);
  EXPECT_EQ(10, camera.getCountB());
}
/**
 * @brief Test to check if signDetected is being set
 */
TEST(CameraTest, setSignDetectedTest) {
  Camera camera;
  camera.setSignDetected(true);
  EXPECT_TRUE(camera.getSignDetected());
  camera.setSignDetected(false);
  EXPECT_FALSE(camera.getSignDetected());
}
/**
 * @brief Test to check if green sign is detected
 */
TEST(CameraTest, greenSignTest) { 
  Camera camera;
  cv::Mat image = cv::imread("images/test1.png",1);
  int i = 0;
  while(i<45){
   camera.imageProcessing(image);
   i++;
  }
   EXPECT_EQ(5, camera.getNowTurn()); 
}
/**
 * @brief Test to check if blue sign is detected
 */
TEST(CameraTest, blueSignTest) { 
  Camera camera;
  cv::Mat image = cv::imread("images/test3.png",1);
  int i = 0;
  while(i<46){
   camera.imageProcessing(image);
   i++;
  }
  //EXPECT_TRUE(camera.getSignDetected());
  EXPECT_EQ(10, camera.getNowTurn()); 
}
/**
 * @brief Test to check if red sign is detected
 */
TEST(CameraTest, redSignTest) { 
  Camera camera;
  cv::Mat image = cv::imread("images/test2.png",1);
  int i = 0;
  camera.imageProcessing(image);
  EXPECT_EQ(15, camera.getNowTurn()); 
}
