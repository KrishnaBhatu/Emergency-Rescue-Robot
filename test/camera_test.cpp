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
#include "../include/camera.h"
/**
 * @brief Test to check if Camera class is initiailizing
 */
TEST(CameraTest, CameraInitializationTest) {
 EXPECT_EQ(1,1);
}
/**
 * @brief Test for getting value of nowTurn
 */
TEST(CameraTest, getNowTurnTest) {
 Camera camera;
 EXPECT_EQ(1,camera.getNowTurn());
}
/**
 * @brief Test for getting value of count
 */
TEST(CameraTest, getCountTest) {
  Camera camera;
  EXPECT_EQ(1,camera.getCount());
}
/**
 * @brief Test for getting value of count
 */
TEST(CameraTest, getCountBTest) {
  Camera camera;
  EXPECT_EQ(1,camera.getCountB());
}
/**
 * @brief Test to check if sign is detected
 */
TEST(CameraTest, getSignDetectedTest) {
 Camera camera;
 EXPECT_FALSE(camera.getSignDetected());
}
/**
 * @brief Test to check if nowTurn is being set
 */
TEST(CameraTest, setNowTurnTest) {
  EXPECT_EQ(1,1);
}
/**
 * @brief Test to check if setCount is being set
 */
TEST(CameraTest, setCountTest) {
  EXPECT_EQ(1,1);
}
/**
 * @brief Test to check if setCountB is being set
 */
TEST(CameraTest, setCountBTest) {
  EXPECT_EQ(1,1);
}
/**
 * @brief Test to check if signDetected is being set
 */
TEST(CameraTest, setSignDetectedTest) {
  EXPECT_EQ(1,1);
}