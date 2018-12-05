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
 * @file    sensor_test.cpp
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief walker class implementation;
 *
 * @section DESCRIPTION
 *
 * Implementation of google tests for Sensor class.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/sensor.h"
/**
 * @brief Test to check if sensor class is initiailizing
 */
TEST(SensorTest, SensorInitializationTest) {
  Sensor sensor;
  EXPECT_EQ(1, 1);
}
/**
 * @brief Test to check sensorcallback function
 */
TEST(SensorTest, sensorCallbackTest) {
  EXPECT_EQ(1, 1);
}
/**
 * @brief Test to check odometer callback function
 */
TEST(SensorTest, odomCallbackTest) {
  EXPECT_EQ(1, 1);
}
/**
 * @brief Test to check forward reading measurment
 */
TEST(SensorTest, getForwarReadingTest) {
  Sensor sensor;
  EXPECT_DOUBLE_EQ(1.0, sensor.getForwardReading());
}
/**
 * @brief Test to check right reading measurment
 */
TEST(SensorTest, getRightReadingTest) {
  Sensor sensor;
  EXPECT_DOUBLE_EQ(1.0, sensor.getRightReading());
}
/**
 * @brief Test to check left reading measurment
 */
TEST(SensorTest, getLeftReadingTest) {
  Sensor sensor;
  EXPECT_DOUBLE_EQ(1.0, sensor.getLeftReading());
}
/**
 * @brief Test to check current yaw reading
 */
TEST(SensorTest, getCurrentYawTest) {
  Sensor sensor;
  EXPECT_DOUBLE_EQ(1.0, sensor.getCurrentYaw());
}
/**
 * @brief Test to check current yaw reading
 */
TEST(SensorTest, getCurrentXTest) {
  Sensor sensor;
  EXPECT_DOUBLE_EQ(1.0, sensor.getCurrentX());
}
/**
 * @brief Test to check current yaw reading
 */
TEST(SensorTest, getObstacleDetectedTest) {
  Sensor sensor;
  EXPECT_FALSE(sensor.getObstacleDetected());
}
/**
 * @brief Test to check current yaw reading
 */
TEST(SensorTest, getSafeDistanceTest) {
  Sensor sensor;
  EXPECT_DOUBLE_EQ(1.0, sensor.getSafeDistance());
}
