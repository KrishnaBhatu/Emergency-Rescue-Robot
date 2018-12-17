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
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
/**
 * @brief Test to check sensor initiailization
 */
TEST(SensorTest, SensorInitializationTest) {
  Sensor sensor;
  EXPECT_EQ(1.2, sensor.getSafeDistance());
}
/**
 * @brief Test to check sensorcallback function
 */
TEST(SensorTest, sensorCallbackTest) {
  ros::NodeHandle nh;
  ros::Publisher mockPub = nh.advertise < sensor_msgs::LaserScan
      > ("/scan", 50);
  sensor_msgs::LaserScan mockLaserData;
  mockLaserData.angle_min = -0.52;
  mockLaserData.angle_max = 0.52;
  mockLaserData.angle_increment = 0.0016;
  mockLaserData.time_increment = 0.0;
  mockLaserData.range_min = 0.44;
  mockLaserData.range_max = 3.0;
  mockLaserData.ranges.resize(50);
  mockLaserData.intensities.resize(50);

  for (auto& i : mockLaserData.ranges) {
    i = 0.0;
  }
  int counter = 0;

  while (ros::ok()) {
    mockPub.publish(mockLaserData);
    Sensor sensor;

    if (counter == 3) {
      break;
    }
    ros::spinOnce();
    EXPECT_EQ(1, mockPub.getNumSubscribers());
    counter++;
  }
}
/**
 * @brief Test to check odometer callback function
 */
TEST(SensorTest, odomCallbackTest) {
  ros::NodeHandle nh;
  ros::Publisher mockPub = nh.advertise < nav_msgs::Odometry > ("/odom", 50);
  nav_msgs::Odometry mockOdom;
  mockPub.publish(mockOdom);
  Sensor sensor;
  EXPECT_EQ(1, mockPub.getNumSubscribers());
}
/**
 * @brief Test to check forward reading measurment
 */
TEST(SensorTest, getForwarReadingTest) {
  Sensor sensor;
  sensor.setForwardReading(2.0);
  EXPECT_DOUBLE_EQ(2.0, sensor.getForwardReading());
}
/**
 * @brief Test to check right reading measurment
 */
TEST(SensorTest, getRightReadingTest) {
  Sensor sensor;
  sensor.setRightReading(2.0);
  EXPECT_DOUBLE_EQ(2.0, sensor.getRightReading());
}
/**
 * @brief Test to check left reading measurment
 */
TEST(SensorTest, getLeftReadingTest) {
  Sensor sensor;
  sensor.setLeftReading(2.0);
  EXPECT_DOUBLE_EQ(2.0, sensor.getLeftReading());
}
/**
 * @brief Test to check current yaw reading
 */
TEST(SensorTest, getSafeDistanceTest) {
  Sensor sensor;
  EXPECT_DOUBLE_EQ(1.2, sensor.getSafeDistance());
}
/**
 * @brief Test to check obstacle detection flag
 */
TEST(SensorTest, getObstacleDetectionTest) {
  Sensor sensor;
  sensor.setObstacleDetected(false);
  EXPECT_FALSE(sensor.getObstacleDetected());
}
/**
 * @brief Get currentX test
 */
TEST(SensorTest, getCurrentXTest) {
  Sensor sensor;
  sensor.setCurrentX(1.0);
  EXPECT_DOUBLE_EQ(1.0, sensor.getCurrentX());
}
