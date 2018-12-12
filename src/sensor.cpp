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
 * @file    sensor.cpp
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief walker class implementation;
 *
 * @section DESCRIPTION
 *
 * C++ implementation for Sensor class which collects lazer and odometry
 * sensor data.
 */
#include "../include/sensor.h"
Sensor::Sensor()
    : obstacleDetected(false),
      safeDistance(1.2),
      forwardReading(0),
      rightReading(0),
      leftReading(0) {
  /// Subscribe to /scan topic where we get obstacle distance
  subSensor = nh.subscribe < sensor_msgs::LaserScan
       > ("/scan", 10, &Sensor::sensorCallback, this);
  /// Subscribe to /odom topic to get the position
  odom = nh.subscribe("/odom", 10, &Sensor::odomCallback, this);
}
Sensor::~Sensor() {
}
void Sensor::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  /// ranges array contains float values of sensor reading
  rightReading = msg->ranges[0];
  forwardReading = msg->ranges[319];
  leftReading = msg->ranges[msg->ranges.size() - 1];
  /// Read all readings of the sensor and check if any object/wall
  /// is within the safe distance.
  for (const float &m : msg->ranges) {
    if (m < safeDistance) {
      obstacleDetected = true;
      return;
    }
  }
  obstacleDetected = false;
}
void Sensor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double r, p;
  m.getRPY(r, p, currentYaw);
  currentX = msg->pose.pose.position.x;
}
float Sensor::getForwardReading() {
  return forwardReading;
}
float Sensor::getRightReading() {
  return rightReading;
}
float Sensor::getLeftReading() {
  return leftReading;
}
double Sensor::getCurrentYaw() {
  return currentYaw;
}
double Sensor::getCurrentX() {
  return currentX;
}
bool Sensor::getObstacleDetected() {
  return obstacleDetected;
}
double Sensor::getSafeDistance() {
  return safeDistance;
}
