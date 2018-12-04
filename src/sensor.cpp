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
Sensor::Sensor() {
  /// Subscribe to /scan topic where we get obstacle distance
   subSensor = nh.subscribe < sensor_msgs::LaserScan
       > ("/scan", 10, &Sensor::sensorCallback, this);
   /// Subscribe to /odom topic to get the position
   odom = nh.subscribe("/odom", 10, &Sensor::odomCallback, this);
}
Sensor::~Sensor() {
  
}
void Sensor::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  /// Callback function of the Laser Data, code for using laser distance data
}
void Sensor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  /** Callback function of the Odometry data, code for using current poistion 
    * and orientation of turtlebot
    */
}
float Sensor::getForwardReading() {
  return 1.0;
}
float Sensor::getRightReading() {
  return 1.0;
}
float Sensor::getLeftReading() {
  return 1.0;
}
double Sensor::getCurrentYaw() {
  return 1.0;
}
double Sensor::getCurrentX() {
  return 1.0;
}
bool Sensor::getObstacleDetected() {
  return false;
}
double Sensor::getSafeDistance() {
  return 1.0;
}
