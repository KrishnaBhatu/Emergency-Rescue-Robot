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
 * @file    sensor.h
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief Sensor class source file;
 *
 * @section DESCRIPTION
 *
 * C++ header file for Sensor class which collects lazer and odometry
 * sensor data.
 */
#ifndef INCLUDE_SENSOR_H_
#define INCLUDE_SENSOR_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
class Sensor {
 private:
  /// Subscriber object
  ros::Subscriber subSensor;
  /// Subscriber to Odom
  ros::Subscriber odom;
  /// Sensor reading at 0 deg
  float forwardReading;
  /// Sensor reading at -30 deg
  float rightReading;
  /// Sensor reading at 30 deg
  float leftReading;
  /// Obstacle detected
  bool obstacleDetected;
  /// Allowable distance between bot and obstacle
  double safeDistance;
  /// Current yaw reading of the robot
  double currentYaw;
  /// Current x coordinate of the robot
  double currentX;
  /// Initiate NodeHandle object
  ros::NodeHandle nh;
 public:
  /**
   * @brief Default Sensor class constructor
   */
  Sensor();
  /**
   * @brief Default Sensor class destructor
   */
  ~Sensor();
  /**
   * @brief Callback method for laser subsciber
   * @param msg laser data
   * @return void
   */
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
   * @brief Callback method for odom subscriber
   * @param msg odometry data
   * @return void
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  /**
   * @brief Get sensor reading at 0deg
   * @return float
   */
  float getForwardReading();
  /**
   * @brief Get sensor reading at -30deg
   * @return float
   */
  float getRightReading();
  /**
   * @brief Get sensor reading at 30deg
   * @return float
   */
  float getLeftReading();
  /**
   * @brief check if obstacle is detected
   * @return bool
   */
  bool getObstacleDetected();
  /**
   * @brief Get max safe distance
   * @return double
   */
  double getSafeDistance();
  /**
   * @brief Get sensor reading of current yaw angle
   * @return double
   */
  double getCurrentYaw();
  /**
   * @brief Get sensor reading of current x position
   * @return double
   */
  double getCurrentX();
  /**
   * @brief Set current value of forward reading
   * @param fwd distance value
   * @return void
   */
  void setForwardReading(float fwd);
  /**
   * @brief Set current value of the right reading
   * @param right right distance value
   * @return void
   */
  void setRightReading(float right);
  /**
   * @brief Set current value of the left reading
   * @param left left distance value
   * @return void
   */
  void setLeftReading(float left);
  /**
   * @brief Set current value of the x position
   * @param cX current x
   * @return void
   */
  void setCurrentX(double cX);
  /**
   * @brief Set current value of the yaw angle
   * @param yaw yaw angle
   * @return void
   */
  void setCurrentYaw(double yaw);
  /**
   * @brief Set status of obstacle detection
   * @param obs true of false value
   * @return void
   */
  void setObstacleDetected(bool obs);
};
#endif // INCLUDE_SENSOR_H_
