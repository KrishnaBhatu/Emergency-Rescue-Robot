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
 * @file    bot.h
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief Bot class implementation;
 *
 * @section DESCRIPTION
 *
 * C++ header file for Bot class which controls motion of the robot .
 */
#ifndef INCLUDE_BOT_H_
#define INCLUDE_BOT_H_
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "sensor.h"
#include "camera.h"
class Bot {
 private:

  /// Check if robot is in safe decision
  bool obstacleDetected;
  /// Publisher msg of type Twist
  geometry_msgs::Twist msg;
  /// Publisher object
  ros::Publisher pubVel;
  /// Subscriber object
  ros::Subscriber subSensor;
  /// Subscriber to Odom
  ros::Subscriber odom;
  /// Subscriber to images
  ros::Subscriber subImage;
  /// MaxSpeed of turtlebot
  float maxSpeed;
  /// Initiate NodeHandle object
  ros::NodeHandle nh;
  /// Turn right next
  bool nextTurnRight;
  /// Turn left next
  bool nextTurnLeft;
  /// Object of Sensor class
  Sensor* sensor;
  /// Object of Camera class
  Camera* camera;
 public:
  /**
   * @brief Bot class constructor
   */
  Bot(Sensor* iSensor, Camera* iCamera);
  /**
   * @brief Default destructor
   */
  ~Bot();
  /**
   * @brief Start running the bot
   * @return void
   */
  void startMotion();
  /**
   * @brief reset velocities of the bot
   * @return void
   */
  void resetBot();
  /**
   * @brief turn robot in right direction
   * @param desired_angle turn angle in radians
   * @return void
   */
  void turnRight(double desiredAngle);
  /**
   * @brief turn robot in left direction
   * @param desired_angle turn angle in radians
   * @return void
   */
  void turnLeft(double desiredAngle);
  /**
   * @brief move forward by some distance
   * @param desiredPos distancez
   * @return void
   */
  void moveForward(double desiredPos);
  /**
   * @brief if sign is not present make decisions
   * based on sensor reading to turn left or right
   */
  void checkFreeDirection();
  /**
   * @brief if door is detected, move through the door
   */
  void doorDetection();
  /**
   * @brief get maximum speed of robot
   * @return float
   */
  float getMaxSpeed();
};
#endif  // INCLUDE_BOT_H_
