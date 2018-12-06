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
 * @file    bot.cpp
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief Bot class implementation;
 *
 * @section DESCRIPTION
 *
 * C++ implementation for Bot class which controls motion of the robot
 */
#include "../include/bot.h"
/// Implementation of default Bot constructor
Bot::Bot(Sensor* iSensor, Camera* iCamera) {
  sensor = iSensor;
  camera = iCamera;
  nextTurnRight = false;
  nextTurnLeft = false;
  maxSpeed = 0.5;
  /// Publisher to publish messages on /navi topic
  pubVel = nh.advertise < geometry_msgs::Twist> ("/cmd_vel_mux/input/navi",1000);
  ROS_INFO("Default ROBOT Created!");
  resetBot();
}
/// Destructor that resets the velocity of the bot
Bot::~Bot() {
  resetBot();
}
/// Start the bot with obstacle avoidance functionality
void Bot::startMotion() {
  ros::Rate loop_rate(5);
  while (ros::ok()) {
    /// Get values from the sensor
    //float sensor->getRightReading() = sensor->getRightReading();
    //float leftDist = sensor->getLeftReading();
    //float forwardDist = sensor->getForwardReading();
    //int nowTurn = camera->getNowTurn();
    ROS_INFO("Right: %f, left: %f, forward: %f, nowturn: %d", sensor->getRightReading(),
             sensor->getLeftReading(), sensor->getForwardReading(),camera->getNowTurn());
    if (sensor->getObstacleDetected()) {
      ROS_INFO("Wall Detected");
      msg.linear.x = 0.0;
      pubVel.publish(msg);
      /// Adjust itself to be perpendicular to the wall
      if (!isnanf(sensor->getRightReading())
          && !isnanf(sensor->getLeftReading()) && sensor->getRightReading() < 3
          && sensor->getLeftReading() < 3 && fabs(sensor->getRightReading() - sensor->getLeftReading()) < 1) {
        double error = fabs(fabs(sensor->getRightReading()) - fabs(sensor->getLeftReading()));
        ROS_INFO("Adjusting");
        if (sensor->getRightReading() > sensor->getLeftReading() && error > 0.05 && error < 1) {
          turnLeft(error);
          ROS_INFO("Adjusting left");
        } else if (sensor->getRightReading() < sensor->getLeftReading() && error > 0.05 && error < 1) {
          turnRight(error);
          ROS_INFO("Adjusting right");
        } else {
          msg.angular.z = 0.0;
          ROS_INFO("Adjustment zero");
        }
      }
        pubVel.publish(msg);
    } else {
      ROS_INFO("Path is clear!");
      msg.angular.z = 0.0;
      msg.linear.x = maxSpeed;
    }
    /// Publish the velocity information
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
/// Set translational and rotational properties to zero
void Bot::resetBot() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVel.publish(msg);
}
/// get maximum linear speed by the bot
float Bot::getMaxSpeed() {
  return maxSpeed;
}
/// Set maximum linear speed by the bot
void Bot::setMaxSpeed(const float& speed) {
  maxSpeed = speed;
}
/// turn the robot in right direction
void Bot::turnRight(double desiredAngle) {
}
/// turn the robot in left direction
void Bot::turnLeft(double desiredAngle) {
}
/// move forward by certain distance
void Bot::moveForward(double desiredPos) {
}
/// Search for the free path
void Bot::checkFreeDirection() {
}
/// Pass through the door without colliding
void Bot::doorDetection() {
}
