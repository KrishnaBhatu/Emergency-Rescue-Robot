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
#include <math.h>
#include "../include/sensor.h"
#include "../include/camera.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// Implementation of default Bot constructor
Bot::Bot(Sensor* iSensor, Camera* iCamera)
    : sensor(iSensor),
      camera(iCamera),
      nextTurnRight(false),
      nextTurnLeft(false),
      maxSpeed(0.5),
      turnAngle(1.57),
      searchAngle(1.2) {
  // Publisher to publish messages on /navi topic
  pubVel = nh.advertise < geometry_msgs::Twist
      > ("/cmd_vel_mux/input/navi", 1000);
  ROS_INFO("Default ROBOT Created!");
  resetBot();
}
// Destructor that resets the velocity of the bot
Bot::~Bot() {
  resetBot();
}
// Start the bot with obstacle avoidance functionality
void Bot::startMotion() {
  ros::Rate loop_rate(5);
  while (ros::ok()) {
    // Get values from the sensor
    ROS_INFO("Right: %f, left: %f, forward: %f, nowturn: %d",
             sensor->getRightReading(), sensor->getLeftReading(),
             sensor->getForwardReading(), camera->getNowTurn());
    if (sensor->getObstacleDetected()) {
      ROS_INFO("Wall Detected");
      msg.linear.x = 0.0;
      pubVel.publish(msg);
      // Adjust itself to be perpendicular to the wall
      if (!isnanf(sensor->getRightReading())
          && !isnanf(sensor->getLeftReading()) && sensor->getRightReading() < 3
          && sensor->getLeftReading() < 3
          && fabs(sensor->getRightReading() - sensor->getLeftReading()) < 1) {
        auto error = fabs(
            fabs(sensor->getRightReading()) - fabs(sensor->getLeftReading()))
            * 1.0;
        ROS_INFO("Adjusting");
        if (sensor->getRightReading() > sensor->getLeftReading() && error > 0.05
            && error < 1) {
          turnLeft(error);
          ROS_INFO("Adjusting left");
        } else if (sensor->getRightReading() < sensor->getLeftReading()
            && error > 0.05 && error < 1) {
          turnRight(error);
          ROS_INFO("Adjusting right");
        } else {
          msg.angular.z = 0.0;
          ROS_INFO("Adjustment zero");
        }
      }
      pubVel.publish(msg);
      ROS_INFO("No Adjustment");
      checkFreeDirection(searchAngle, turnAngle);
    } else {
      ROS_INFO("Path is clear!");
      auto error = fabs(
          fabs(sensor->getRightReading()) - fabs(sensor->getLeftReading()))
          * 1.0;
      if (!isnanf(sensor->getRightReading())
          && !isnanf(sensor->getLeftReading())) {
        auto gain = 0.0;
        if (error > 0.5)
          gain = 0.05;
        else
          gain = 0.2;
        ROS_INFO("Error: %f, gain: %f", error, gain);
        if (sensor->getRightReading() > sensor->getLeftReading()) {
          msg.angular.z = -gain * error;
        } else {
          msg.angular.z = gain * error;
        }
      }
      if (camera->getNowTurn() == 10 && sensor->getRightReading() > 3
          && !nextTurnRight) {
        nextTurnRight = true;
        nextTurnLeft = false;
        camera->setNowTurn(0);
        camera->setCountB(0);
        camera->setSignDetected(false);
      } else if (camera->getNowTurn() == 5 && sensor->getLeftReading() > 3
          && !nextTurnLeft) {
        nextTurnLeft = true;
        nextTurnRight = false;
        camera->setNowTurn(0);
        camera->setCount(0);
        camera->setSignDetected(false);
      } else if (camera->getNowTurn() == 15) {
          msg.angular.z = 0.0;
        doorDetection();
        return;
        }
      if (nextTurnRight) {
        msg.angular.z = -0.48;
        ROS_INFO_STREAM("Right Turn Initiate");
          camera->setNowTurn(0);
        if (isnanf(sensor->getForwardReading())
            || sensor->getForwardReading() > 5) {
          nextTurnRight = false;
          msg.angular.z = 0;
          camera->setCountB(0);
          camera->setNowTurn(0);
        }
        }
      if (nextTurnLeft) {
        msg.angular.z = 0.48;
        ROS_INFO_STREAM("Left Turn Initiate");
          camera->setNowTurn(0);
        if (isnanf(sensor->getForwardReading())
            || sensor->getForwardReading() > 5) {
          nextTurnLeft = false;
          msg.angular.z = 0;
          camera->setCount(0);
          camera->setNowTurn(0);
        }
        }
      msg.linear.x = maxSpeed;
      }
    // Publish the velocity information
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
// Set translational and rotational properties to zero
void Bot::resetBot() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVel.publish(msg);
}
// get maximum linear speed by the bot
float Bot::getMaxSpeed() {
  return maxSpeed;
}
// turn the robot in right direction
void Bot::turnRight(double desiredAngle) {
  ROS_INFO("Turning in right direction");
  ros::Rate loop_rate(10);
  desiredAngle = desiredAngle - sensor->getCurrentYaw();
  while (ros::ok()) {
    auto error = fabs(fabs(sensor->getCurrentYaw()) - fabs(desiredAngle)) * 1.0;
    msg.angular.z = -1.5 * error;
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    if (error <= 0.01 && error >= -0.99) {
      ROS_INFO_STREAM("break");
      break;
    }
  }
}
// turn the robot in left direction
void Bot::turnLeft(double desiredAngle) {
  ROS_INFO("Turning in left direction");
  ros::Rate loop_rate(10);
  desiredAngle = sensor->getCurrentYaw() + desiredAngle;
  while (ros::ok()) {
    auto error = fabs(fabs(desiredAngle) - fabs(sensor->getCurrentYaw())) * 1.0;
    msg.angular.z = 1.5 * error;
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    if (error <= 0.01 && error >= -0.99) {
      ROS_INFO_STREAM("break");
      break;
    }
  }
}
// move forward by certain distance
void Bot::moveForward(double desiredPos) {
  ros::Time start = ros::Time::now();
  ros::Duration duration(2, 0);
  ros::Rate loop_rate(10);
  // velocity = distance / time
  maxSpeed = desiredPos / 2;
  while ((ros::Time::now() - start) < duration) {
    msg.linear.x = maxSpeed;
    msg.angular.z = 0.0;
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
// Search for the free path
void Bot::checkFreeDirection(double searchAngle, double turnAngle) {
  ROS_INFO("Checking free direction");
  // door reached
  if (sensor->getForwardReading() > 10 && searchAngle == 0.0
      && turnAngle == 0.0) {
    sensor->setObstacleDetected(false);
    return;
  }
  // Get values from the sensor
  auto safeDistance = sensor->getSafeDistance();
  turnRight(searchAngle);
  // turn 60 and get right value which will be 180deg
  // value for original configuration
  ROS_INFO("Right reading: %f", sensor->getRightReading());
  auto extremeRightVal = sensor->getRightReading();
  turnLeft(searchAngle);
  turnLeft(searchAngle);
  // turn 60 and get left value which will be 180deg
  // value for original configuration
  ROS_INFO("Left reading: %f", sensor->getLeftReading());
  auto extremeLeftVal = sensor->getLeftReading();
  turnRight(searchAngle);
  ROS_INFO("Straight: %f, Right: %f, Left: %f", sensor->getForwardReading(),
           sensor->getRightReading(),
           sensor->getLeftReading());
  if (isnanf(sensor->getForwardReading()) || sensor->getForwardReading() > 4) {
    ROS_INFO("Forward Path is longer");
    moveForward(1);
    maxSpeed = 0.5;
    return;
  }
  if (isnanf(extremeRightVal) || sensor->getRightReading() > 5) {
    ROS_INFO("Right Path is longer");
    turnRight(turnAngle);
    return;
  }
  if (isnanf(extremeLeftVal) || sensor->getLeftReading() > 5) {
    ROS_INFO("Left Path is longer");
    turnLeft(turnAngle);
    return;
  }
  if (sensor->getForwardReading() < safeDistance
      && extremeRightVal > extremeLeftVal
      && extremeRightVal > 3) {
    ROS_INFO("Right Path is longer");
    turnRight(turnAngle);
    return;
  }
  if (sensor->getForwardReading() < safeDistance
      && extremeRightVal < extremeLeftVal
      && extremeLeftVal > 3) {
    ROS_INFO("Left Path is longer");
    turnLeft(turnAngle);
    return;
  } else {
    ROS_INFO("Turn Around");
    turnLeft(turnAngle);
    turnLeft(turnAngle);
    return;
  }
}
// Pass through the door without colliding
void Bot::doorDetection() {
  ros::Time start = ros::Time::now();
  ros::Duration duration(12, 0);
  ros::Rate loop_rate(10);
  while ((ros::Time::now() - start) < duration) {
    auto rightDist = sensor->getRightReading();
    auto leftDist = sensor->getLeftReading();
    auto forwardDist = sensor->getForwardReading();
    ROS_INFO("Door detection");
    ROS_INFO("Left: %f, Forward: %f, Right: %f", leftDist, rightDist,
             forwardDist);
    if (isnanf(sensor->getForwardReading()))
      forwardDist = 5;
    if (isnanf(sensor->getRightReading()))
      rightDist = 5;
    if (isnanf(sensor->getLeftReading()))
      leftDist = 5;
    msg.linear.x = 0.3;
    auto error = fabs(fabs(rightDist) - fabs(leftDist)) * 1.0;
    auto gain = 0.2;
    ROS_INFO("Error: %f, gain: %f", error, gain);
    if (rightDist > leftDist) {
      msg.angular.z = -gain * error;
    } else {
      msg.angular.z = gain * error;
    }
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void Bot::setSearchAngle(double search) {
  searchAngle = search;
}
void Bot::setTurnAngle(double turn) {
  turnAngle = turn;
}
