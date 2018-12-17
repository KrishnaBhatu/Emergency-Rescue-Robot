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
 * @file    bot_test.cpp
 * @author  Krishna Bhatu, Siddhesh Rane
 * @version 1.0
 * @brief walker class implementation;
 *
 * @section DESCRIPTION
 *
 * Implementation of google tests for Bot class.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/bot.h"
/**
 * @brief Test to check if Bot class is initiailizing
 */
TEST(BotTest, botInitialization) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  EXPECT_DOUBLE_EQ(0.5, bot.getMaxSpeed());
}
/**
 * @brief Test robot motion function
 */
TEST(BotTest, turnRightTest) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  sensor.setCurrentYaw(1.2);
  bot.turnRight(0.005);
  EXPECT_DOUBLE_EQ(1.2, sensor.getCurrentYaw());
}
/**
 * @brief Test robot motion function
 */
TEST(BotTest, turnLeftTest) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  sensor.setCurrentYaw(1.2);
  bot.turnLeft(0.0);
  EXPECT_DOUBLE_EQ(1.2, sensor.getCurrentYaw());
}
/**
 * @brief Test moveFoward function
 */
TEST(BotTest, moveForwardTest) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  bot.moveForward(2.0);
  EXPECT_EQ(1.0, bot.getMaxSpeed());
}
/**
 * @brief Test door detection function
TEST(BotTest, doorDetectionTest) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  sensor.setForwardReading(2.0);
  sensor.setRightReading(2.0);
  sensor.setLeftReading(2.0);
  bot.doorDetection();
}
/**
 * @brief Test move Forward
 */
TEST(BotTest, startMotionTest) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  camera.setNowTurn(15);
  bot.startMotion();
}
/**
 * @brief Test move Forward
 */
TEST(BotTest, checkFreeDirectionTest) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  sensor.setForwardReading(5);
  bot.checkFreeDirection(0.0, 0.0);
  sensor.setForwardReading(0.5);
  sensor.setRightReading(6);
  bot.checkFreeDirection(0.0, 0.0);

  sensor.setRightReading(0.5);
  sensor.setLeftReading(6);
  bot.checkFreeDirection(0.0, 0.0);

  sensor.setForwardReading(0.5);
  sensor.setRightReading(4);
  sensor.setLeftReading(3);
  bot.checkFreeDirection(0.0, 0.0);

  sensor.setRightReading(3);
  sensor.setLeftReading(4);
  bot.checkFreeDirection(0.0, 0.0);

  sensor.setRightReading(0.5);
  sensor.setLeftReading(0.5);
  bot.checkFreeDirection(0.0, 0.0);
}
/**
 * @brief obstacle detection test
 */
TEST(BotTest, obstacleDetectionTest) {
  Camera camera;
  Sensor sensor;
  Bot bot(&sensor, &camera);
  sensor.setObstacleDetected(true);
  sensor.setForwardReading(12);
  camera.setNowTurn(15);
  bot.setSearchAngle(0.0);
  bot.setTurnAngle(0.0);
  bot.startMotion();
}
