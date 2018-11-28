/**
 * 3-clause BSD License
 *
 * Copyright (c) 2018 Krishna Bhatu, Siddhesh Rane
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 * the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file bot.hpp
 * @version 1.0
 * @author Siddhesh Rane, Krishna Bhatu
 * @brief Bot class implementation
 *
 * @section DESCRIPTION
 *
 * Header of the Bot class which defines properties and functionalities of the
 * robot to navigate in the unknown environment.
 */
#ifndef INCLUDE_BOT_HPP_
#define INCLUDE_BOT_HPP_
#include "ros/ros.h"
class Bot {
 private:
  float currVel;
  float currRotation;
  float maxVelocity;
  float maxAngularVel;
  bool goalReached;
  bool signDetected;
  int noOfSignsDetected;
  bool obstacleDetected;
  float nextDirection;
 public:
  Bot();
  void moveForward();
  void stop();
  void turnAround(float angle);
  float interpretSign();
  void followSign();
  }
};
#endif // INCLUDE_BOT_HPP_