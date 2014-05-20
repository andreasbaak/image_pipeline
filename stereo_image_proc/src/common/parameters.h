/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, BMW Car IT GmbH
*  Author: Andreas Baak (andreas.baak@bmw.de)
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the BMW Car IT GmbH nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <ros/ros.h>
#include <string>

const std::string CS_EAST_UP_SOUTH_STR("EAST_UP_SOUTH");
const std::string CS_EAST_NORTH_UP_STR("EAST_NORTH_UP");

enum CoordinateSystem
{
    CS_EAST_UP_SOUTH,
    CS_EAST_NORTH_UP,
    CS_NONE
};

int toInt(const CoordinateSystem c) {
    return static_cast<int>(c);
}

CoordinateSystem fromInt(const int i) {
    if (i >=0 && i < CS_NONE) {
        return static_cast<CoordinateSystem>(i);
    } else {
        ROS_WARN("Coordinate system for integer id %d is not known. Assuming default coordinate system.", i);
        return CS_EAST_UP_SOUTH;
    }
}

std::string toString(const CoordinateSystem c) {
    switch (c) {
    case CS_EAST_UP_SOUTH:
        return CS_EAST_UP_SOUTH_STR;
    case CS_EAST_NORTH_UP:
        return CS_EAST_NORTH_UP_STR;
    default:
        ROS_WARN("toString(c) for the chosen coordinate system c=%d is not implemented.", toInt(c));
        ROS_WARN("Returning default coordinate system %s", CS_EAST_UP_SOUTH_STR.c_str());
        return CS_EAST_UP_SOUTH_STR;
    }
}

CoordinateSystem fromString(const std::string& str) {
    if (str == CS_EAST_UP_SOUTH_STR) {
        return CS_EAST_UP_SOUTH;
    } else if (str == CS_EAST_NORTH_UP_STR) {
        return CS_EAST_NORTH_UP;
    } else {
        ROS_WARN("Coordinate system for string '%s' is not known. Returning default coordinate system.", str.c_str());
        return CS_EAST_UP_SOUTH;
    }
}

#endif /* PARAMETERS_H_ */
