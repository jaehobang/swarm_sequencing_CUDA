/*
 *  Copyright 2017 Sasanka Nagavalli
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <algorithm>

#include <cmath>

#include "common.h"

#include "utility.cuh"

const float MAX_LINEAR_VELOCITY = 8.0f;
const float MAX_ANGULAR_VELOCITY = M_PI/8.0f;

__forceinline__ __host__ __device__
void applyRobotModel(BehaviorContext * ctx, int i)
{
  float u_v = doClamp(ctx->control_v[i], -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  float u_w = doClamp(ctx->control_w[i], -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  float theta = ctx->poses.theta[i];
  float dx = u_v*std::cos(theta)*TIME_STEP_DT;
  float dy = u_v*std::sin(theta)*TIME_STEP_DT;
  float dtheta = u_w*TIME_STEP_DT;
  float distance_travelled = std::sqrt(dx*dx + dy*dy);
  ctx->travelled[i] += distance_travelled;  
  ctx->poses.x[i] += dx; 
  ctx->poses.y[i] += dy;
  ctx->poses.theta[i] = wrapToPi(theta + dtheta);
}

__forceinline__ __host__ __device__
bool outsideMapLimits(BehaviorContext * ctx, int i, MapLimits * limits)
{
  float x = ctx->poses.x[i];
  float y = ctx->poses.y[i];
  return (x < limits->minX) || (y < limits->minY) ||
         (x > limits->maxX) || (y > limits->maxY);
}

__forceinline__ __host__ __device__
bool robotIntersectsObstacle(BehaviorContext * ctx, Obstacle * obs, int i) 
{
  float dx = obs->x - ctx->poses.x[i];
  float dy = obs->y - ctx->poses.y[i];
  float r = obs->radius + ROBOT_RADIUS;
  return (dx*dx + dy*dy < r*r);
}

#endif // ROBOT_MODEL_H
