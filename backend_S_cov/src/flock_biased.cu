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

#include <cmath>

#include "cub/cub.cuh"

#include "common.h"

#include "utility.cuh"

#include "flock_biased.cuh"

__device__
void behavior_flock_biased_stage1(BehaviorContext * context, int i, int j) {
  BehaviorFlockBiasedContext * ctx = 
    static_cast<BehaviorFlockBiasedContext *>(context);
  typedef cub::BlockReduce<float, ROBOTS, cub::BLOCK_REDUCE_RAKING_COMMUTATIVE_ONLY> BlockReduce;
  __shared__ typename BlockReduce::TempStorage storage;

  float dx = ctx->poses.x[j] - ctx->poses.x[i];
  float dy = ctx->poses.y[j] - ctx->poses.y[i];
  float heading_j = ctx->poses.theta[j];
  float heading_i = ctx->poses.theta[i];
  float dtheta = heading_j - heading_i;
  float squared_distance = dx*dx + dy*dy;
  float distance = sqrtf(squared_distance);
  float dvx = 0.0f;
  float dvy = 0.0f;
  float dw = 0.0f;
  if (distance == 0.0f) {
    // Do nothing - should only happen for (i == j)
  } else if (distance < REPULSION_RADIUS) {
    dvx = -dx / squared_distance;
    dvy = -dy / squared_distance;
    dw = atan2f(dvy, dvx) - heading_i;
  } else if (distance < ALIGNMENT_RADIUS) {
    dvx = 0.0f;
    dvy = 0.0f;
    dw = dtheta;
  } else if (distance < ATTRACTION_RADIUS) {
    dvx = dx;
    dvy = dy;
    dw = atan2f(dvy, dvx) - heading_i;
  }
  float delta_theta = wrapToPi(dw);
  int c = (distance < ATTRACTION_RADIUS);
  float n = BlockReduce(storage).Sum(c); __syncthreads();
  float vx = BlockReduce(storage).Sum(dvx*c); __syncthreads();
  float vy = BlockReduce(storage).Sum(dvy*c); __syncthreads();
  float w = BlockReduce(storage).Sum(delta_theta*c); __syncthreads();

  if (j == 0) {
    ctx->velocity_x[i] = vx / n;
    ctx->velocity_y[i] = vy / n;
    ctx->angular[i] = w / n;
  }
}

__device__
void behavior_flock_biased_stage2(BehaviorContext * context, int i) {
  BehaviorFlockBiasedContext * ctx = 
    static_cast<BehaviorFlockBiasedContext *>(context);
  float bias_x = ctx->bias_x;
  float bias_y = ctx->bias_y;
  float vx = ctx->velocity_x[i] + bias_x;
  float vy = ctx->velocity_y[i] + bias_y;
  float theta = ctx->poses.theta[i];
  float w = wrapToPi(ctx->angular[i]) + wrapToPi(atan2f(bias_y, bias_x) - theta);
  float bx = cosf(theta);
  float by = sinf(theta);
  ctx->control_v[i] = GAIN_V * (vx*bx + vy*by);
  ctx->control_w[i] = GAIN_W * w;
}
