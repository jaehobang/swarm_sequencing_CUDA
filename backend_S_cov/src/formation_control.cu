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

#include "formation_control.cuh"

__device__
void behavior_formation_control_stage1(BehaviorContext * context, int i, int j) {
  BehaviorFormationControlContext * ctx = 
    static_cast<BehaviorFormationControlContext *>(context);
  typedef cub::BlockReduce<float, ROBOTS, cub::BLOCK_REDUCE_RAKING_COMMUTATIVE_ONLY> BlockReduce;
  __shared__ typename BlockReduce::TempStorage storage;

  float dx = ctx->poses.x[j] - ctx->poses.x[i];
  float dy = ctx->poses.y[j] - ctx->poses.y[i];
  float fx = ctx->formation_x[j] - ctx->formation_x[i];
  float fy = ctx->formation_y[j] - ctx->formation_y[i];
  float distance = sqrtf(dx*dx + dy*dy);
  float delta_x = dx - fx;
  float delta_y = dy - fy;
  int c = (distance < CONNECTIVITY_RADIUS);
  float n = BlockReduce(storage).Sum(c); __syncthreads();
  float vx = BlockReduce(storage).Sum(delta_x*c); __syncthreads();
  float vy = BlockReduce(storage).Sum(delta_y*c); __syncthreads();

  if (j == 0) {
    ctx->velocity_x[i] = vx / n;
    ctx->velocity_y[i] = vy / n;
  }
}

__device__
void behavior_formation_control_stage2(BehaviorContext * context, int i) {
  BehaviorFormationControlContext * ctx = 
    static_cast<BehaviorFormationControlContext *>(context);
  float vx = ctx->velocity_x[i];
  float vy = ctx->velocity_y[i];
  float theta = ctx->poses.theta[i];
  float bx = cosf(theta);
  float by = sinf(theta);
  float w = wrapToPi(atan2f(vy, vx) - theta);
  ctx->control_v[i] = GAIN_V * (vx*bx + vy*by);
  ctx->control_w[i] = GAIN_W * w;
}

