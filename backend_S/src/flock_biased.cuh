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

#ifndef FLOCK_BIASED_H
#define FLOCK_BIASED_H

#include "common.h"

const float REPULSION_RADIUS = 5.0f;
const float ALIGNMENT_RADIUS = 10.0f;
const float ATTRACTION_RADIUS = 20.0f;

struct BehaviorFlockBiasedContext : public BehaviorContext {
  float bias_x;
  float bias_y;
  float velocity_x[ROBOTS];
  float velocity_y[ROBOTS];
  float angular[ROBOTS];
};

struct BehaviorFlockingContext : public BehaviorFlockBiasedContext {
  __host__ __device__
  BehaviorFlockingContext()
  {
    bias_x = 0.0f;
    bias_y = 0.0f;
  }
};

struct BehaviorFlockNorthContext : public BehaviorFlockBiasedContext {
  __host__ __device__
  BehaviorFlockNorthContext()
  {
    bias_x = 0.0f;
    bias_y = 1.0f;
  }
};

struct BehaviorFlockEastContext : public BehaviorFlockBiasedContext {
  __host__ __device__
  BehaviorFlockEastContext()
  {
    bias_x = 1.0f;
    bias_y = 0.0f;
  }
};

struct BehaviorFlockSouthContext : public BehaviorFlockBiasedContext {
  __host__ __device__
  BehaviorFlockSouthContext()
  {
    bias_x = 0.0f;
    bias_y = -1.0f;
  }
};

struct BehaviorFlockWestContext : public BehaviorFlockBiasedContext {
  __host__ __device__
  BehaviorFlockWestContext()
  {
    bias_x = -1.0f;
    bias_y = 0.0f;
  }
};

__device__ 
void behavior_flock_biased_stage1(BehaviorContext * context, int i, int j);

__device__
void behavior_flock_biased_stage2(BehaviorContext * context, int i);

#endif // FLOCK_BIASED_H
