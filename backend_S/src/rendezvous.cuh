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

#ifndef RENDEZVOUS_H
#define RENDEZVOUS_H

#include "common.h"

const float RENDEZVOUS_CONNECTIVITY_RADIUS = 20.0f;

struct BehaviorRendezvousCommonContext : public BehaviorContext {
  float gain;
  float velocity_x[ROBOTS];
  float velocity_y[ROBOTS];
};

struct BehaviorRendezvousContext : public BehaviorRendezvousCommonContext {
  __host__ __device__
  BehaviorRendezvousContext()
  {
    gain = 1.0f;
  }
};

struct BehaviorAntiRendezvousContext : public BehaviorRendezvousCommonContext {
  __host__ __device__
  BehaviorAntiRendezvousContext()
  {
    gain = -1.0f;
  }
};

__device__ 
void behavior_rendezvous_stage1(BehaviorContext * context, int i, int j);

__device__ 
void behavior_rendezvous_stage2(BehaviorContext * context, int i);

#endif // RENDEZVOUS_H
