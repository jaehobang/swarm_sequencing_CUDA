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

#ifndef FORMATION_CONTROL_H
#define FORMATION_CONTROL_H

#include "common.h"

const float CONNECTIVITY_RADIUS = 20.0f;
const float LINE_SPACING = 2.0f;

struct BehaviorFormationControlContext : public BehaviorContext {
  float formation_x[ROBOTS];
  float formation_y[ROBOTS];
  float velocity_x[ROBOTS];
  float velocity_y[ROBOTS];
};

struct BehaviorLineXContext : public BehaviorFormationControlContext {
  __host__ __device__
  BehaviorLineXContext()
  {
    for (int i=0; i<ROBOTS; i++) {
      formation_x[i] = i * LINE_SPACING;
      formation_y[i] = 0.0f;
    }
  }
};

struct BehaviorLineYContext : public BehaviorFormationControlContext {
  __host__ __device__
  BehaviorLineYContext()
  {
    for (int i=0; i<ROBOTS; i++) {
      formation_x[i] = 0.0f;
      formation_y[i] = i * LINE_SPACING;
    }
  }
};

__device__ 
void behavior_formation_control_stage1(BehaviorContext * context, int i, int j);

__device__ 
void behavior_formation_control_stage2(BehaviorContext * context, int i);

#endif // FORMATION_CONTROL_H
