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

#ifndef SIMULATOR_PIPELINE_H
#define SIMULATOR_PIPELINE_H

#include "common.h"

#include "behavior.cuh"
#include "robot_model.cuh"
#include "utility.cuh"

// SimulatorStage0: Setup context
__global__
void simulatorStage0(SwarmState * state, BehaviorManager * manager, int behaviorId);

// SimulatorStage1: All to all swarm member computations
__global__
void simulatorStage1(BehaviorManager * manager, int behaviorId);

// SimulatorStage2: All to individual reductions
__global__
void simulatorStage2(BehaviorManager * manager, int behaviorId, SwarmState * state);

__global__
void simulatorStage3(BehaviorManager * manager, int behaviorId, float* dCost);

#endif // SIMULATOR_PIPELINE_H
