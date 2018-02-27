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

#ifndef PLANNER_PIPELINE_H
#define PLANNER_PIPELINE_H

#include "common.h"

#include "robot_model.cuh"
#include "utility.cuh"

// StageA1: Initialize the nodes to be expanded
__global__
void plannerStageA1(Node * nodesIn, Node * nodesOut, DurationSequence * durations);

// StageA2: Initialize the initial poses for each node to be expanded
__global__
void plannerStageA2(Node * nodesIn, Node * nodesOut, BehaviorManager * manager);

// StageB1: All to all swarm member computations
__global__
void plannerStageB1(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, DurationSequence * durations);

// StageB2: Individual updates
__global__
void plannerStageB2(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, DurationSequence * durations, MapLimits * mapLimits);

// StageC1: Obstacle checks
__global__
void plannerStageC1(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, DurationSequence * durations, Obstacle * obstacles);

// StageC1_5: Obstacle checks
__global__
void plannerStageC1_5(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, DurationSequence * durations, Obstacle_n * obstacles);

// StageC2: Update time step
__global__
void plannerStageC2(Node * nodesIn, Node * nodesOut, DurationSequence * durations);

// StageD1: Compute cost and heuristics
__global__
void plannerStageD1(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, Target * target, float robotRadius, float * priorities, Node * bestNode);

#endif // PLANNER_PIPELINE_H
