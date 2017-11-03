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

#include "cub/cub.cuh"

#include "common.h"

#include "behavior.cuh"
#include "robot_model.cuh"
#include "utility.cuh"

__global__
void plannerStageA1(Node * nodesIn, Node * nodesOut, DurationSequence * durations)
{
  int n = blockIdx.x;
  int b = threadIdx.x;
  int q = n*BEHAVIORS + b;
  nodesOut[q].sequence = nodesIn[n].sequence;
  nodesOut[q].valid = (nodesOut[q].sequence.length < durations->length);
  if (nodesOut[q].valid) {
    nodesOut[q].sequence.ids[nodesOut[q].sequence.length] = b;
    nodesOut[q].sequence.length++;
    nodesOut[q].t = nodesIn[q].t;
  }
}

__global__
void plannerStageA2(Node * nodesIn, Node * nodesOut, BehaviorManager * manager)
{
  int n = blockIdx.x;
  int b = blockIdx.y;
  int i = threadIdx.x;
  BehaviorContext * ctx = manager->getBehavior(b)->getContext(n);
  ctx->poses.x[i] = nodesIn[n].poses.x[i];
  ctx->poses.y[i] = nodesIn[n].poses.y[i];
  ctx->poses.theta[i] = nodesIn[n].poses.theta[i];
  ctx->travelled[i] = 0.0f;
}

__global__
void plannerStageB1(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, DurationSequence * durations)
{
  int n = blockIdx.x;
  int b = blockIdx.y;
  int i = blockIdx.z;
  int j = threadIdx.x;
  int q = n*BEHAVIORS + b;
  if (nodesOut[q].valid) {
    int duration = durations->times[nodesIn[n].sequence.length];
    if (nodesOut[q].t < nodesIn[n].t + duration) {
      Behavior * behavior = manager->getBehavior(b);
      BehaviorContext * ctx = behavior->getContext(n);
      behavior->stage1(ctx, i, j);
    }
  }
}

__global__
void plannerStageB2(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, 
  DurationSequence * durations, MapLimits * mapLimits)
{
  int n = blockIdx.x;
  int b = blockIdx.y;
  int i = threadIdx.x;
  int q = n*BEHAVIORS + b;
  if (nodesOut[q].valid) {
    int duration = durations->times[nodesIn[n].sequence.length];
    if (nodesOut[q].t < nodesIn[n].t + duration) {
      Behavior * behavior = manager->getBehavior(b);
      BehaviorContext * ctx = behavior->getContext(n);
      behavior->stage2(ctx, i);
      applyRobotModel(ctx, i);
      if (outsideMapLimits(ctx, i, mapLimits)) {
        nodesOut[q].valid = false;
      }
    }
  }
}

__global__
void plannerStageC1(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, 
  DurationSequence * durations, Obstacle * obstacles)
{
  int n = blockIdx.x;
  int b = blockIdx.y;
  int o = blockIdx.z;
  int i = threadIdx.x;
  int q = n*BEHAVIORS + b;
  if (nodesOut[q].valid) {
    int duration = durations->times[nodesIn[n].sequence.length];
    if (nodesOut[q].t < nodesIn[n].t + duration) {
      Behavior * behavior = manager->getBehavior(b);
      BehaviorContext * ctx = behavior->getContext(n);
      if (robotIntersectsObstacle(ctx, obstacles+o, i)) {
        nodesOut[q].valid = false;
      }
    }
  }
}

__global__
void plannerStageC2(Node * nodesIn, Node * nodesOut, DurationSequence * durations)
{
  int n = blockIdx.x;
  int b = threadIdx.x;
  int q = n*BEHAVIORS + b;
  if (nodesOut[q].valid) {
    int duration = durations->times[nodesIn[n].sequence.length];
    if (nodesOut[q].t < nodesIn[n].t + duration) {
      nodesOut[q].t++;
    }
  }
}

__global__
void plannerStageD1(Node * nodesIn, Node * nodesOut, BehaviorManager * manager, Target * target, float robotRadius, float * priorities, Node * bestNode)
{
  int n = blockIdx.x;
  int b = blockIdx.y;
  int i = threadIdx.x;
  int q = n*BEHAVIORS + b;
  if (nodesOut[q].valid) {
    BehaviorContext * ctx = manager->getBehavior(b)->getContext(n);
    float x = ctx->poses.x[i];
    float y = ctx->poses.y[i];
    float theta = ctx->poses.theta[i];
    nodesOut[q].poses.x[i] = x;
    nodesOut[q].poses.y[i] = y;
    nodesOut[q].poses.theta[i] = theta;
    float dx = target->x - x;
    float dy = target->y - y;
    float distance = std::sqrt(dx*dx + dy*dy) - (target->radius - robotRadius);
    if (distance < 0.0f) { distance = 0.0f; }

    typedef cub::BlockReduce<float, ROBOTS> BlockReduce;
    __shared__ typename BlockReduce::TempStorage storage;
    float travelled = ctx->travelled[i];
    float cost = nodesIn[n].cost + BlockReduce(storage).Sum(travelled); 
    __syncthreads();
    float heuristic = HEURISTIC_WEIGHT*BlockReduce(storage).Sum(distance); 
    __syncthreads();

    if (i == 0) {
      nodesOut[q].cost = cost;
      nodesOut[q].heuristic = heuristic;
      float cost_estimate = cost + heuristic; 
      float best_estimate = bestNode->cost + bestNode->heuristic;
      if (cost_estimate < best_estimate) {
        priorities[q] = cost_estimate;
      } 
    }
  }
}

