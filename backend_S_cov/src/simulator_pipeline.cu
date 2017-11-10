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

#include "common.h"
#include "cub/cub.cuh"

#include "behavior.cuh"
#include "robot_model.cuh"
#include "utility.cuh"

__global__
void simulatorStage0(SwarmState * state, BehaviorManager * manager, int behaviorId)
{
  int i = threadIdx.x;
  Behavior * behavior = manager->getBehavior(behaviorId);
  BehaviorContext * ctx = behavior->getContext(0);
  SwarmState * poses = &(ctx->poses);
  poses->x[i] = state->x[i];
  poses->y[i] = state->y[i];
  poses->theta[i] = state->theta[i];
}

__global__
void simulatorStage1(BehaviorManager * manager, int behaviorId)
{
  int i = blockIdx.x;
  int j = threadIdx.x;
  Behavior * behavior = manager->getBehavior(behaviorId);
  BehaviorContext * ctx = behavior->getContext(0);
  behavior->stage1(ctx, i, j);
}

__global__
void simulatorStage2(Node * nodeIn, BehaviorManager * manager, int behaviorId, 
  SwarmState * state, MapLimits * mapLimits)
{
  int i = threadIdx.x;
  Behavior * behavior = manager->getBehavior(behaviorId);
  BehaviorContext * ctx = behavior->getContext(0);
  behavior->stage2(ctx, i);
  applyRobotModel(ctx, i);
  SwarmState * poses = &(ctx->poses);
  state->x[i] = poses->x[i];
  state->y[i] = poses->y[i];
  state->theta[i] = poses->theta[i];
  if (outsideMapLimits(ctx, i, mapLimits)) {
    nodeIn->valid = false;
  }
} 

__global__
void simulatorStage2_1(Node * nodeIn, BehaviorManager * manager, int behaviorId, 
  SwarmState * state, Obstacle * obstacles)
{
  int o = blockIdx.x;
  int i = threadIdx.x;
  Behavior * behavior = manager->getBehavior(behaviorId);
  BehaviorContext * ctx = behavior->getContext(0);
  if (robotIntersectsObstacle(ctx, obstacles+o, i)) {
    nodeIn->valid = false;
  }
}

__global__
void simulatorStage3(BehaviorManager * manager, int behaviorId, float* dCost)
{
  int i = threadIdx.x;
  Behavior * behavior = manager->getBehavior(behaviorId);
  BehaviorContext * ctx = behavior->getContext(0);

  typedef cub::BlockReduce<float, ROBOTS> BlockReduce;
  __shared__ typename BlockReduce::TempStorage storage;
  float travelled = ctx->travelled[i];
  float tmp_cost = BlockReduce(storage).Sum(travelled);
  __syncthreads();

  ctx->travelled[i] = 0.0f;
  if(i==0)
  {
    dCost[0] += tmp_cost;
    printf("behavior ID is %d, (tmp_cost, cost) is (%f, %f)\n", behaviorId, tmp_cost, dCost[0]);
  } 



} 

__global__
void simulatorStage4(Node * nodeIn, BehaviorManager * manager, Target * target, int behaviorId, float robotRadius)
{
  int i = threadIdx.x;
  Behavior * behavior = manager->getBehavior(behaviorId);
  BehaviorContext * ctx = behavior->getContext(0);
  SwarmState * poses = &(ctx->poses);


  float x = poses->x[i];
  float y = poses->y[i];

  float dx = target->x - x;
  float dy = target->y - y;
  float distance = std::sqrt(dx*dx + dy*dy) - (target->radius - robotRadius);
  if (distance < 0.0f) { distance = 0.0f; }
  if(distance != 0){
    nodeIn->complete = false;
  }
  
  if(i == 0)
  {
    memcpy(&nodeIn->seen_map.seen[0][0], &ctx->seen_map.seen[0][0], sizeof(int)*MAPSIZE*MAPSIZE*4);
    nodeIn->seen_map.seen_count = ctx->seen_map.seen_count;
    int mapsize = nodeIn->seen_map.mapsize;
    int total_cells = mapsize * mapsize * 4;
    nodeIn->coverage_ratio = (float) nodeIn->seen_map.seen_count / total_cells;
  }
  


}