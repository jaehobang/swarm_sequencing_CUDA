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

#include <algorithm>

#include <thrust/execution_policy.h>

#include "common.h"

#include "simulator_pipeline.cuh"
#include "simulator.cuh"

std::vector<SwarmState>
Simulator::simulate(float* costp, Node* best_node)
{
  const BehaviorSequence & behaviors = parameters.behaviors;
  const DurationSequence & durations = parameters.durations;
  assert(durations.length == behaviors.length);

  

  int totalDuration = std::accumulate(&(durations.times[0]), 
    &(durations.times[durations.length]), 0);
  states.assign(totalDuration+1, parameters.initial);
  BehaviorManager * pBehaviorManager = thrust::raw_pointer_cast(behaviorManager.data());
  SwarmState * pStates = thrust::raw_pointer_cast(states.data());
  

  dParameters.assign(1, planner_parameters);
  dObstacles = obstacles;
  Obstacle * pObstacles = thrust::raw_pointer_cast(dObstacles.data());
  dObstacles_n = obstacles_n;
  Obstacle_n * pObstacles_n = thrust::raw_pointer_cast(dObstacles_n.data());
  PlannerParameters * pParameters = thrust::raw_pointer_cast(dParameters.data());
  MapLimits * pMapLimits = reinterpret_cast<MapLimits *>(
    reinterpret_cast<char *>(pParameters) + offsetof(PlannerParameters, mapLimits));
  Target * pTarget = reinterpret_cast<Target *>(
    reinterpret_cast<char *>(pParameters) + offsetof(PlannerParameters, target));




  float* dCost;
  cudaMalloc(&dCost, sizeof(float));
  cudaMemcpy(dCost, costp, sizeof(float), cudaMemcpyHostToDevice);

  Node* nodeIn;
  cudaMalloc(&nodeIn, sizeof(Node));
  cudaMemcpy(nodeIn, best_node, sizeof(Node), cudaMemcpyHostToDevice);

  int t = 0;
  for (int d=0; d<durations.length; d++) {
    int behaviorId = parameters.behaviors.ids[d];
    //std::cout << "behaviorId=" << behaviorId << std::endl;
    for (int i=0; i<durations.times[d]; i++, t++) {
      //std::cout << "t=" << t << std::endl;
      SwarmState * pStateIn = pStates + t;
      SwarmState * pStateOut = pStateIn + 1;
      simulatorStage0<<<1, ROBOTS>>>(pStateIn, pBehaviorManager, behaviorId);
      simulatorStage1<<<ROBOTS, ROBOTS>>>(pBehaviorManager, behaviorId);
      simulatorStage2<<<1, ROBOTS>>>(nodeIn, pBehaviorManager, behaviorId, pStateOut, pMapLimits);
      dim3 stage2_1Grid(obstacles.size(), 1, 1);
      simulatorStage2_1<<<stage2_1Grid, ROBOTS>>>(nodeIn, pBehaviorManager, behaviorId, pStateOut, pObstacles);
      dim3 stage2_2Grid(obstacles_n.size(), 1, 1);
      simulatorStage2_2<<<stage2_2Grid, ROBOTS>>>(nodeIn, pBehaviorManager, behaviorId, pStateOut, pObstacles_n);
      //std::copy_n(states[t+1].x, ROBOTS , std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
      //std::copy_n(states[t+1].y, ROBOTS , std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
      //std::copy_n(states[t+1].theta, ROBOTS , std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
    }
    simulatorStage3<<<1,ROBOTS>>>(pBehaviorManager, behaviorId, dCost);
  }
  float robotRadius = planner_parameters.robotRadius;
  int behaviorId = parameters.behaviors.ids[durations.length - 1];
  simulatorStage4<<<1,ROBOTS>>>(nodeIn, pBehaviorManager, pTarget, behaviorId, robotRadius);

  cudaMemcpy(costp, dCost, sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(best_node, nodeIn, sizeof(Node), cudaMemcpyDeviceToHost);
  
  std::vector<SwarmState> trajectory(states.size());
  thrust::copy(states.begin(), states.end(), trajectory.begin());
  return trajectory;
}

