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

#ifndef PLANNER_H
#define PLANNER_H

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "common.h"

#include "behavior.cuh"

class Planner {
 public:
  Planner(const PlannerParameters & parameters_, const std::vector<Obstacle> & obstacles_, const std::vector<Obstacle_n> & obstacles_n_)
  : parameters(parameters_), obstacles(obstacles_), obstacles_n(obstacles_n_), behaviorManager(1)
  {}
  
  Node makePlan();
  
  PlannerParameters parameters;
  thrust::host_vector<Obstacle> obstacles;
  thrust::host_vecotr<Obstacle_n> obstacles_n;
  
private:
  Node executePipeline();

  thrust::device_vector<PlannerParameters> dParameters;
  thrust::device_vector<Obstacle> dObstacles;
  thrust::device_vector<Obstacle_n> dObstacles_n;
  
  thrust::device_vector<BehaviorManager> behaviorManager;
  
  thrust::device_vector<float> priorities;
  thrust::device_vector<Node> nodes;  
  thrust::device_vector<Node> best;
  thrust::device_vector<Node> best_attempt;
};

#endif // PLANNER_H

