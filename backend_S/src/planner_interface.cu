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

#include <vector>

#include "common.h"

#include "planner.cuh"
#include "simulator.cuh"

#include "planner_interface.h"

extern "C" {

Node computeBehaviorSequence(
  const PlannerParameters & parameters, 
  const std::vector<Obstacle> & obstacles,
  const std::vector<Obstacle_n> & obstacles_n)
{
  std::cout << "Inside compute Behavior Sequence" << std::endl;
  Planner planner(parameters, obstacles, obstacles_n);
  std::cout << "Planner initialized" << std::endl;
  Node node = planner.makePlan();
  //return (node.valid) ? node.sequence : BehaviorSequence();
  return node;
}

std::vector<SwarmState> executeBehaviorSchedule(
  const SimulatorParameters & parameters, float* costp, 
  const PlannerParameters & planner_parameters, const std::vector<Obstacle> & obstacles,
  const std::vector<Obstacle_n> & obstacles_n, 
  Node* best_node)
{
  Simulator simulator(parameters, planner_parameters, obstacles, obstacles_n);
  return simulator.simulate(costp, best_node);
}

}
