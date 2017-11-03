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

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "common.h"

#include "behavior.cuh"

class Simulator {
 public:
  Simulator(const SimulatorParameters & parameters_)
  : parameters(parameters_)
  , behaviorManager(1)
  {}
 
  std::vector<SwarmState> simulate(float*);
  
  SimulatorParameters parameters;
  
 private: 

  thrust::device_vector<BehaviorManager> behaviorManager; 
  thrust::device_vector<SwarmState> states;
};

#endif // SIMULATOR_H 

