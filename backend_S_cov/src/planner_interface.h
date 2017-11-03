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

#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include <vector>

#include "common.h"

extern "C" {

Node computeBehaviorSequence(const PlannerParameters &, 
                                         const std::vector<Obstacle> &);
std::vector<SwarmState> executeBehaviorSchedule(const SimulatorParameters &, float* );

}

#endif // PLANNER_INTERFACE_H
