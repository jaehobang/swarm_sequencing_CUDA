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

#ifndef COMMON_H
#define COMMON_H

const int ROBOTS = 10;
const int BEHAVIORS = 8;
const int MAX_SIMULTANEOUS_EXPANSIONS = 1024;

const int MAX_SEQUENCE_LENGTH = 12;

const float GAIN_V = 4.0f;
const float GAIN_W = 4.0f;

const float ROBOT_RADIUS = 0.5f;

constexpr float HEURISTIC_WEIGHT = 1.0f;
static_assert(HEURISTIC_WEIGHT >= 1.0f, "Invalid weight");

const float TIME_STEP_DT = 0.1f;

constexpr float MAX_MISSION_TIME = 60.0f;
const int MAX_DURATION = MAX_MISSION_TIME / TIME_STEP_DT;

typedef struct SwarmState {
  float x[ROBOTS];
  float y[ROBOTS];
  float theta[ROBOTS];
} SwarmState;

enum BehaviorId {
  BEHAVIOR_RENDEZVOUS = 0,
  BEHAVIOR_ANTIRENDEZVOUS,
  
  BEHAVIOR_FLOCK_EAST,
  BEHAVIOR_FLOCK_NORTH,
  
  BEHAVIOR_FLOCK_WEST,
  BEHAVIOR_FLOCK_SOUTH,
  BEHAVIOR_LINE_X,
  BEHAVIOR_LINE_Y,
  BEHAVIOR_FINAL_ID
};
static_assert(BEHAVIOR_FINAL_ID == BEHAVIORS, "Wrong number of BehaviorId");

typedef struct BehaviorSequence {
  int length;
  int ids[MAX_SEQUENCE_LENGTH];
} BehaviorSequence;

// Note: Durations are given as number of time steps
// of size TIME_STEP_DT (hence integer)
typedef struct DurationSequence {
  int length;
  int times[MAX_SEQUENCE_LENGTH];
} DurationSequence;

// Note: Node.t is given as number of time steps 
// of size TIME_STEP_DT (hence integer)
typedef struct Node {
  int t;
  float cost;
  float heuristic;
  BehaviorSequence sequence;
  SwarmState poses;
  bool valid;
} Node;

typedef struct BehaviorContext {
  SwarmState poses;
  float control_v[ROBOTS];
  float control_w[ROBOTS];
  float travelled[ROBOTS];
  float target_distance[ROBOTS];
} BehaviorContext;

typedef struct MapLimits {
  float minX;
  float minY;
  float maxX;
  float maxY;
} MapLimits;

typedef struct Target {
  float x;
  float y;
  float radius;
} Target;

typedef struct Obstacle {
  float x;
  float y;
  float radius;
} Obstacle;

typedef struct PlannerParameters {
  float robotRadius;
  Target target;
  MapLimits mapLimits;
  SwarmState initial;
  DurationSequence durations;
} PlannerParameters;

typedef struct SimulatorParameters {
  SwarmState initial;
  DurationSequence durations;
  BehaviorSequence behaviors;
} SimulatorParameters;

#endif // COMMON_H
