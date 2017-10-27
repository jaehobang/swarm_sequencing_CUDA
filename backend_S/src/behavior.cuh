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

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "common.h"

#include "flock_biased.cuh"
#include "formation_control.cuh"
#include "rendezvous.cuh"

typedef void (*Stage1BehaviorFunction)(BehaviorContext *, int, int);
typedef void (*Stage2BehaviorFunction)(BehaviorContext *, int);

class Behavior {
 public:
 __device__
  Behavior()
  {}
 
  __device__
  Behavior(Stage1BehaviorFunction stage1_, 
           Stage2BehaviorFunction stage2_, 
           void * contexts_,
           size_t contextSize_)
  : stage1(stage1_)
  , stage2(stage2_)
  , contexts(contexts_)
  , contextSize(contextSize_)
  {}
  
  __host__ __device__
  BehaviorContext * getContext(int index) const {
    static_assert(sizeof(char) == 1, "Wrong size char?");
    char * ctx = static_cast<char *>(contexts);
    ctx += contextSize*index;
    return reinterpret_cast<BehaviorContext *>(ctx);
  }  

  // Do NOT call any of these from __host__ code
  Stage1BehaviorFunction stage1;
  Stage2BehaviorFunction stage2;
  
 private:
  void * contexts;
  size_t contextSize;
};

class BehaviorManager {
 public:
  __device__
  BehaviorManager()
  {
    behaviors[BEHAVIOR_RENDEZVOUS] = 
      Behavior(behavior_rendezvous_stage1, 
               behavior_rendezvous_stage2,
               &(behaviorRendezvousContexts[0]),
               sizeof(BehaviorRendezvousContext));
    behaviors[BEHAVIOR_ANTIRENDEZVOUS] = 
      Behavior(behavior_rendezvous_stage1, 
               behavior_rendezvous_stage2,
               &(behaviorAntiRendezvousContexts[0]),
               sizeof(BehaviorAntiRendezvousContext));    
    behaviors[BEHAVIOR_FLOCK_EAST] = 
      Behavior(behavior_flock_biased_stage1, 
               behavior_flock_biased_stage2,
               &(behaviorFlockEastContexts[0]),
               sizeof(BehaviorFlockEastContext));
    behaviors[BEHAVIOR_FLOCK_NORTH] = 
      Behavior(behavior_flock_biased_stage1, 
               behavior_flock_biased_stage2,
               &(behaviorFlockNorthContexts[0]),
               sizeof(BehaviorFlockNorthContext));
    behaviors[BEHAVIOR_FLOCK_WEST] = 
      Behavior(behavior_flock_biased_stage1, 
               behavior_flock_biased_stage2,
               &(behaviorFlockWestContexts[0]),
               sizeof(BehaviorFlockWestContext));
    behaviors[BEHAVIOR_FLOCK_SOUTH] = 
      Behavior(behavior_flock_biased_stage1, 
               behavior_flock_biased_stage2,
               &(behaviorFlockSouthContexts[0]),
               sizeof(BehaviorFlockSouthContext));
    behaviors[BEHAVIOR_LINE_X] = 
      Behavior(behavior_formation_control_stage1,
               behavior_formation_control_stage2,
               &(behaviorLineXContexts[0]),
               sizeof(BehaviorLineXContext));
    behaviors[BEHAVIOR_LINE_Y] = 
      Behavior(behavior_formation_control_stage1,
               behavior_formation_control_stage2,
               &(behaviorLineYContexts[0]),
               sizeof(BehaviorLineYContext));
  }
  
  __host__ __device__
  Behavior * getBehavior(int index) {
    return &(behaviors[index]);
  } 
    
 private:
  Behavior behaviors[BEHAVIORS];
  BehaviorRendezvousContext behaviorRendezvousContexts[MAX_SIMULTANEOUS_EXPANSIONS];
  BehaviorAntiRendezvousContext behaviorAntiRendezvousContexts[MAX_SIMULTANEOUS_EXPANSIONS];
  
  BehaviorFlockEastContext behaviorFlockEastContexts[MAX_SIMULTANEOUS_EXPANSIONS];
  BehaviorFlockNorthContext behaviorFlockNorthContexts[MAX_SIMULTANEOUS_EXPANSIONS];
  
  BehaviorFlockWestContext behaviorFlockWestContexts[MAX_SIMULTANEOUS_EXPANSIONS];
  BehaviorFlockSouthContext behaviorFlockSouthContexts[MAX_SIMULTANEOUS_EXPANSIONS];
  BehaviorLineXContext behaviorLineXContexts[MAX_SIMULTANEOUS_EXPANSIONS];
  BehaviorLineYContext behaviorLineYContexts[MAX_SIMULTANEOUS_EXPANSIONS];
};

#endif // BEHAVIOR_H
