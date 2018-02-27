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

#include <iostream>
#include <limits>
#include <ctime>

#include <cassert>

#include <thrust/complex.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/functional.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/transform.h>

#include "common.h"

#include "planner.cuh"
#include "planner_pipeline.cuh"

Node Planner::makePlan()
{
  assert(parameters.durations.length <= MAX_SEQUENCE_LENGTH);
  assert(parameters.robotRadius <= parameters.target.radius);
  
  // Sanitize parameters
  int totalDuration = 0;
  for (int i=0; i<parameters.durations.length; i++) {
    if (totalDuration + parameters.durations.times[i] > MAX_DURATION) {
      parameters.durations.times[i] = MAX_DURATION - totalDuration;
    }
    totalDuration += parameters.durations.times[i];
  }
  for (int i=0; i<ROBOTS; i++) {
    parameters.initial.theta[i] = wrapToPi(parameters.initial.theta[i]);
  }

  Node node;
  node.t = 0;
  node.cost = 0.0f;
  node.heuristic = 0.0f;
  node.sequence.length = 0;
  node.poses = parameters.initial;
  node.valid = true;
  node.complete = false;
  node.optimal = false;




  // FIXME: Do parallel reduce  
  for (int i=0; i<ROBOTS; i++) {
    float dx = parameters.target.x - node.poses.x[i];
    float dy = parameters.target.y - node.poses.y[i];
    float dr = parameters.target.radius - parameters.robotRadius;
    float distance = std::sqrt(dx*dx + dy*dy) - dr;
    node.heuristic += std::max(distance, 0.0f);
  }
  node.heuristic *= HEURISTIC_WEIGHT;
  std::cout << "Initial cost=" << node.cost 
            << ", heuristic=" << node.heuristic << std::endl;



  Node node1;
  memcpy(&node1, &node, sizeof(Node));
  node1.heuristic = HEURISTIC_WEIGHT * 100000;

  // Copy to device and execute pipeline
  dParameters.assign(1, parameters);
  dObstacles = obstacles;
  dObstacles_n = obstacles_n;
  priorities.assign(1, node.cost + node.heuristic);
  nodes.assign(1, node);
  node.cost = std::numeric_limits<float>::infinity();
  node.heuristic = 0.0f; 
  best.assign(1, node);
  best_attempt.assign(1, node1);

  Node result = executePipeline();
  return result;
}

struct compare_durations {
  DurationSequence * durations;
  
  compare_durations(DurationSequence * durations_)
  : durations(durations_)
  {}
  
  __host__ __device__
  bool operator()(const Node & lhs, const Node & rhs) {
    int a = (lhs.sequence.length < durations->length) 
          ? durations->times[lhs.sequence.length] 
          : 0;
    int b = (rhs.sequence.length < durations->length) 
          ? durations->times[rhs.sequence.length] 
          : 0;
    return a < b;
  }
};

struct compare_node {
  compare_node()
  {}

  // Node quality comparison:  
  // (1) valid nodes less than invalid nodes
  // (2) lower heuristic (will be 0 for feasible sequences)
  // (3) lower cost from start
  __host__ __device__
  bool operator()(const Node & lhs, const Node & rhs) {
    bool c1  = (lhs.valid && rhs.valid);
    bool c2  = (lhs.heuristic == rhs.heuristic); //reached destination
    bool r12 = (lhs.cost < rhs.cost);
    bool r1  = (lhs.heuristic < rhs.heuristic);
    bool r   = lhs.valid;
    return (c1 ? (c2 ? r12 : r1) : r);
  }
};

template<typename T>
struct not_finite 
{
  __host__ __device__
  T operator()(const T & x) const {
    return !isfinite(x);
  }
};

struct node_completed
{
  DurationSequence * durations;
 
  node_completed(DurationSequence * durations_)
  : durations(durations_)
  {}
  
  __host__ __device__
  bool operator()(const Node & n) const {
    return n.sequence.length == durations->length; 
  }
};

struct gte_value
{
  const float v;

  gte_value(float v_) 
  : v(v_)
  {}

  __host__ __device__
  bool operator()(const float f) const {
    return v <= f;
  }
};

Node Planner::executePipeline()
{
  // XXX: These are raw device pointers, never dereference them!
  static_assert(sizeof(char) == 1, "Wrong size char");
  PlannerParameters * pParameters = thrust::raw_pointer_cast(dParameters.data());
  DurationSequence * pDurations = reinterpret_cast<DurationSequence *>(
    reinterpret_cast<char *>(pParameters) + offsetof(PlannerParameters, durations));
  MapLimits * pMapLimits = reinterpret_cast<MapLimits *>(
    reinterpret_cast<char *>(pParameters) + offsetof(PlannerParameters, mapLimits));
  Target * pTarget = reinterpret_cast<Target *>(
    reinterpret_cast<char *>(pParameters) + offsetof(PlannerParameters, target));
  BehaviorManager * pBehaviorManager = thrust::raw_pointer_cast(behaviorManager.data());
  Node * pBestNode = thrust::raw_pointer_cast(best.data());
  Obstacle * pObstacles = thrust::raw_pointer_cast(dObstacles.data());
  Obstacle_n * pObstacles_n = thrust::raw_pointer_cast(dObstacles_n.data());
  

  clock_t start = clock();
  while (!nodes.empty()) {
    clock_t end = clock();
    float time_elapsed = float(end - start);

    if (time_elapsed > 20000000) //10 sec
    {
      printf("Exceeded time limit of %f (ms)", time_elapsed /1e6);
      Node bestNode = best[0];
      Node bestAttemptNode = best_attempt[0];
      if(bestNode.complete) return bestNode;
      else return bestAttemptNode;

    }


    Node bestNode = best[0];
    float bestPriority = bestNode.cost + bestNode.heuristic;
    if (bestPriority <= priorities[0]) { 
      bestNode.optimal = 1;
      printf("Found the optimal node!!\n");
      return bestNode;
    }





    int oldNumNodes = nodes.size();
    int nodesToExpand = std::min(oldNumNodes, MAX_SIMULTANEOUS_EXPANSIONS);
    priorities.resize(oldNumNodes+nodesToExpand*BEHAVIORS, std::numeric_limits<float>::infinity());
    nodes.resize(oldNumNodes+nodesToExpand*BEHAVIORS, Node());
    float * pPriorities = thrust::raw_pointer_cast(priorities.data());
    float * pPrioritiesExpanded = pPriorities + oldNumNodes;
    Node * pNodes = thrust::raw_pointer_cast(nodes.data());
    Node * pNodesExpanded = pNodes + oldNumNodes;
    
    //std::cout << "Priorities: ";  
    //thrust::copy(priorities.begin(), priorities.end(), std::ostream_iterator<float>(std::cout, " "));
    //std::cout << std::endl;

    // StageA: Setup nodes for expansion
    plannerStageA1<<<nodesToExpand, BEHAVIORS>>>(pNodes, pNodesExpanded, pDurations);
    //std::cout << "StageA1 complete" << std::endl;

    dim3 stageA2Grid(nodesToExpand, BEHAVIORS);
    plannerStageA2<<<stageA2Grid, ROBOTS>>>(pNodes, pNodesExpanded, pBehaviorManager);
    //std::cout << "StageA2 complete" << std::endl;
 
    Node maxDurationNode = *thrust::max_element(nodes.begin(), nodes.begin() + nodesToExpand, 
      compare_durations(pDurations));
    int d = maxDurationNode.sequence.length;
    // std::cout << "maxDurationNode.sequence.length = " << d << std::endl;
    assert(d < parameters.durations.length);
    int max_duration = parameters.durations.times[d];
    //std::cout << "Max duration identified " << max_duration << std::endl;
   
    for (int t=0; t<max_duration; t++) {
      // StageB: Forward integrate behavior dynamics
      dim3 stageB1Grid(nodesToExpand, BEHAVIORS, ROBOTS);
      plannerStageB1<<<stageB1Grid, ROBOTS>>>(pNodes, pNodesExpanded, pBehaviorManager, pDurations);
      dim3 stageB2Grid(nodesToExpand, BEHAVIORS);
      plannerStageB2<<<stageB2Grid, ROBOTS>>>(pNodes, pNodesExpanded, pBehaviorManager, pDurations, pMapLimits);
      
      // StageC: Check collisions
      dim3 stageC1Grid(nodesToExpand, BEHAVIORS, obstacles.size());
      plannerStageC1<<<stageC1Grid, ROBOTS>>>(pNodes, pNodesExpanded, pBehaviorManager, pDurations, pObstacles);
      dim3 stageC1_5Grid(nodesToExpand, BEHAVIORS, obstacles_n.size());
      plannerStageC1_5<<<stargeC1_5Grid, ROBOTS>>>(pNodes, pNodesExpanded, pBehaviorManager, pDurations, pObstacles);
      plannerStageC2<<<nodesToExpand, BEHAVIORS>>>(pNodes, pNodesExpanded, pDurations);
    }
    //std::cout << "Beginning StageD" << std::endl;

    // StageD: Compute metrics such as cost and heuristic
    dim3 stageD1Grid(nodesToExpand, BEHAVIORS);
    plannerStageD1<<<stageD1Grid, ROBOTS>>>(pNodes, pNodesExpanded, pBehaviorManager, pTarget, 
      parameters.robotRadius, pPrioritiesExpanded, pBestNode);  
    //std::cout << "StageD1 complete" << std::endl;
    //std::cout << "Priorities: ";  
    //thrust::copy(priorities.begin(), priorities.end(), std::ostream_iterator<float>(std::cout, " "));
    //std::cout << std::endl;
    
    // Remove invalid nodes 
    nodes.erase(thrust::remove_if(nodes.begin() + oldNumNodes, nodes.end(), 
      priorities.begin() + oldNumNodes, not_finite<float>()), nodes.end());
    priorities.erase(thrust::remove_if(priorities.begin() + oldNumNodes, 
      priorities.end(), not_finite<float>()), priorities.end());
    //std::cout << "Invalid nodes erased" << std::endl;

    // All expanded nodes are now valid, so evaluate them
    compare_node cmpn;
    auto it = thrust::min_element(nodes.begin() + oldNumNodes, nodes.end(), cmpn);
    if (it != nodes.end()) {
      Node candidate = *it;

      //Node curr_best_attempt = best_attempt[0];
      if(cmpn(candidate, best_attempt[0]))
      {
        std::cout << "Updating best attempt!!" << std::endl;
        printf("best_attempt valid %d\n", candidate.valid);
        std::copy_n(candidate.sequence.ids, candidate.sequence.length, std::ostream_iterator<int>(std::cout, " ")); 
        best_attempt[0] = candidate;
      }
      
      
      //std::cout << "Candidate cost=" << candidate.cost 
      //          << ", heuristic=" << candidate.heuristic 
      //          << ", sequence=";
      //const BehaviorSequence & seq = candidate.sequence;
      //std::copy_n(seq.ids, seq.length, std::ostream_iterator<int>(std::cout, " ")); 
      //std::cout << std::endl;
      if (cmpn(candidate, best[0])) {
        candidate.complete = 1; 
        best[0] = candidate; 
        bestPriority = candidate.cost + candidate.heuristic;
        std::cout << "Updated best node" << std::endl;
      }
    }

    // Remove nodes that cannot be expanded further
    priorities.erase(thrust::remove_if(priorities.begin() + oldNumNodes, 
      priorities.end(), nodes.begin() + oldNumNodes, 
      node_completed(pDurations)), priorities.end());
    nodes.erase(thrust::remove_if(nodes.begin() + oldNumNodes, nodes.end(), 
      node_completed(pDurations)), nodes.end());
    //std::cout << "Erased nodes that cannot be expanded further" << std::endl;

    // Remove expanded elements
    nodes.erase(nodes.begin(), nodes.begin() + nodesToExpand);
    priorities.erase(priorities.begin(), priorities.begin() + nodesToExpand);
    //std::cout << "Expanded nodes erased" << std::endl;

    // Remove nodes that cannot be best
    nodes.erase(thrust::remove_if(nodes.begin(), nodes.end(), priorities.begin(), 
      gte_value(bestPriority)), nodes.end());
    priorities.erase(thrust::remove_if(priorities.begin(), priorities.end(), 
      gte_value(bestPriority)), priorities.end());
    //std::cout << "Removed nodes that cannot be best" << std::endl;
    //std::cout << "Number of nodes remaining: " << nodes.size() << std::endl;
    //std::cout << "Priorities: ";  
    //thrust::copy(priorities.begin(), priorities.end(), std::ostream_iterator<float>(std::cout, " "));
    //std::cout << std::endl;

    // Update queue
    assert(priorities.size() == nodes.size());
    thrust::device_vector<int> indices(nodes.size());
    thrust::sequence(indices.begin(), indices.end(), 0, 1);
    thrust::sort_by_key(priorities.begin(), priorities.end(), indices.begin());
    thrust::device_vector<Node> sortedNodes(nodes.size());
    thrust::gather(indices.begin(), indices.end(), nodes.begin(), sortedNodes.begin());
    nodes = sortedNodes;
    //std::cout << "Updated queue" << std::endl;
    //std::cout << "Priorities: ";  
    //thrust::copy(priorities.begin(), priorities.end(), std::ostream_iterator<float>(std::cout, " "));
    //std::cout << std::endl;
  }
  printf("Searched entire space!!\n");
  Node bestNode = best[0];
  Node bestAttemptNode = best_attempt[0];
  if(bestNode.complete) {

    bestNode.optimal = 1;
    printf("returning bestNode!\n");
    printf("cost %f, heuristic %f\n", bestNode.cost, bestNode.heuristic);
    return bestNode;
  }
  else {
    printf("returning best attempt\n");
    printf("bestattempt cost %f, heuristic %f\n", bestAttemptNode.cost, bestAttemptNode.heuristic);
    return bestAttemptNode;
  }
}
