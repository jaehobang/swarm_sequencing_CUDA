
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
#include <string>
#include <ctime>

#include <cassert>

#include "common.h"
#include "planner_interface.h"
#include "smha_headers.hpp"

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>


using namespace std;

char *behavior_array[DIR] = { "rendezvous", "antirendezvous", "flock_east", "flock_north", "flock_west", "flock_south",  "line_x", "line_y" };

std::vector<string> behavior_array_display = { "Rendezvous", "Antirendezvous", "Flock East", "Flock North", "Flock West", "Flock South", "Line X", "Line Y" };




void initialize_parameters(PARAM* param, std::vector<float> time_array, std::vector<int>sequence_array, std::vector<uint8_t> fix_array)
{ 
  param->time_array_count = time_array.size();
  param->sequence_array_count = sequence_array.size();
  printf("Inside initialize_params....\n");
  printf("Length of time_array is %d, Length of sequence is %d\n", time_array.size(), sequence_array.size());

  for(int i = 0; i < time_array.size(); i++)
  {
    param->time_array[i] = time_array[i];
  }
  for(int i = 0; i < sequence_array.size(); i++)
  {
    param->sequence_array[i] = sequence_array[i];
  }
  /*
  for(int i = 0; i < param->N; i++)
  {
    printf("robot %d pos = %f %f %f\n", i, param->robot_pos[i][0], param->robot_pos[i][1],
            param->robot_pos[i][2]);
  }
  for(int i = 0; i < param->M; i++)
  {
    printf("obstacle %d pos = %f %f %f\n", i, param->obstacle_pos[i][0], param->obstacle_pos[i][1],
          param->obstacle_pos[i][2]);
  }
  */
  printf("Returning from init params\n");
  return;
}

void convertParam2PlanParam(PARAM* param, PlannerParameters* planParams, int is_aided)
{

  printf("Inside convertParam2PlanParam\n");
  constexpr float mission_time = 60.0f;
  //constexpr int sequence_length = param->time_array_count;
  static_assert(mission_time <= MAX_MISSION_TIME, "Mission too long");
  //static_assert(sequence_length <= MAX_SEQUENCE_LENGTH, "Too many durations");


  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0, 1);

  planParams->robotRadius = param->robot_radius;
  printf("ROBOT radius is %f\n", planParams->robotRadius);
  planParams->target.x = param->target_center[0];
  planParams->target.y = param->target_center[1];
  planParams->target.radius = param->target_radius;
  planParams->mapLimits.minX = -param->mapsize;
  planParams->mapLimits.maxX = param->mapsize;
  planParams->mapLimits.minY = -param->mapsize;
  planParams->mapLimits.maxY = param->mapsize;
  for (int i=0; i<ROBOTS; i++) {
    planParams->initial.x[i] = param->robot_pos[i][0];
    planParams->initial.y[i] = param->robot_pos[i][1];
    planParams->initial.theta[i] = param->robot_pos[i][2];
  }
  if(is_aided == 0 || is_aided == 1)
  {
    planParams->durations.length = param->sequence_array_count;
  }
  else if(is_aided == 2)
  {
    planParams->durations.length = param->time_array_count;
  }
  for (int i=0; i<planParams->durations.length; i++) {
    if ( i == 0) {
      planParams->durations.times[i] = 
      (int) (param->time_array[i] / param->dt);
    } else {
      planParams->durations.times[i] = 
      (int) ((param->time_array[i] - param->time_array[i-1]) / param->dt);
    } 
  }
}


void generateReturn(RETURN* return_1, SimulatorParameters* simParams, Node* best_node, std::vector<SwarmState> trajectory)
{
  /* std::vector<POS> robot_positions; - done
    std::vector<int> sequence_end_indices; - done
    float cost_of_path; - need - done
    uint8_t is_valid_path; -need - done //will always be valid from now on
    uint8_t is_complete; - need //this is something to be discussed
    std::vector<std::string> sequence_string_array; -done
  */
  for(int i = 0; i < trajectory.size(); i++)
  {
    POS pos;
    for(int j = 0; j < ROBOTS; j++)
    {     
      pos.robot_pos[j][0] = trajectory[i].x[j];
      pos.robot_pos[j][1] = trajectory[i].y[j];
      pos.robot_pos[j][2] = trajectory[i].theta[j]; 
    }
    return_1->robot_positions.push_back(pos);
  }

  int count = 0;
  printf("simParams duration.length is %d, behaviors.length is %d\n", simParams->durations.length, simParams->behaviors.length);
  for(int i = 0; i < simParams->durations.length; i++)
  {
    count += simParams->durations.times[i];
    return_1->sequence_end_indices.push_back(count);
  }

  for(int i = 0; i < simParams->behaviors.length; i++)
  {
    return_1->sequence_string_array.push_back(behavior_array_display[simParams->behaviors.ids[i]]);
  }

  return_1->cost_of_path = best_node->cost;
  return_1->is_valid_path = best_node->valid;
  return_1->is_complete = best_node->complete; //For now, valid and complete are the same things
  return_1->is_optimal = best_node->optimal;

  /*DEBUGGING Printing everything instide the return struct */
  printf("robot_positions size is %d\n", return_1->robot_positions.size());
  printf("end indices are....\n");
  std::copy(return_1->sequence_end_indices.begin(), return_1->sequence_end_indices.end(), std::ostream_iterator<int>(std::cout, " "));
  //std::copy_n(&return_1->sequence_end_indices, return_1->sequence_end_indices.size(), std::ostream_iterator<int>(std::cout, " ")); std::cout << std::endl;
  printf("sequence string array is ....\n");
  std::copy(return_1->sequence_string_array.begin(), return_1->sequence_string_array.end(), std::ostream_iterator<std::string>(std::cout, " "));
  //std::copy_n(return_1->sequence_string_array, return_1->sequence_string_array.size(), std::ostream_iterator<std::string>(std::cout, " ")); std::cout << std::endl;
  printf("cost of path is %f\n", return_1->cost_of_path);
  printf("is valid path is %d\n", return_1->is_valid_path);
  printf("is complete %d\n", return_1->is_complete);
  printf("is optimal %d\n", return_1->is_optimal);

  return;

}


void updatePlanParams(PARAM* param, PlannerParameters* planParams, SimulatorParameters* simParams, std::vector<SwarmState> trajectory)
{
  //TODO:
  planParams->initial = trajectory[trajectory.size() - 1];
  planParams->durations.length = param->time_array_count - param->sequence_array_count;
  for (int i=0; i<planParams->durations.length; i++) {
    planParams->durations.times[i] = 
    (int) ((param->time_array[i + param->sequence_array_count] - 
            param->time_array[i + param->sequence_array_count - 1]) / param->dt);
  }
  return;
}


void updateSimParams(SimulatorParameters* newSimParams, SimulatorParameters* oldSimParams)
{
//TODO:
  int newduration_size = newSimParams->durations.length + oldSimParams->durations.length;
  int newbehavior_size = newSimParams->behaviors.length + oldSimParams->behaviors.length;
  for(int i = oldSimParams->durations.length; i < newduration_size; i++)
  {
    oldSimParams->durations.times[i] = newSimParams->durations.times[i - oldSimParams->durations.length];
    oldSimParams->behaviors.ids[i] = newSimParams->behaviors.ids[i - oldSimParams->durations.length];
     
  }
  memcpy(&newSimParams->durations.times[0], &oldSimParams->durations.times[0], sizeof(int)*newduration_size);
  memcpy(&newSimParams->behaviors.ids[0], &oldSimParams->behaviors.ids[0], sizeof(int)*newbehavior_size);

  newSimParams->durations.length = newduration_size;
  newSimParams->behaviors.length = newbehavior_size;

  printf("new duration size %d, new behavior size %d\n", newduration_size, newbehavior_size);
  

  return;
}




RETURN testmain(PARAM* param, int is_aided, std::vector<float> time_array, std::vector<int> sequence_array, std::vector<uint8_t> isFixed)
{

  printf("Starting\n");

  clock_t start;
  double duration;

  start = std::clock();

  RETURN return_1;

  initialize_parameters(param, time_array, sequence_array, isFixed);

  PlannerParameters planParams;

  convertParam2PlanParam(param, &planParams, is_aided);
  
  std::cout << "PlannerParameters initialized" << std::endl;
  std::copy_n(planParams.initial.x, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
  std::copy_n(planParams.initial.y, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
  std::copy_n(planParams.initial.theta, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
  std::copy_n(planParams.durations.times, planParams.durations.length, std::ostream_iterator<int>(std::cout, " ")); std::cout << std::endl;
  std::copy_n(param->time_array, param->time_array_count, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;

  std::vector<Obstacle> obstacles;
  std::vector<Obstacle_n> obstacles_n;
  

  for (int i=0; i<param->C; i++) {
    Obstacle obstacle;
    obstacle.x = param->obstacle_pos[i][0];
    obstacle.y = param->obstacle_pos[i][1];
    obstacle.radius = param->obstacle_pos[i][2];
    obstacles.push_back(obstacle);
  }

  for (int i=0; i< param->S; i++) {
    Obstacle_n obstacle_n;
    obstacle_n.x = param->obstacle_narrow_pos[i][0];
    obstacle_n.y = param->obstacle_narrow_pos[i][1];
    obstacle_n.x_scale = param->obstacle_narrow_pos[i][2];
    obstacle_n.y_scale = param->obstacle_narrow_pos[i][3];
    obstacles_n.push_back(obstacle_n);
  }


  

  if(is_aided == 0) { //unaided
    
    SimulatorParameters simParams;
    simParams.initial = planParams.initial;
    simParams.durations = planParams.durations;
    

    BehaviorSequence bs_tmp;
    bs_tmp.length = param->sequence_array_count;
    memcpy(&simParams.behaviors.ids[0], &param->sequence_array[0], sizeof(int)*param->sequence_array_count);
    simParams.behaviors.length = param->sequence_array_count;


    printf("behaviors.length = %d, durations.length = %d\n", simParams.behaviors.length, simParams.durations.length);

    float cost = 0;
    Node best_node;
    best_node.complete = true;
    best_node.optimal = true;
    best_node.valid = true;
    std::vector<SwarmState> trajectory = executeBehaviorSchedule(simParams, &cost, planParams, obstacles, obstacles_n, &best_node);
    best_node.cost = cost;

    generateReturn(&return_1, &simParams, &best_node, trajectory);
    
  }


  else if(is_aided == 1) //aided and unaided combined
  {
    /* UNAIDED PORTION */
    SimulatorParameters simParams_phase1;
    simParams_phase1.initial = planParams.initial;
    simParams_phase1.durations = planParams.durations;
    


    memcpy(&simParams_phase1.behaviors.ids[0],&param->sequence_array[0], sizeof(int)*param->sequence_array_count);
    simParams_phase1.behaviors.length = param->sequence_array_count;

    float cost_phase1 = 0;

    Node best_node1;
    best_node1.valid = true;
    best_node1.optimal = true;
    best_node1.complete = true;
    std::vector<SwarmState> trajectory_phase1 = executeBehaviorSchedule(simParams_phase1, &cost_phase1, 
                                          planParams, obstacles, obstacles_n, &best_node1);
    best_node1.cost = cost_phase1;


    //generateReturn(&return_1, &simParams, &best_node, trajectory);

    if(best_node1.valid)
    {
      if(param->sequence_array_count != param->time_array_count && best_node1.complete == false)
      {
        /* AIDED PORTION */
        /* NEED TO UPDATE planParams.initial/durations, behavior length etc */
        updatePlanParams(param, &planParams, &simParams_phase1, trajectory_phase1);



        Node best_node = computeBehaviorSequence(planParams, obstacles, obstacles_n);
        std::cout << "Computed behavior sequence: ";
        std::copy_n(best_node.sequence.ids, best_node.sequence.length, std::ostream_iterator<int>(std::cout, " ")); 
        std::cout << std::endl;
        
        assert(best_node.sequence.length <= planParams.durations.length);
        SimulatorParameters simParams_phase2;
        simParams_phase2.initial = planParams.initial;
        simParams_phase2.durations = planParams.durations;
        simParams_phase2.behaviors = best_node.sequence;

       
        simParams_phase2.durations.length = simParams_phase2.behaviors.length;
        float cost_phase2 = 0;

        Node best_node2;
        best_node2.valid = true;
        best_node2.optimal = false;
        best_node2.complete = true;
        std::vector<SwarmState> trajectory_phase2 = executeBehaviorSchedule(simParams_phase2, &cost_phase2, planParams, obstacles, obstacles_n, &best_node2);

        /* Combine the results from two phases before going into generateReturn */
        best_node.cost += cost_phase1;
        //best_node.valid &= best_node1.valid;
        //best_node.optimal &= best_node1.optimal;
        //best_node.complete &= best_node1.complete;
        if(best_node2.valid == false)
        {
          printf("SOMETHING IS VERY WRONG!!!!!!!!! simulator returned invalid sequence!!!!!\n");
        }

        trajectory_phase1.insert(trajectory_phase1.end(), trajectory_phase2.begin() + 1, trajectory_phase2.end());

        updateSimParams(&simParams_phase2, &simParams_phase1);
        generateReturn(&return_1, &simParams_phase2, &best_node, trajectory_phase1);
        /*
        for (int i=0; i<trajectory.size(); i++) {
          std::cout << "Trajectory[" << i << "]: " << std::endl;
          std::copy_n(trajectory[i].x, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
          std::copy_n(trajectory[i].y, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
          std::copy_n(trajectory[i].theta, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
        }
        */
      }
      else{ //there is no need to use the system to generate sequence
        generateReturn(&return_1, &simParams_phase1, &best_node1, trajectory_phase1);
      }
    
    }
    else{


      generateReturn(&return_1, &simParams_phase1, &best_node1, trajectory_phase1);
    }
    
  }

  else if(is_aided == 2) //only aided
  {
    printf("Computing Behavior Sequence...\n");
    Node best_node = computeBehaviorSequence(planParams, obstacles, obstacles_n);
    std::cout << "Computed behavior sequence: ";
    std::copy_n(best_node.sequence.ids, best_node.sequence.length, std::ostream_iterator<int>(std::cout, " ")); 
    std::cout << std::endl;
    
    assert(best_node.sequence.length <= planParams.durations.length);
    SimulatorParameters simParams;
    simParams.initial = planParams.initial;
    simParams.durations = planParams.durations;
    simParams.behaviors = best_node.sequence;

   
    simParams.durations.length = simParams.behaviors.length;
    float cost = 0;

    Node best_node1;
    std::vector<SwarmState> trajectory = executeBehaviorSchedule(simParams, &cost, planParams, obstacles, obstacles_n, &best_node1);
    best_node1.cost = cost;
    
    generateReturn(&return_1, &simParams, &best_node, trajectory);
    /*
    for (int i=0; i<trajectory.size(); i++) {
      std::cout << "Trajectory[" << i << "]: " << std::endl;
      std::copy_n(trajectory[i].x, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
      std::copy_n(trajectory[i].y, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
      std::copy_n(trajectory[i].theta, ROBOTS, std::ostream_iterator<float>(std::cout, " ")); std::cout << std::endl;
    }
    */
  }

  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

  std::cout<<"printf: "<< duration <<'\n';
  printf("return_1 complete, optimal %d %d\n", return_1.is_complete, return_1.is_optimal);


  
    

 
  return return_1;
}
