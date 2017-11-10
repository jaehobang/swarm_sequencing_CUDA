/* Implement SMHA - this one works!!!! */
/* 9/8/2017 making modifications for linux version */


#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda.h"

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <iterator>
#include <fstream>
#include <sstream>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <vector>
#include <set>
#include <algorithm>
#include <map>

#include "math.h"
#include "math_constants.h"
#include "smha_headers.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace std;

char *behavior_array[DIR] = { "rendezvous", "antirendezvous", "flock_east", "flock_north", "flock_west", "flock_south", "line_x", "line_y" };

std::vector<string> behavior_array_display = {"Rendezvous", "Antirendezvous", "Flock East", "Flock North", "Flock West", "Flock South", "Line X", "Line Y"};

/* Error Checking..... */
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////


/* DEVICE FUNCTIONS (BEHAVIOR and KERNEL) */

__device__
float d_wrapToPi(float input)
{
	float result = fmodf(input + CUDART_PI_F, 2 * CUDART_PI_F);
	result += (result < 0) * 2 * CUDART_PI_F;
	result -= CUDART_PI_F;
	return result;
}

__forceinline__ __device__
void updateSeenMap(node* ctx, int i)
{

  int grid_x = (int) rintf(ctx->robot_pos[i][0] + MAPSIZE);
  int grid_y = (int) rintf(ctx->robot_pos[i][1] + MAPSIZE);
  grid_x = fmaxf(grid_x, 0);
  grid_x = fminf(grid_x, MAPSIZE * 2 - 1);
  grid_y = fmaxf(grid_y, 0);
  grid_y = fminf(grid_y, MAPSIZE * 2 - 1);
  if(ctx->seen_map.seen[grid_y][grid_x] == 0)
  {
  	printf("Seen update (%f, %f), (%d, %d)\n", 
  		ctx->robot_pos[i][1], ctx->robot_pos[i][0], 
  		(int) rintf(ctx->robot_pos[i][1]), (int) rintf(ctx->robot_pos[i][0]));
    ctx->seen_map.seen[grid_y][grid_x] = 1;
    atomicAdd(&ctx->seen_map.seen_count, 1);
  }
  return;
}


__device__
void d_robot_model_i(node* current, int i, float u_v, float u_w, float dt)
{
	float max_linear_velocity = 8; //Doubled max linear, angular velocity by 2
	float max_angular_velocity = CUDART_PI_F / 8;
	u_v = fmaxf(-max_linear_velocity, fminf(max_linear_velocity, u_v));
	u_w = fmaxf(-max_angular_velocity, fminf(max_angular_velocity, u_w));
	current->robot_pos[i][0] = current->robot_pos[i][0] + u_v*cosf(current->robot_pos[i][2])*dt;
	current->robot_pos[i][1] = current->robot_pos[i][1] + u_v*sinf(current->robot_pos[i][2])*dt;
	current->robot_pos[i][2] = d_wrapToPi(current->robot_pos[i][2] + u_w*dt);
	updateSeenMap(current, i);
	return;
}


__device__
void d_formation_control(node* future, node* current, int i, float formation[][2], float dt)
{
	float connectivity_radius = 20.0f;
	float gain_v = 4.0f;
	float gain_w = 4.0f;
	float position_i[2], position_j[2];
	float heading_i, heading_j;
	float v[2], d[2], dv[2], b[2];
	float w, n, dtheta;
	float norm_d;
	float u_v, u_w;

	int N = current->N;
	position_i[0] = current->robot_pos[i][0];
	position_i[1] = current->robot_pos[i][1];
	heading_i = current->robot_pos[i][2];

	v[0] = 0;
	v[1] = 0;
	w = 0;
	n = 0;

	for (int j = 0; j < N; j++)
	{
		if(i == j) continue;
		position_j[0] = current->robot_pos[j][0];
		position_j[1] = current->robot_pos[j][1];
		heading_j = current->robot_pos[j][2];

		d[0] = position_j[0] - position_i[0];
		d[1] = position_j[1] - position_i[1];
		norm_d = sqrtf(powf(d[0], 2) + powf(d[1], 2));
		dtheta = 0;

		if (norm_d < connectivity_radius)
		{
			dv[0] = d[0] - (formation[j][0] - formation[i][0]);
			dv[1] = d[1] - (formation[j][1] - formation[i][1]);
			n += 1;
		}
		else{
			dv[0] = 0;
			dv[1] = 0;
		}
		v[0] += dv[0];
		v[1] += dv[1];

	}
	v[0] /= (n + 1);
	v[1] /= (n + 1);

	float tmp = atan2f(v[1], v[0]);
	tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
	dtheta = tmp - heading_i;

	tmp = atan2f(sinf(dtheta), cosf(dtheta));
	tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
	w = w + tmp;

	b[0] = cosf(heading_i);
	b[1] = sinf(heading_i);
	u_v = gain_v * (v[0] * b[0] + v[1] * b[1]);
	u_w = gain_w * w;

	d_robot_model_i(future, i, u_v, u_w, dt);

}



__device__
void d_line_x(node* future, node *current, int i, float dt)
{
	float formation[ROBOT_MAX][2];
	for(int i = 0; i < ROBOT_MAX; i++)
	{
		formation[i][0] = 2*(i + 1);
		formation[i][1] = 0;
	}

	d_formation_control(future, current, i, formation, dt);
}

__device__
void d_line_y(node* future, node *current, int i, float dt)
{
	float formation[ROBOT_MAX][2];
	for(int i = 0; i < ROBOT_MAX; i++)
	{
		formation[i][0] = 0;
		formation[i][1] = 2 * (i + 1);
	} 

	d_formation_control(future, current, i, formation, dt);
}



__device__
void d_antirendezvous(node* future, node *current, int i, float dt)
{
	//printf("Inside rendezvous\n");
	float connectivity_radius = 20;
	float gain_v = 4;
	float gain_w = 4;
	int N = current->N;
	float v[2], dv[2], b[2];
	float w, n;
	float position_i[2], position_j[2], d[2];
	float heading_i;
	float dtheta;
	float u_v, u_w;
	position_i[0] = current->robot_pos[i][0];
	position_i[1] = current->robot_pos[i][1];
	heading_i = current->robot_pos[i][2];
	v[0] = 0;
	v[1] = 0;
	w = 0;
	n = 0;
	for (int j = 0; j<N; j++)
	{
		if (i == j) continue;
		position_j[0] = current->robot_pos[j][0];
		position_j[1] = current->robot_pos[j][1];
		//heading_j = current->robot_pos[j][2];
		d[0] = position_j[0] - position_i[0];
		d[1] = position_j[1] - position_i[1];
		float norm_d = sqrtf(powf(d[0], 2) + powf(d[1], 2));
		if (norm_d < connectivity_radius)
		{
			dv[0] = -d[0];
			dv[1] = -d[1];
			n = n + 1;
		}
		else
		{
			dv[0] = 0;
			dv[1] = 0;
		}
		v[0] = v[0] + dv[0];
		v[1] = v[1] + dv[1];
	}
	v[0] = v[0] / (n + 1);
	v[1] = v[1] / (n + 1);
	float tmp = atan2f(v[1], v[0]);
	tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
	dtheta = tmp - heading_i;
	w = atan2f(sinf(dtheta), cosf(dtheta));
	w += 2 * CUDART_PI_F * (w == -CUDART_PI_F);
	b[0] = cosf(heading_i);
	b[1] = sinf(heading_i);
	u_v = gain_v * (v[0] * b[0] + v[1] * b[1]);
	u_w = gain_w * w;
	d_robot_model_i(future, i, u_v, u_w, dt);
	return;
}


__device__
void d_rendezvous(node* future, node *current, int i, float dt)
{
	//printf("Inside rendezvous\n");
	float connectivity_radius = 20;
	float gain_v = 4;
	float gain_w = 4;
	int N = current->N;
	float v[2], dv[2], b[2];
	float w, n;
	float position_i[2], position_j[2], d[2];
	float heading_i;
	float dtheta;
	float u_v, u_w;
	position_i[0] = current->robot_pos[i][0];
	position_i[1] = current->robot_pos[i][1];
	heading_i = current->robot_pos[i][2];
	v[0] = 0;
	v[1] = 0;
	w = 0;
	n = 0;
	for (int j = 0; j<N; j++)
	{
		if (i == j) continue;
		position_j[0] = current->robot_pos[j][0];
		position_j[1] = current->robot_pos[j][1];
		//heading_j = current->robot_pos[j][2];
		d[0] = position_j[0] - position_i[0];
		d[1] = position_j[1] - position_i[1];
		float norm_d = sqrtf(powf(d[0], 2) + powf(d[1], 2));
		if (norm_d < connectivity_radius)
		{
			dv[0] = d[0];
			dv[1] = d[1];
			n = n + 1;
		}
		else
		{
			dv[0] = 0;
			dv[1] = 0;
		}
		v[0] = v[0] + dv[0];
		v[1] = v[1] + dv[1];
	}
	v[0] = v[0] / (n + 1);
	v[1] = v[1] / (n + 1);
	float tmp = atan2f(v[1], v[0]);
	tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
	dtheta = tmp - heading_i;
	w = atan2f(sinf(dtheta), cosf(dtheta));
	w += 2 * CUDART_PI_F * (w == -CUDART_PI_F);
	b[0] = cosf(heading_i);
	b[1] = sinf(heading_i);
	u_v = gain_v * (v[0] * b[0] + v[1] * b[1]);
	u_w = gain_w * w;
	d_robot_model_i(future, i, u_v, u_w, dt);
	return;
}

__device__
void d_flock_biased(node* future, node *current, int i, float* bias, float dt)
{
	float min_linear_velocity = 1;
	float repulsion_radius = 5;
	float alignment_radius = 10;
	float attraction_radius = 20;
	float gain_v = 4;
	float gain_w = 4;
	float position_i[2], position_j[2];
	float heading_i, heading_j;
	float v[2], d[2], dv[2], b[2];
	float w, n, dtheta;
	float norm_d;
	float u_v, u_w;
	int N = current->N;
	position_i[0] = current->robot_pos[i][0];
	position_i[1] = current->robot_pos[i][1];
	heading_i = current->robot_pos[i][2];
	v[0] = 0;
	v[1] = 0;
	w = 0;
	n = 0;

	for (int j = 0; j < N; j++)
	{
		if (i == j) continue;
		position_j[0] = current->robot_pos[j][0];
		position_j[1] = current->robot_pos[j][1];
		heading_j = current->robot_pos[j][2];
		n = n + 1;
		d[0] = position_j[0] - position_i[0];
		d[1] = position_j[1] - position_i[1];
		norm_d = sqrtf(powf(d[0], 2) + powf(d[1], 2));
		dtheta = 0;
		if (norm_d < repulsion_radius)
		{
			dv[0] = -d[0] / powf(norm_d, 2);
			dv[1] = -d[1] / powf(norm_d, 2);
			float tmp = atan2f(dv[1], dv[0]);
			tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
			dtheta = tmp - heading_i;
		}
		else if (norm_d < alignment_radius)
		{
			dv[0] = 0;
			dv[1] = 0;
			dtheta = heading_j - heading_i;
		}
		else if (norm_d < attraction_radius)
		{
			dv[0] = d[0];
			dv[1] = d[1];
			float tmp = atan2f(dv[1], dv[0]);
			tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
			dtheta = tmp - heading_i;
		}
		else
		{
			dv[0] = 0;
			dv[1] = 0;
			dtheta = 0;
			n = n - 1;
		}
		v[0] += dv[0];
		v[1] += dv[1];
		float tmp = atan2f(sinf(dtheta), cosf(dtheta));
		tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
		w = w + tmp;
	}
	v[0] = v[0] / (n + 1) + bias[0]; //incorporated Sasanka's line 51 and 54 at once
	v[1] = v[1] / (n + 1) + bias[1];
	w = d_wrapToPi(w / (n + 1));

	float tmp = atan2f(bias[1], bias[0]);
	tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
	dtheta = tmp - heading_i;

	tmp = atan2f(sinf(dtheta), cosf(dtheta));
	tmp += 2 * CUDART_PI_F * (tmp == -CUDART_PI_F);
	w = w + tmp;

	b[0] = cosf(heading_i);
	b[1] = sinf(heading_i);
	u_v = gain_v * (v[0] * b[0] + v[1] * b[1]);
	u_w = gain_w * w;
	u_v = fmaxf(min_linear_velocity, u_v);

	d_robot_model_i(future, i, u_v, u_w, dt);
	return;
}


__device__
void d_flocking(node* future, node* current, int i, float dt)
{
	float bias[2] = { 0, 0 };
	d_flock_biased(future, current, i, bias, dt);
	return;
}


__device__
void d_flock_east(node* future, node* current, int i, float dt)
{
	float direction[2] = { 1, 0 };
	float gain = 1.0;
	float bias[2];
	bias[0] = gain * direction[0];
	bias[1] = gain * direction[1];
	d_flock_biased(future, current, i, bias, dt);
	return;
}


__device__
void d_flock_north(node* future, node* current, int i, float dt)
{
	float direction[2] = { 0, 1 };
	float gain = 1.0;
	float bias[2];
	bias[0] = gain * direction[0];
	bias[1] = gain * direction[1];
	d_flock_biased(future, current, i, bias, dt);
	return;
}

__device__
void d_flock_west(node* future, node* current, int i, float dt)
{
	float direction[2] = { -1, 0 };
	float gain = 1.0;
	float bias[2];
	bias[0] = gain * direction[0];
	bias[1] = gain * direction[1];
	d_flock_biased(future, current, i, bias, dt);
	return;
}

__device__
void d_flock_south(node* future, node* current, int i, float dt)
{
	float direction[2] = { 0, -1 };
	float gain = 1.0;
	float bias[2];
	bias[0] = gain * direction[0];
	bias[1] = gain * direction[1];
	d_flock_biased(future, current, i, bias, dt);
	return;
}

__device__
void d_move_stop(node* future, node* current, int i, float dt)
{
	return;
}


typedef void(*op_func) (node*, node*, int, float);
__device__ op_func func[DIR] = { d_rendezvous, d_antirendezvous, d_flock_east, d_flock_north, d_flock_west, d_flock_south, d_line_x, d_line_y };
__device__ char *d_behavior_array[DIR] = { "rendezvous", "antirendezvous", "flock_east", "flock_north", "flock_west", "flock_south", "line_x", "line_y" };


/* returns@ 0 is target is not reached; else 1 */
__device__
int d_target_reached(node curr_node, PARAM* param)
{
	int N = curr_node.N;
	float robot_radius = param->robot_radius;
	float target_radius = param->target_radius;
	/* L2norm(robot_pos(i) - target_center) */
	float robot_x, robot_y;
	float target_x = param->target_center[0];
	float target_y = param->target_center[1];
	float distance;
	for (int i = 0; i < N; i++)
	{
		robot_x = curr_node.robot_pos[i][0];
		robot_y = curr_node.robot_pos[i][1];
		distance = sqrt(powf(robot_x - target_x, 2) + powf(robot_y - target_y, 2));
		if (distance >= target_radius - robot_radius) return 0;
	}
	return 1;
}


/* returns@ 0 is not valid; else 1 */
__device__
int d_valid_poses(node curr_node, PARAM* param)
{
	int i, j;
	int N = curr_node.N;
	int M = param->M;
	float robot_x, robot_y;
	float robot_radius = param->robot_radius;
	float mapsize = param->mapsize;
	float safety_bounds = 0;
	/* check if position is off the map */
	for (i = 0; i< N; i++)
	{
		robot_x = curr_node.robot_pos[i][0];
		robot_y = curr_node.robot_pos[i][1];
		if (robot_x > mapsize || robot_y > mapsize ||
			robot_x < -mapsize || robot_y < -mapsize) {
			return 0;
		}
	}
	/* check if collided with obstacle */
	float obs_x, obs_y, obs_r;
	float distance;
	for (i = 0; i < N; i++)
	{
		robot_x = curr_node.robot_pos[i][0];
		robot_y = curr_node.robot_pos[i][1];
		for (j = 0; j < M; j++)
		{
			obs_x = param->obstacle_pos[j][0];
			obs_y = param->obstacle_pos[j][1];
			obs_r = param->obstacle_pos[j][2];
			distance = sqrt(powf(robot_x - obs_x, 2) + powf(robot_y - obs_y, 2));
			if (distance < obs_r + robot_radius + safety_bounds) {
				return 0;
			}
		}
	}
	return 1;
}




__device__
float d_calculate_G(node d_expanded, node d_open, PARAM* d_param, int mode)
{


	float final_G = d_open.G;
  float N = d_open.N;
	for (int i = 0; i < N; i++)
	{
		final_G += sqrtf(powf(d_expanded.robot_pos[i][0] - d_open.robot_pos[i][0], 2) +
			powf(d_expanded.robot_pos[i][1] - d_open.robot_pos[i][1], 2));
	}
	return final_G;
}



/* calculate the heuristic between curr node and the destination */
/* cost_to_go is distance left: cost_to_go = sum(max(pdist2(poses_next(:,1:2),target_center') - (target_radius-robot_radius),0)); */
__device__
float d_calculate_H(node d_expanded, PARAM* d_param, int mode)
{
	float heuristic = (sqrtf(2) / 3) * 
          fmaxf(0, d_param->coverage_ratio * (MAPSIZE * MAPSIZE * 4) - d_expanded.seen_map.seen_count - 4 * ROBOT_MAX);
  return heuristic;

}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/* param @ d_expanded - result of expanded nodes
param @ d_open - input of nodes to expand
param @ d_param - parameters about the swarm
param @ dir - Number of directions (7 for now)
param @ iteration - used for backtracing */
__global__
void k_expandStates(node* d_expanded, node* d_open, PARAM* d_param, int dir, int iteration, int real_copies, int queue_i)
{
	int dir_index = threadIdx.x;
	int robot_index = threadIdx.y;
	int node_index = blockIdx.x;
	int index = node_index * dir + dir_index;

	/* Check thread boundary */
	if (dir_index >= dir)  return;
	if (robot_index >= d_param->N) return;
	if (node_index >= real_copies) return;

	/* Expand the nodes */
	float dt = d_param->dt;
	//float ti = d_param->ti;
	float dT = d_param->dT;
	//float tf = d_param->tf;
 
  if(d_open[node_index].sequence_numel >= d_param->time_array_count) {
    d_expanded[index].isEmpty = 1;
    return;
  }

  updateSeenMap(d_open, robot_index);

	int steps = (int)dT / dt;
	int currSequenceIndex = d_open[node_index].sequence_numel;

	if(currSequenceIndex == 0) steps = 
		(int) (d_param->time_array[currSequenceIndex] / dt);
	else steps = (int) ((d_param->time_array[currSequenceIndex] - d_param->time_array[currSequenceIndex - 1]) / dt);
	/* Only write to global memory once */
	if (robot_index == 0) d_expanded[index] = d_open[node_index];

	__syncthreads();

	for (int i = 0; i < steps; i++)
	{
		node d_expanded_old = d_expanded[index];
		func[dir_index](&d_expanded[index], &d_expanded_old, robot_index, dt);
		__syncthreads();
		/* Check swarm map boundary and obstacle collision to separate valid vs invalid */
		if (d_valid_poses(d_expanded[index], d_param) == 0) {
			d_expanded[index].isEmpty = 1;
			return;
		}
		d_expanded[index].G = d_calculate_G(d_expanded[index], d_expanded_old, d_param, queue_i);
	}

	if (robot_index == 0)
	{
		/* Only write to global memory once */
		d_expanded[index].behaviorIndices[currSequenceIndex] = dir_index;
		d_expanded[index].behaviorIdx = d_expanded[index].behaviorIdx * 10 + dir_index + 1;

		/* Check swarm if reached destination */
		//d_expanded[index].reached_destination = d_target_reached(d_expanded[index], d_param);

		/* cost_estimate is total cost: cost_estimate = cost_next + H*cost_to_go */
		d_expanded[index].F = d_expanded[index].G + (d_param->H) * d_calculate_H(d_expanded[index], d_param, queue_i);
		d_expanded[index].sequence_numel += 1;
		int denom = MAPSIZE * MAPSIZE * 4;
		d_expanded[index].coverage_ratio = (float) d_expanded[index].seen_map.seen_count / denom;
		if(d_expanded[index].coverage_ratio >= d_param->coverage_ratio) d_expanded[index].reached_destination = 1;
		//printf("INSIDE KERNEL>>> [%d].F = %f, .behaviorIdx = %llu\n", 
    //        index, d_expanded[index].F, d_expanded[index].behaviorIdx);
	}
	return;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////

/* HOST FUNCTIONS */

/* calculate the heuristic between curr node and the destination */
float h_calculate_H1(node d_expanded, PARAM* param, int N)
{
	float heuristic = (sqrtf(2) / 3) * 
          fmaxf(0, param->coverage_ratio * (MAPSIZE * MAPSIZE * 4) - d_expanded.seen_map.seen_count - 4 * ROBOT_MAX);
  return heuristic;
}



////////////////////////////////////////////////////////////////////////////////////////////

/* this function serves to convert position to grid index */
int convert_position(float mapsize, float robot_pos_x, float robot_pos_y)
{
	int col_offset = (int)(robot_pos_x + mapsize);
	int row_offset = (int)(mapsize - robot_pos_y);
	return row_offset * 2 * mapsize + col_offset;
}


std::vector<float> convert_index(float mapsize, int index)
{
	int row = index / (2 * mapsize);
	int col = index % (int)(2 * mapsize);
	std::vector<float> pos;
	pos.push_back(col - mapsize);
	pos.push_back(mapsize - row);

	return pos;
}


void print_distance_left(node best_node, PARAM* param)
{
	float average_x = 0;
	float average_y = 0;
	for (int i = 0; i < param->N; i++)
	{
		average_x += best_node.robot_pos[i][0];
		average_y += best_node.robot_pos[i][1];
	}
	average_x /= param->N;
	average_y /= param->N;
	float distance_left = sqrt(pow(param->target_center[0] - average_x, 2) + pow(param->target_center[1] - average_y, 2));
	printf("Distance left is %f\n", distance_left);
	return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////


Queue* initializeQps(PARAM* param, node h_start)
{
	Queue* qps = new Queue[QUEUE_SIZE];

	for (int i = 0; i < param->q_count; i++)
	{
		qps[i].iteration = 1;
	}



	for (int i = 0; i < param->q_count; i++)
	{
    qps[i].h_open.push_back(h_start);
	}

	return qps;
}


float h_calculate_H(node d_expanded, PARAM* d_param, int mode)
{

	float heuristic = (sqrtf(2) / 3) * 
          fmaxf(0, d_param->coverage_ratio * (MAPSIZE * MAPSIZE * 4) - d_expanded.seen_map.seen_count - 4 * ROBOT_MAX);
  return heuristic;

}

void updateQueues(Queue* qps, node curr, PARAM* param, int queue_i)
{
	curr.F = curr.G + param->H * h_calculate_H(curr, param, 0);
	float queue_0_F = curr.F;
  //printf("UpdateQueues>>>> curr.F = %f, curr.behaviorIdx = %llu\n", curr.F, curr.behaviorIdx);

	qps[0].h_open.push_back(curr);
  int index = qps[0].h_open.size() - 1;
  //printf("Just to make sure UpdateQueus>>> curr.F = %f, curr.behaviorIdx = %llu\n", 
  //       qps[0].h_open[index].F, qps[0].h_open[index].behaviorIdx);
	return;
}


void expandStates(Queue* qps, PARAM* param, node* best_node_p, node* best_attempt, int queue_i)
{
	PARAM* d_param; /* device parameters */
	gpuErrchk( cudaMalloc(&d_param, sizeof(PARAM)) );
	gpuErrchk( cudaMemcpy(d_param, param, sizeof(PARAM), cudaMemcpyHostToDevice) );

	node *d_open; /* device open nodes */
	node *d_expanded; /* device expanded nodes */

					  /* Dequeue from h_open */
	int iteration = qps[queue_i].iteration;
	//printf("Inside ExpandStates....iteration is %d\n", iteration);

	int h_open_size = qps[queue_i].h_open.size();
	int real_copies = (h_open_size > ARRAY_SIZE) ? ARRAY_SIZE : h_open_size;
	node* h_open_array = new node[real_copies]; /* temporary data used for cudaMemcpy of h_open */
	node* h_expanded = new node[real_copies * DIR]; /* data used for retrieving expanded nodes data; Also used for backtracking */

	//printf("For %d, real_copies %d\n", queue_i, real_copies);

	std::copy(qps[queue_i].h_open.begin(), qps[queue_i].h_open.begin() + real_copies, h_open_array);
	/* Erase the same nodes from all queues */
	qps[queue_i].h_open.erase(qps[queue_i].h_open.begin(), qps[queue_i].h_open.begin() + real_copies);
	/* TODO: Make sure this portion of code is correct!!!! */

  //printf("Copying is done\n");
//////////////
	for (int queue_j = 0; queue_j < param->q_count; queue_j++)
	{
		if (queue_i == queue_j) continue;
	  //printf("queue_j h_open size is %d\n", qps[queue_j].h_open.size());
   	std::vector<int> erase_indices;
		for (int i = 0; i < real_copies; i++) {
			for (int j = 0; j < qps[queue_j].h_open.size(); j++)
			{
				if (h_open_array[i].behaviorIdx == qps[queue_j].h_open[j].behaviorIdx) {
          		
          			erase_indices.push_back(j);
					break;
				}
			}
		}

    	std::sort(erase_indices.begin(), erase_indices.end());
    	for (int k = erase_indices.size() - 1; k >= 0; k--)
		{
			qps[queue_j].h_open.erase(qps[queue_j].h_open.begin() + erase_indices[k]);
		}
		
	}
  
	/* Copy necessary data to device memory */
	gpuErrchk( cudaMalloc(&d_open, sizeof(node) * real_copies) );
	gpuErrchk( cudaMemcpy(d_open, h_open_array, sizeof(node) * real_copies, cudaMemcpyHostToDevice) );
	gpuErrchk( cudaMalloc(&d_expanded, sizeof(node) * real_copies * DIR) );


	/* Allocate proper blocks and threads */
	const dim3 blockSize(DIR, param->N, 1);
	const dim3 gridSize(real_copies, 1, 1);
	//const dim3 blockSize(real_copies, 1, 1);
	//const dim3 gridSize(dir, param->N, 1);

	/* Run the GPU code */
  	k_expandStates << < gridSize, blockSize >> >(d_expanded, d_open, d_param, DIR, iteration, real_copies, queue_i);
  gpuErrchk( cudaPeekAtLastError() );

  	/* Copy back from GPU to CPU */
	gpuErrchk( cudaMemcpy(h_expanded, d_expanded, sizeof(node) * real_copies * DIR, cudaMemcpyDeviceToHost) );

	/* Update open list with expanded nodes and update best_node */
	for (int ind = 0; ind < real_copies * DIR; ind++)
	{ 
	    /* Debuggin */
	   if (h_expanded[ind].isEmpty == 0 && h_expanded[ind].reached_destination == 0)
		 {
	      //printf("After Kernel>>>>> [%d].F = %f, .behaviorIdx = %llu\n", 
	      //      ind, h_expanded[ind].F, h_expanded[ind].behaviorIdx);
			
		
			node curr = h_expanded[ind];
			float distance_to_goal = curr.F - curr.G;
	  		if(distance_to_goal < best_attempt->F - best_attempt->G) memcpy(best_attempt, &curr, sizeof(node));
				updateQueues(qps, curr, param, queue_i);
			}
			else if (h_expanded[ind].isEmpty == 0 && h_expanded[ind].reached_destination == 1
				&& h_expanded[ind].G < best_node_p->G) {
				memcpy(best_node_p, &h_expanded[ind], sizeof(node));
				//printf("UPDATING BEST NODE!! cost is %f, sequence_numel is %d\n", best_node_p->G, best_node_p->sequence_numel);
				//printf("printing sequence...\n");
				for (int i = 0; i < best_node_p->sequence_numel; i++)
				{
					int index = best_node_p->behaviorIndices[i];
					//printf("hello world????\n");
				//	printf("%s, ", behavior_array[index]);
					//printf("%d ", index);
				}
				//printf("\n");
			}
	}


	/* Sort the open_list */
	for (int i = 0; i < QUEUE_SIZE; i++)
	{
		std::sort(qps[i].h_open.begin(), qps[i].h_open.end(), [](node left, node right) {return left.F < right.F; });
	}

	/* Prepare for next iteration */
	qps[queue_i].iteration++;
	delete(h_open_array);
	cudaFree(d_expanded);
	cudaFree(d_open);
	cudaFree(d_param);

}

__global__
void k_noSMHA(POS* d_poses, node* d_result, PARAM* d_param, int* d_sequence_end_indices)
{
	int robot_index = threadIdx.x;
	/* Error checking */
	if (robot_index >= d_param->N) return;
	
	if (robot_index == 0) {
		memcpy(d_poses[0].robot_pos, d_param->robot_pos, sizeof(float)*d_param->N * 3);
		printf("Inside k_noSMHA!!!!\n");
		printf("SEQUENCE NUMEL IS %d\n", d_result->sequence_numel);
	}
	__syncthreads();
	
	/* Expand the nodes */
	float dt = d_param->dt;
	float ti = d_param->ti;
	float dT = d_param->dT;
	//float tf = d_param->tf;
	int steps = (int)dT / dt;
	float dstart = ti;
	float dend = 0;

	int sequence_count = d_result->sequence_numel;

	updateSeenMap(d_result, robot_index);
	node d_local;
	memcpy(&d_local, d_result, sizeof(node));
	int d_poses_index = 0;

	for (int i = 0; i < sequence_count; i++)
	{
		dstart = ti + dstart + dend;
		dend = d_param->time_array[i];
		if(i == 0) steps = (int) (d_param->time_array[i] / dt);
		else steps = (int) ( (d_param->time_array[i] - d_param->time_array[i-1]) / dt);
		for (int j = 1; j <= steps; j++)
		{
			/* Forward kinematics -> Save it to POS* -> Update d_local */
			func[d_result->behaviorIndices[i]](d_result, &d_local, robot_index, dt);
			__syncthreads();
		
			if(d_valid_poses(*d_result, d_param) == 0) { d_result->isEmpty = 1;}
			d_result->G = d_calculate_G(*d_result, d_local, d_param, 0);
			if (robot_index == 0) {
				memcpy(d_poses[d_poses_index].robot_pos, d_result->robot_pos, 
				sizeof(float) * d_param->N * 3);
			}
			__syncthreads();
			memcpy(&d_local, d_result, sizeof(node));
			d_poses_index++;
		}
		d_sequence_end_indices[i] = d_poses_index - 1;
	}

  //d_result->reached_destination = d_target_reached(*d_result, d_param);
  d_result->F = d_result->G + d_param->H * d_calculate_H(*d_result, d_param, 0);
  int denom = MAPSIZE * MAPSIZE * 4;
  d_result->coverage_ratio = (float) d_result->seen_map.seen_count / denom;
  if(d_result->coverage_ratio >= d_param->coverage_ratio) d_result->reached_destination = 1;
  if(robot_index == 0)
  {
  	printf("\n");
  	for(int i = 0 ; i < MAPSIZE * 2; i++)
  	{
  		for(int j = 0; j < MAPSIZE * 2; j++)
  		{
  			if(d_result->seen_map.seen[i][j] == 0)
  			{
  				printf("*");
  			}
  			else{
  				printf("+");
  			}
  		}
  		printf("\n");
  	}
  	printf("seen_count %d, denom %d\n", d_result->seen_map.seen_count, denom);
  	printf("coverage_ratio is %f\n", d_result->coverage_ratio);
  }

  

	return;
}


void noSMHAstar(PARAM* param, RETURN* return_1, node* result_node)
{
	/* Only for debugging purposes need to make sure that all the info is delivered correctly */
	printf("Inside noSMHAstar.....\n");
	printf("number of switch times given.... %d\n", param->time_array_count);
	for(int i = 0; i < param->time_array_count; i++)
	{
		printf("%f ", param->time_array[i]);
	}
	printf("\n");

	printf("checking result node also.... sequence_numel is %d\n", result_node->sequence_numel);
	for(int i = 0; i < param->sequence_array_count; i++)
	{
		printf("%s %d", behavior_array[result_node->behaviorIndices[i]], result_node->behaviorIndices[i]);
	}
	printf("\n");



	PARAM* d_param; /* device parameters */
	gpuErrchk(cudaMalloc(&d_param, sizeof(PARAM)));
	gpuErrchk(cudaMemcpy(d_param, param, sizeof(PARAM), cudaMemcpyHostToDevice));

	node* d_result;
	gpuErrchk(cudaMalloc(&d_result, sizeof(node)));
	gpuErrchk(cudaMemcpy(d_result, result_node, sizeof(node), cudaMemcpyHostToDevice));


	int seq_n = result_node->sequence_numel;
    printf("param->tf = %f, param->dt = %f\n", param->tf, param->dt);
	int h_result_size = param->tf / param->dt + 1;
	printf("h_result_size is ... %d\n", h_result_size);
	POS* h_poses = new POS[h_result_size];
	POS* d_poses;
	gpuErrchk(cudaMalloc(&d_poses, sizeof(POS) * h_result_size));

	
	/* Allocate space on device for sequence_end_indices array, cost_of_path, is_valid_path */
	//float cost_of_path = 0;
	//int is_valid_path = 0;
	int* h_sequence_end_indices = new int[param->sequence_array_count];
	int* d_sequence_end_indices;
	gpuErrchk(cudaMalloc(&d_sequence_end_indices, sizeof(int) * (param->sequence_array_count)));



	const dim3 gridSize(1, 1, 1);
	const dim3 blockSize(param->N, 1, 1);
	k_noSMHA << <gridSize, blockSize >> >(d_poses, d_result, d_param, d_sequence_end_indices);
    gpuErrchk( cudaPeekAtLastError() );

	/* Copy back from GPU to CPU */
	cudaMemcpy(h_poses, d_poses, sizeof(POS) * h_result_size, cudaMemcpyDeviceToHost);
	cudaMemcpy(h_sequence_end_indices, d_sequence_end_indices, 
		   sizeof(int) * (param->sequence_array_count), cudaMemcpyDeviceToHost);


    node* h_result = new node[1];
	cudaMemcpy(h_result, d_result, sizeof(node), cudaMemcpyDeviceToHost);

	
	printf("returned from kernel...\n");
	printf("printing all necessary information to check that kernel returned correctly\n");
	printf("Cost of path(G) = %f\n", h_result->G);
    printf("h_result->isEmpty %d\n", h_result->isEmpty);
	if(h_result->isEmpty == 0) printf("PATH IS VALID\n");
 	else printf("PATH IS INVALID\n");

	/* Copy all the necessary information back to return struct */
  return_1->cost_of_path = h_result->G;
	if(h_result->isEmpty) return_1->is_valid_path = 0;
	else return_1->is_valid_path = 1;
	return_1->is_complete = h_result->reached_destination;
	return_1->is_optimal = h_result->optimal;
	return_1->coverage_ratio = h_result->coverage_ratio;
	return_1->seen_map = h_result->seen_map;

  printf("updating robot_positions...\n");
  printf("h_result_size is %d\n", h_result_size);
	for(int i = 0; i < h_result_size; i++)
	{
    //printf("[%d] x,y,r = (%f %f %f)\n",i, h_poses[i].robot_pos[0][0], h_poses[i].robot_pos[0][1],
				//	h_poses[i].robot_pos[0][2]); 
		return_1->robot_positions.push_back(h_poses[i]);
	}
  printf("updating sequence end indices and sequence string array\n");
  printf("seq_n is %d\n", seq_n);
	for(int i = 0; i < seq_n; i++)
	{
		return_1->sequence_end_indices.push_back(h_sequence_end_indices[i]);
		return_1->sequence_string_array.push_back(behavior_array_display[h_result->behaviorIndices[i]]);
	}

	printf("making sure that return struct has necessary info...\n");
	printf("return_1->cost_of_path = %f\n", return_1->cost_of_path);
	printf("return-1->is_valid_path = %d\n", (int) return_1->is_valid_path);
	printf("return_1->coverage_ratio = %f\n", return_1->coverage_ratio);
  for(int i = 0; i < seq_n; i++)
  {
    printf("return_1->sequence_end_indices[%d] = %d\n", 
            i, return_1->sequence_end_indices[i]);
  }

  memcpy(result_node, h_result, sizeof(node));
  delete h_result;
	return;
}


/* returns @ string of behavior sequence (for now) behavior__behavior__behavior__....behavior*/
node SMHAstar(PARAM* param, node h_start)
{
	clock_t start = clock();
	cout << "Starting IMHA star " << endl;

	Queue* qps = initializeQps(param, h_start);
	node result;
	result.isEmpty = 1;
	result.F = numeric_limits<float>::max();

	node* best_node_p = new node[1];
	best_node_p->reached_destination = 0;
	best_node_p->isEmpty = 1;
	best_node_p->F = numeric_limits<float>::max();
	best_node_p->G = numeric_limits<float>::max();
	best_node_p->N = param->N;

	node best_attempt; //criteria is distance to destination
	memcpy(&best_attempt, &h_start, sizeof(node));

	//convert double to float for time_arra
	while (!qps[0].h_open.empty())
	{
		/* Exit if exceeded the amount of time */
		clock_t end = clock();
		float time_elapsed = float(end - start);

		if (time_elapsed > 10000000) //50 sec
		{
			printf("Exceeded time limit of %f (ms)", time_elapsed);
			if(result.isEmpty && best_node_p->isEmpty) 
			memcpy(&result, &best_attempt, sizeof(node));
			else if(result.isEmpty)
			memcpy(&result, best_node_p, sizeof(node));

			return result;
		}
  		
		
        node* minKeys = &qps[0].h_open[0];
				

		
		int size = qps[0].h_open.size() > ARRAY_SIZE ? ARRAY_SIZE : qps[0].h_open.size();
		for(int i = 0; i < size; i++)
		{
			node minKey_0 = minKeys[i];
			if (best_node_p->F <= minKey_0.F && minKey_0.coverage_ratio >= param->coverage_ratio)
			{
				printf("DONE WITH SEARCH!! NOW PRINTING RESULTS\n");
				printf("Current best node cost %f, sequence_numel is %d\n", best_node_p->G, best_node_p->sequence_numel);
				print_distance_left(*best_node_p, param);

				printf("printing sequence...\n");
				for (int i = 0; i < best_node_p->sequence_numel; i++)
				{
					int index = best_node_p->behaviorIndices[i];
					printf("%s, ", behavior_array[index]);
	 			}
				printf("\n");

				memcpy(&result, best_node_p, sizeof(node));
				clock_t end = clock();
				float time_elapsed = float(end - start);
				cout << "Time to calculate the route (ms): " << time_elapsed << endl;
				printf("inside smha!!! result sequence_numel!!!%d\n", result.sequence_numel);
				result.optimal = true;
				return result;
				
			}
		}

		expandStates(qps, param, best_node_p, &best_attempt, 0);
		
			

	}


	if (best_node_p->isEmpty == 0 && best_node_p->G < result.G) memcpy(&result, best_node_p, sizeof(node));
	

  if (result.isEmpty == 1 && best_node_p->isEmpty == 0) {
    memcpy(&result, best_node_p, sizeof(node));
    result.optimal = true;
  }

	if (result.isEmpty == 1 && best_node_p->isEmpty == 1) {
		printf("no route found, returning the best attempt\n");
		memcpy(&result, &best_attempt, sizeof(node));
	}

	else {
		printf("printing sequence...\n");
		for (int i = 0; i < result.sequence_numel; i++)
		{
			int index = result.behaviorIndices[i];
			printf("%s, ", behavior_array[index]);
		}
		printf("\n");
	}


	clock_t end = clock();
	float time_elapsed = float(end - start);
	cout << "Time to calculate the route (ms): " << time_elapsed << endl;
	return result;


}


__global__
void k_SAVE(POS* d_poses, node* d_result, node* d_start, PARAM* d_param)
{
	int robot_index = threadIdx.x;
	/* Error checking */
	if (robot_index >= d_param->N) return;

	if (robot_index == 0) memcpy(d_poses[0].robot_pos, d_param->robot_pos, sizeof(float)*d_param->N * 3);

	__syncthreads();

	/* Expand the nodes */
	float dt = d_param->dt;
	//float ti = d_param->ti;
	float dT = d_param->dT;
	//float tf = d_param->tf;
	int steps = (int)dT / dt;

	int sequence_count = d_result->sequence_numel;

	node d_local;
	memcpy(&d_local, d_start, sizeof(node));
	for (int i = 0; i < sequence_count; i++)
	{

		for (int j = 1; j <= steps; j++)
		{
			/* Forward kinematics -> Save it to POS* -> Update d_local */
			func[d_result->behaviorIndices[i]](d_start, &d_local, robot_index, dt);
			__syncthreads();
			if (robot_index == 0) memcpy(d_poses[i*steps + j].robot_pos, d_start->robot_pos, sizeof(float) * d_param->N * 3);
			__syncthreads();
			memcpy(&d_local, d_start, sizeof(node));
		}
	}


	return;

}

void SAVE_launch(node result_node, RETURN* return_1, PARAM* param)
{
	/* Open file and retrieve file id */

	printf("Printing out the sequence...\n");
	for(int i = 0; i < result_node.sequence_numel; i++)
	{
		printf("%s ", behavior_array[result_node.behaviorIndices[i]]);
	}
	printf("\n");

	PARAM* d_param; /* device parameters */
	gpuErrchk(cudaMalloc(&d_param, sizeof(PARAM)));
	gpuErrchk(cudaMemcpy(d_param, param, sizeof(PARAM), cudaMemcpyHostToDevice));

	node* d_result;
	gpuErrchk(cudaMalloc(&d_result, sizeof(node)));
	gpuErrchk(cudaMemcpy(d_result, &result_node, sizeof(node), cudaMemcpyHostToDevice));

	node h_start;
	h_start.isEmpty = 0;
	h_start.N = param->N;
	h_start.sequence_numel = 0;
	memcpy(&h_start.robot_pos, &param->robot_pos, sizeof(float)*ROBOT_MAX * 3);


	/* DEBUGGING!!!! */
	printf("making sure h_start has correct positions.....\n");
	for (int i = 0; i < param->N; i++)
	{
		printf("(%f %f %f) ", h_start.robot_pos[i][0], h_start.robot_pos[i][1], h_start.robot_pos[i][2]);
	}
	printf("\n");
	/* End of debugging.... */

	node* d_start;
	gpuErrchk(cudaMalloc(&d_start, sizeof(node)));
	gpuErrchk(cudaMemcpy(d_start, &h_start, sizeof(node), cudaMemcpyHostToDevice));


	int steps = (int)param->dT / param->dt;
	int seq_n = result_node.sequence_numel;
	int h_result_size = steps* seq_n + 1;
	printf("steps, seq_n, %d, %d\n", steps, seq_n);
	printf("h_result_size is ... %d\n", h_result_size);
	POS* h_poses = new POS[h_result_size];
	POS* d_poses;
	gpuErrchk(cudaMalloc(&d_poses, sizeof(POS) * h_result_size));




	const dim3 gridSize(1, 1, 1);
	const dim3 blockSize(param->N, 1, 1);
	k_SAVE << <gridSize, blockSize >> >(d_poses, d_result, d_start, d_param);
  gpuErrchk( cudaPeekAtLastError() );

	/* Copy back from GPU to CPU */
	cudaMemcpy(h_poses, d_poses, sizeof(POS) * h_result_size, cudaMemcpyDeviceToHost);

	/* Now save everything onto the txt */
	cout << "saving everything in txt file" << endl;


	//output_f = fopen(output_fname.str().c_str(), "w");
	int times = 0;
	printf("H_RESULT_SIZE IS %d\n", h_result_size);
	for (int i = 0; i < h_result_size; i++)
	{
		for (int j = 0; j < param->N; j++)
		{
			//fprintf(output_f, (std::to_string(h_poses[i].robot_pos[j][0]) + " " + std::to_string(h_poses[i].robot_pos[j][1])
			//	+ " " + std::to_string(h_poses[i].robot_pos[j][2]) + "\n").c_str());
			times++;
		}
		//fprintf(output_f, "\n");
	}
	return;
}

void SMHAstar_wrapper(PARAM* param, RETURN* result_1)
{
  printf("Inside smhastar_wrapper\n");
	node h_start; //starting node

	h_start.isEmpty = 0;
	h_start.N = param->N;
	h_start.sequence_numel = param->sequence_array_count;
  h_start.behaviorIdx = 0;
  for(int i = 0; i < h_start.sequence_numel; i++)
  {
    h_start.behaviorIndices[i] = param->sequence_array[i];
  }

	printf("Inside smhastar_wrapper, param->N = %d\n", param->N);

	std::copy(&param->robot_pos[0][0], &param->robot_pos[0][0] + param->N * 3, &h_start.robot_pos[0][0]);

	printf("Done copying robot_positions param->h_start\n");
	h_start.F = param->H * h_calculate_H1(h_start, param, h_start.N);
	h_start.G = 0;
	h_start.reached_destination = 0;

	h_start.seen_map.seen_count = 0;
	h_start.seen_map.mapsize = MAPSIZE;
	for(int i = 0; i < MAPSIZE*2; i++)
	{
		for(int j = 0; j < MAPSIZE*2; j++)
		{
			h_start.seen_map.seen[i][j] = 0;
		}
	}
	

  RETURN result_tmp;
  if(h_start.sequence_numel != 0){

    noSMHAstar(param, result_1, &h_start);
    if(param->time_array_count == param->sequence_array_count) return;
  }
////////////////////////////////////////////////////////////////////

  //Need to check that this works okay...
  printf("Printing h_start information insdie smha_star_wrapper\n");
  printf("isEmpty = %d, N = %d, F = %f, G = %f, behaviorIdx = %llu, reached_destination = %d, sequence_numel = %d\n", h_start.isEmpty,
          h_start.N, h_start.F, h_start.G, h_start.behaviorIdx,
          h_start.reached_destination, h_start.sequence_numel);
  for(int i = 0; i < h_start.sequence_numel; i++)
  {
    printf("%s ", behavior_array[h_start.behaviorIndices[i]]);
  }
  printf("\n");
  for(int i = 0; i < h_start.N; i++)
  {
    printf("robot[%d] x,y,r = %f %f %f\n", i, h_start.robot_pos[i][0],
			h_start.robot_pos[i][1], h_start.robot_pos[i][2]);
  }

  if(h_start.isEmpty == 1)
  { /*
    printf("H_start is empty!!!! making sure result_tmp has all the correct information....\n");
    for(int i = 0; i < result_tmp.sequence_end_indices.size(); i++)
    {
      printf("sequence[%d] end index is %d\n", i, result_tmp.sequence_end_indices[i]);
    }

    
    memcpy(result_1, &result_tmp, sizeof(RETURN));
		for(int i = 0; i < result_1->sequence_end_indices.size(); i++)
    {
      printf("sequence[%d] end index is %d\n", i, result_1->sequence_end_indices[i]);
    }
    */
    printf("RETURNING EARLY!!!\n");
    return;
  }  
  

	node result_node = SMHAstar(param, h_start); //note this result_node might be simply the closest attempt to the goal

  printf("\n\n\n\n\n\n\n");
  printf("After returning from SMHAstar...\n"); 
  printf("isEmpty = %d, N = %d, F = %f, G = %f, behaviorIdx = %llu, reached_destination = %d, sequence_numel = %d\n", result_node.isEmpty,
          result_node.N, result_node.F, result_node.G, result_node.behaviorIdx,
          result_node.reached_destination, result_node.sequence_numel);
  for(int i = 0; i < result_node.sequence_numel; i++)
  {
    printf("%s ", behavior_array[result_node.behaviorIndices[i]]);
  }
  printf("\n");
  for(int i = 0; i < result_node.N; i++)
  {
    printf("robot[%d] x,y,r = %f %f %f\n", i, result_node.robot_pos[i][0],
			result_node.robot_pos[i][1], result_node.robot_pos[i][2]);
  }
//////////////////////////////////////////////////////////////

	node h_final; //starting node

	h_final.isEmpty = 0;
	h_final.N = param->N;
	h_final.sequence_numel = result_node.sequence_numel;
  h_start.behaviorIdx = 0;
  for(int i = 0; i < h_final.sequence_numel; i++)
  {
    h_final.behaviorIndices[i] = result_node.behaviorIndices[i];
  }


	std::copy(&param->robot_pos[0][0], &param->robot_pos[0][0] + param->N * 3, &h_final.robot_pos[0][0]);

	printf("Done copying robot_positions param->h_start\n");
	h_final.F = param->H * h_calculate_H1(h_final, param, h_final.N);
	h_final.G = 0;
	h_final.reached_destination = 0;

	h_final.seen_map.seen_count = 0;
	h_final.seen_map.mapsize = MAPSIZE;
	for(int i = 0; i < MAPSIZE*2; i++)
	{
		for(int j = 0; j < MAPSIZE*2; j++)
		{
			h_final.seen_map.seen[i][j] = 0;
		}
	}	

	printf("checking the sequence before entering noSMHAstar...\n");
	printf("h_final sequence count is %d\n", h_final.sequence_numel);
	for(int i = 0; i < h_final.sequence_numel; i++)
	{
		printf("%s ", behavior_array_display[h_final.behaviorIndices[i]].c_str());

	}
	printf("\n");

  param->sequence_array_count = h_final.sequence_numel;
  for(int i = 0; i < h_final.sequence_numel; i++)
  {
    param->sequence_array[i] = h_final.behaviorIndices[i];
  }

  result_1->robot_positions.clear();
  result_1->sequence_end_indices.clear();
  result_1->sequence_string_array.clear();

	noSMHAstar(param, result_1, &h_final);
	return;



}




void onlySMHAstar_wrapper(PARAM* param, RETURN* result_1)
{
  printf("Inside smhastar_wrapper\n");
	node h_start; //starting node

	h_start.isEmpty = 0;
	h_start.N = param->N;
	h_start.sequence_numel = param->sequence_array_count;
	h_start.behaviorIdx = 0;
  for(int i = 0; i < h_start.sequence_numel; i++)
  {
    h_start.behaviorIndices[i] = param->sequence_array[i];
  }

	printf("Inside smhastar_wrapper, param->N = %d\n", param->N);

	std::copy(&param->robot_pos[0][0], &param->robot_pos[0][0] + param->N * 3, &h_start.robot_pos[0][0]);

	printf("Done copying robot_positions param->h_start\n");
	h_start.F = param->H * h_calculate_H1(h_start, param, h_start.N);
	h_start.G = 0;
	h_start.reached_destination = 0;

	h_start.seen_map.seen_count = 0;
	h_start.seen_map.mapsize = MAPSIZE;
	for(int i = 0; i < MAPSIZE*2; i++)
	{
		for(int j = 0; j < MAPSIZE*2; j++)
		{
			h_start.seen_map.seen[i][j] = 0;
		}
	}
	

	node result_node = SMHAstar(param, h_start); //note this result_node might be simply the closest attempt to the goal

  printf("\n\n\n\n\n\n\n");
  printf("After returning from SMHAstar...\n"); 
  printf("isEmpty = %d, N = %d, F = %f, G = %f, behaviorIdx = %llu, reached_destination = %d, sequence_numel = %d\n", result_node.isEmpty,
          result_node.N, result_node.F, result_node.G, result_node.behaviorIdx,
          result_node.reached_destination, result_node.sequence_numel);
  for(int i = 0; i < result_node.sequence_numel; i++)
  {
    printf("%s ", behavior_array[result_node.behaviorIndices[i]]);
  }
  printf("\n");
  for(int i = 0; i < result_node.N; i++)
  {
    printf("robot[%d] x,y,r = %f %f %f\n", i, result_node.robot_pos[i][0],
			result_node.robot_pos[i][1], result_node.robot_pos[i][2]);
  }
//////////////////////////////////////////////////////////////

	node h_final; //starting node

	h_final.isEmpty = 0;
	h_final.N = param->N;
	h_final.sequence_numel = result_node.sequence_numel;
  h_start.behaviorIdx = 0;
  for(int i = 0; i < h_final.sequence_numel; i++)
  {
    h_final.behaviorIndices[i] = result_node.behaviorIndices[i];
  }


	std::copy(&param->robot_pos[0][0], &param->robot_pos[0][0] + param->N * 3, &h_final.robot_pos[0][0]);

	printf("Done copying robot_positions param->h_start\n");
	h_final.F = param->H * h_calculate_H1(h_final, param, h_final.N);
	h_final.G = 0;
	h_final.reached_destination = 0;

	h_final.seen_map.seen_count = 0;
	h_final.seen_map.mapsize = MAPSIZE;
	for(int i = 0; i < MAPSIZE*2; i++)
	{
		for(int j = 0; j < MAPSIZE*2; j++)
		{
			h_final.seen_map.seen[i][j] = 0;
		}
	}	

	printf("checking the sequence before entering noSMHAstar...\n");
	printf("h_final sequence count is %d\n", h_final.sequence_numel);
	for(int i = 0; i < h_final.sequence_numel; i++)
	{
		printf("%s ", behavior_array_display[h_final.behaviorIndices[i]].c_str());

	}
	printf("\n");

  param->sequence_array_count = h_final.sequence_numel;
  for(int i = 0; i < h_final.sequence_numel; i++)
  {
    param->sequence_array[i] = h_final.behaviorIndices[i];
  }

  result_1->robot_positions.clear();
  result_1->sequence_end_indices.clear();
  result_1->sequence_string_array.clear();

	noSMHAstar(param, result_1, &h_final);
	return;

}





/////////////////////////////////////////////////////////////////////////////////////////////////////////

void initialize_parameters(PARAM* param, std::vector<float> time_array, 
	std::vector<int>sequence_array, std::vector<uint8_t> fix_array, float coverage_ratio)
{ 
	param->time_array_count = time_array.size();
  param->sequence_array_count = sequence_array.size();
  param->coverage_ratio = coverage_ratio;

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


void fix_robot_positions(PARAM* param)
{
	param->N = 10;
	param->robot_pos[0][0] = -18;
	param->robot_pos[0][1] = -10;
	param->robot_pos[0][2] = 0;

	param->robot_pos[1][0] = -17;
	param->robot_pos[1][1] = -10;
	param->robot_pos[1][2] = 0;

	param->robot_pos[2][0] = -16;
	param->robot_pos[2][1] = -10;
	param->robot_pos[2][2] = 0;

	param->robot_pos[3][0] = -15;
	param->robot_pos[3][1] = -10;
	param->robot_pos[3][2] = 0;

	param->robot_pos[4][0] = -14;
	param->robot_pos[4][1] = -10;
	param->robot_pos[4][2] = 0;

	param->robot_pos[5][0] = -18;
	param->robot_pos[5][1] = -11;
	param->robot_pos[5][2] = 0;

	param->robot_pos[6][0] = -17;
	param->robot_pos[6][1] = -11;
	param->robot_pos[6][2] = 0;

	param->robot_pos[7][0] = -16;
	param->robot_pos[7][1] = -11;
	param->robot_pos[7][2] = 0;

	param->robot_pos[8][0] = -15;
	param->robot_pos[8][1] = -11;
	param->robot_pos[8][2] = 0;

	param->robot_pos[9][0] = -14;
	param->robot_pos[9][1] = -11;
	param->robot_pos[9][2] = 0;
}

void fix_obstacle_positions(PARAM* param)
{
	param->M = 1;
	param->obstacle_pos[0][0] = 0;
	param->obstacle_pos[0][1] = 0;
	param->obstacle_pos[0][2] = 5;

	param->obstacle_pos[1][0] = -10;
	param->obstacle_pos[1][1] = 10;
	param->obstacle_pos[1][2] = 7;
}





void initialize_result(node* result_node, PARAM* param)
{
	result_node->isEmpty = 0;
	result_node->N = param->N;
	result_node->sequence_numel = param->time_array_count;
	memcpy(&result_node->robot_pos[0][0], &param->robot_pos[0][0], sizeof(float) * param->N * 3);
	result_node->F = param->H * h_calculate_H1(*result_node, param, result_node->N);
	result_node->G = 0;
	result_node->reached_destination = 0;
	memcpy(&result_node->behaviorIndices, &param->sequence_array, sizeof(int)*SEQ_MAX);
	result_node->seen_map.seen_count = 0;
	result_node->seen_map.mapsize = MAPSIZE;
	for(int i = 0; i < MAPSIZE*2; i++)
	{
		for(int j = 0; j < MAPSIZE*2; j++)
		{
			result_node->seen_map.seen[i][j] = 0;
		}
	}
	return;
}

RETURN testmain(PARAM* param, int isAided, std::vector<float> time_array, 
	std::vector<int> sequence_array, std::vector<uint8_t> isFixed, float coverage_ratio)
{
	printf("starting\n");

	clock_t start;
    double duration;

    start = std::clock();

    RETURN return_1;

	initialize_parameters(param, time_array, sequence_array, isFixed, coverage_ratio);

	if(isAided == 1) SMHAstar_wrapper(param, &return_1);
	else if(isAided == 0){
		node result_node;
		initialize_result(&result_node, param);
	
		noSMHAstar(param, &return_1, &result_node);
		//printf("after returning from noSMHAstar function....\n");
		//printf("return_1 cost_of_path = %f\n", return_1.cost_of_path);
	}
	else if(isAided == 1)
	{
		onlySMHAstar_wrapper(param, &return_1);
	}

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    std::cout<<"Elapsed: "<< duration << " Seconds" << '\n';


	return return_1;
}
