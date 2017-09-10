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


//////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace std;


const int DIR = 7; //directions I can take {rendezvous, flocking, north, south, west, east, no_move}
const int ARRAY_SIZE = 1024;
const int ROBOT_MAX = 16;
const int OBS_MAX = 5;
const int SEQ_MAX = 11;
const int QUEUE_MAX = 3;

char *behavior_array[DIR] = { "rendezvous", "flocking", "flock_east", "flock_north", "flock_west", "flock_south", "antirendezvous" };


/* each 'cell' corresponds to a node */
typedef struct node
{
	int isEmpty;
	int N;
	float robot_pos[ROBOT_MAX][3]; //(x,y,theta)
	float F; //cost that includes priority
	float G; //just cost
	int behaviorIndices[SEQ_MAX]; //0->rendezvous, 1->flocking, 2->flock_east, 3->flock_north, 4->flock_west, 5->flock_south, 6->move_stop
	long long behaviorIdx;
	int reached_destination;
	int sequence_numel;
} node;


typedef struct PARAM
{
	int N; //number of robots
	int M; //number of obstacles
	float H; //w1
	float robot_pos[ROBOT_MAX][3]; //assume no more than 50 robots at this time; each row (x,y,theta)
	float target_center[2]; //(x,y)
	float target_radius; //r
	float robot_radius;
	float obstacle_pos[OBS_MAX][3];
	//assume no more than 50 obstacles;obstacles are circles; each row (x,y,r)
	float ti; //start time
	float tf; //end time
	float dt; //each time step
	float dT; //each decision time
	float mapsize; //mapsize row, col is same
				   //newly added
	float H2; //w2
	int q_count; //number of queues

} PARAM;

typedef struct Queue_p
{
	std::map<int, node*> *closed_dict_p;
	std::vector<node> *h_open_p;
	std::vector<int> *expanded_number_per_iter_p;
	int iteration;
} Queue_p;

typedef struct POS_GROUP
{
	float robot_pos[ROBOT_MAX][3];
} POS_GROUP;

typedef struct POS_TWO
{
	float pos[2];
} POS_TWO;

typedef struct RETURN
{
	std::vector<POS_GROUP> robot_positions;
	int sequence_length;
	std::vector<int> sequence_end_indices;
	float cost_of_path;
	uint8_t is_valid_path;
	std::vector<int> sequence_names;
}

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


__device__
void d_robot_model_i(node* current, int i, float u_v, float u_w, float dt)
{
	int N = current->N;
	float max_linear_velocity = 2;
	float max_angular_velocity = CUDART_PI_F / 8;
	u_v = fmaxf(-max_linear_velocity, fminf(max_linear_velocity, u_v));
	u_w = fmaxf(-max_angular_velocity, fminf(max_angular_velocity, u_w));
	current->robot_pos[i][0] = current->robot_pos[i][0] + u_v*cosf(current->robot_pos[i][2])*dt;
	current->robot_pos[i][1] = current->robot_pos[i][1] + u_v*sinf(current->robot_pos[i][2])*dt;
	current->robot_pos[i][2] = d_wrapToPi(current->robot_pos[i][2] + u_w*dt);
	return;
}


__device__
void d_antirendezvous(node* future, node *current, int i, float dt)
{
	//printf("Inside rendezvous\n");
	float connectivity_radius = 20;
	float gain_v = 1;
	float gain_w = 1;
	int N = current->N;
	float v[2], dv[2], b[2];
	float w, n;
	float position_i[2], position_j[2], d[2];
	float heading_i, heading_j;
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
		heading_j = current->robot_pos[j][2];
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
	float gain_v = 1;
	float gain_w = 1;
	int N = current->N;
	float v[2], dv[2], b[2];
	float w, n;
	float position_i[2], position_j[2], d[2];
	float heading_i, heading_j;
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
		heading_j = current->robot_pos[j][2];
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
	float gain_v = 1;
	float gain_w = 1;
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
__device__ op_func func[DIR] = { d_rendezvous, d_flocking, d_flock_east, d_flock_north, d_flock_west, d_flock_south, d_antirendezvous };
__device__ char *d_behavior_array[DIR] = { "rendezvous", "flocking", "flock_east", "flock_north", "flock_west", "flock_south", "antirendezvous" };


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
	float safety_bounds = param->robot_radius;
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
	float poses_delta_summed[2];
	poses_delta_summed[0] = 0;
	poses_delta_summed[1] = 0;
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
	float priority = 0;
	float priority_i;
	int i;
	float target_radius = d_param->target_radius;
	float target_center[2];
	target_center[0] = d_param->target_center[0];
	target_center[1] = d_param->target_center[1];
	float robot_radius = d_param->robot_radius;
	int N = d_param->N;
	for (int i = 0; i < N; i++)
	{
		priority_i = sqrt(powf(d_expanded.robot_pos[i][0] - target_center[0], 2)
			+ powf(d_expanded.robot_pos[i][1] - target_center[1], 2));
		priority_i -= target_radius - robot_radius;
		priority_i = fmaxf(priority_i, 0);
		priority += priority_i;
	}
	if (mode == 0) return priority;
	else if (mode == 1) {
		float robot_cx = 0;
		float robot_cy = 0;
		for (int i = 0; i < d_param->N; i++)
		{
			robot_cx += d_expanded.robot_pos[i][0];
			robot_cy += d_expanded.robot_pos[i][1];
		}
		robot_cx /= d_param->N;
		robot_cy /= d_param->N;
		float dist_to_ob = sqrt(powf(robot_cx - d_param->obstacle_pos[0][0], 2) + powf(robot_cy - d_param->obstacle_pos[0][1], 2)) - d_param->obstacle_pos[0][2];;
		for (int j = 0; j < d_param->M; j++)
		{
			dist_to_ob = fminf(dist_to_ob, sqrt(powf(robot_cx - d_param->obstacle_pos[j][0], 2) + powf(robot_cy - d_param->obstacle_pos[j][1], 2)) - d_param->obstacle_pos[0][2]);
		}
		return priority + d_param->N *  powf(3, -(dist_to_ob - 5));
	}

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* param @ d_expanded - result of expanded nodes
param @ d_open - input of nodes to expand
param @ d_param - parameters about the swarm
param @ dir - Number of directions (7 for now)
param @ iteration - used for backtracing */
__global__
void k_expandStates(node *d_expanded, node* d_open, PARAM* d_param, int dir, int iteration, int real_copies, int queue_i)
{
	//int dir_index = blockIdx.x;
	//int node_index = threadIdx.x;
	//int robot_index = blockIdx.y;
	//int index = node_index * dir + dir_index;

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
	float ti = d_param->ti;
	float dT = d_param->dT;
	float tf = d_param->tf;
	float tbegin = ti + d_open[node_index].sequence_numel * dT;

	float tend = tbegin + dT;
	int steps = (int)dT / dt;
	int currSequenceNumel = d_open[node_index].sequence_numel + 1;

	/* Only write to global memory once */
	if (robot_index == 0) {
        	d_expanded[index] = d_open[node_index];
	  	//printf("d_open.F is %f\n", d_open[node_index].F);
	}
	
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
		d_expanded[index].behaviorIndices[currSequenceNumel] = dir_index;
		d_expanded[index].behaviorIdx = d_expanded[index].behaviorIdx * 10 + dir_index;

		/* Swarm sequence invalid if exceeded end time */
		d_expanded[index].isEmpty = (tbegin >= tf);

		/* Check swarm if reached destination */
		d_expanded[index].reached_destination = d_target_reached(d_expanded[index], d_param);

		/* cost_estimate is total cost: cost_estimate = cost_next + H*cost_to_go */
		d_expanded[index].F = d_expanded[index].G + (d_param->H) * d_calculate_H(d_expanded[index], d_param, queue_i);
		d_expanded[index].sequence_numel = currSequenceNumel;
		//printf("SEQUENCE_NUMEL IS %d, F is %f\n", d_expanded[index].sequence_numel, d_expanded[index].F);
	}
	return;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////

/* HOST FUNCTIONS */

/* calculate the heuristic between curr node and the destination */
float h_calculate_H1(float robot_pos[ROBOT_MAX][3], PARAM* param, int N)
{
	float priority = 0;
	for (int i = 0; i < N; i++)
	{
		priority += sqrt(pow(robot_pos[i][0] - param->target_center[0], 2)
			+ pow(robot_pos[i][1] - param->target_center[1], 2));
		priority -= param->target_radius - param->robot_radius;
	}

	return priority;
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


void print_map(node best_node, PARAM* param)
{
	/* Print location of all robots, obstacles and the target location */
	/* Empty space is 0 */
	/* center of Robots are 1 */
	/* Center of Obstacles are 2 */
	/* Center of target is 3 */
	int map_size = (int)param->mapsize * 2 * param->mapsize * 2;
	int* map_grid = new int[map_size];
	for (int i = 0; i < map_size; i++) map_grid[i] = 0;

	for (int i = 0; i < param->N; i++)
	{
		int index = convert_position(param->mapsize, best_node.robot_pos[i][0], best_node.robot_pos[i][1]);
		map_grid[index] = 1;
	}

	for (int i = 0; i < map_size; i++)
	{
		std::vector<float> pos = convert_index(param->mapsize, i);
		for (int j = 0; j < param->M; j++)
		{
			float norm_d = sqrt(pow(param->obstacle_pos[j][0] - pos[0], 2) + pow(param->obstacle_pos[j][1] - pos[1], 2));
			if (norm_d <= param->obstacle_pos[j][2]) map_grid[i] = 2;

			float norm_d1 = sqrt(pow(param->target_center[0] - pos[0], 2) + pow(param->target_center[1] - pos[1], 2));
			if (norm_d1 <= param->target_radius) map_grid[i] = 3;
		}
	}


	for (int i = 0; i < (int)(param->mapsize * 2) * (param->mapsize * 2); i++)
	{
		if (i % (int)(2 * param->mapsize) == 0) printf("\n");
		if (map_grid[i] == 0) printf(".");
		else printf("%d", map_grid[i]);
	}
	printf("\n");
	delete(map_grid);
	return;
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


std::vector<Queue_p*> initializeQps(PARAM* param)
{
	std::vector<Queue_p*> qps;

	for (int i = 0; i < param->q_count; i++)
	{
		Queue_p *qp_p = new Queue_p;
		qp_p->h_open_p = new std::vector<node>;
		qp_p->iteration = 1;


		qps.push_back(qp_p);
	}



	for (int i = 0; i < param->q_count; i++)
	{
		node h_start; //starting node

		h_start.isEmpty = 0;
		h_start.N = param->N;
		h_start.sequence_numel = -1;

		std::copy(&param->robot_pos[0][0], &param->robot_pos[0][0] + param->N * 3, &h_start.robot_pos[0][0]);

		h_start.F = param->H * h_calculate_H1(h_start.robot_pos, param, h_start.N);
		h_start.G = 0;
		h_start.reached_destination = 0;

		(*qps[i]->h_open_p).push_back(h_start);
	}

	return qps;
}


float h_calculate_H(node d_expanded, PARAM* d_param, int mode)
{
	float priority = 0;
	float priority_i;
	int i;
	float target_radius = d_param->target_radius;
	float target_center[2];
	target_center[0] = d_param->target_center[0];
	target_center[1] = d_param->target_center[1];
	float robot_radius = d_param->robot_radius;
	int N = d_param->N;
	for (int i = 0; i < N; i++)
	{
		priority_i = sqrt(pow(d_expanded.robot_pos[i][0] - target_center[0], 2)
			+ powf(d_expanded.robot_pos[i][1] - target_center[1], 2));
		priority_i -= target_radius - robot_radius;
		priority_i = max(priority_i, (float) 0.0);
		priority += priority_i;
	}
	if (mode == 0) return priority;
	else if (mode == 1) {
		float dist_to_ob = 0;
		float dist_to_ob_i = 0;
		for (int i = 0; i < d_param->N; i++)
		{
			dist_to_ob_i = 0;
			dist_to_ob_i = sqrt(powf(d_expanded.robot_pos[i][0] - d_param->obstacle_pos[0][0], 2) + powf(d_expanded.robot_pos[i][1] - d_param->obstacle_pos[0][1], 2)) - d_param->obstacle_pos[0][2];
			for (int j = 0; j < d_param->M; j++)
			{
				dist_to_ob_i = fminf(dist_to_ob_i, sqrt(powf(d_expanded.robot_pos[i][0] - d_param->obstacle_pos[j][0], 2) + powf(d_expanded.robot_pos[i][1] - d_param->obstacle_pos[j][1], 2)) - d_param->obstacle_pos[0][2]);
			}
			dist_to_ob_i = dist_to_ob_i - 3;
			dist_to_ob_i = fmaxf(dist_to_ob_i, 0);
			dist_to_ob += dist_to_ob_i;
		}


		//printf("priority, additional part (%f, %f)\n", priority, powf(3, -(dist_to_ob - 5)));


		return priority + dist_to_ob;
	}

}

void updateQueues(std::vector<Queue_p*> qps, node curr, PARAM* param, int queue_i)
{
	curr.F = curr.G + param->H * h_calculate_H(curr, param, 0);
	float queue_0_F = curr.F;
	(*qps[0]->h_open_p).push_back(curr);
	for (int i = 1; i < qps.size(); i++)
	{
		curr.F = curr.G + param->H * h_calculate_H(curr, param, i);
		//printf("curr_i_F = %f, cuff_0_F = %f\n", curr.F, queue_0_F);
		if (curr.F <= param->H2 * queue_0_F) (*qps[i]->h_open_p).push_back(curr);
	}
	return;
}


void expandStates(std::vector<Queue_p*> qps, PARAM* param, node* best_node_p, int queue_i)
{
	Queue_p* qp = qps[queue_i];
	PARAM* d_param; /* device parameters */
	cudaMalloc(&d_param, sizeof(PARAM));
	cudaMemcpy(d_param, param, sizeof(PARAM), cudaMemcpyHostToDevice);

	node *d_open; /* device open nodes */
	node *d_expanded; /* device expanded nodes */

					  /* Dequeue from h_open */
	int iteration = qp->iteration;
	printf("Inside ExpandStates....iteration is %d\n", iteration);

	int h_open_size = (*qp->h_open_p).size();
	int real_copies = (h_open_size > ARRAY_SIZE) ? ARRAY_SIZE : h_open_size;
	node* h_open_array = new node[real_copies]; /* temporary data used for cudaMemcpy of h_open */
	node* h_expanded = new node[real_copies * DIR]; /* data used for retrieving expanded nodes data; Also used for backtracking */

	printf("For %d, real_copies %d\n", queue_i, real_copies);

	std::copy((*qp->h_open_p).begin(), (*qp->h_open_p).begin() + real_copies, h_open_array);
	/* Erase the same nodes from all queues */
	(*qp->h_open_p).erase((*qp->h_open_p).begin(), (*qp->h_open_p).begin() + real_copies);
	/* TODO: Make sure this portion of code is correct!!!! */


	//////////////
	for (int queue_j = 0; queue_j < param->q_count; queue_j++)
	{
		if (queue_i == queue_j) continue;
		std::vector<int> erase_indices;
		for (int i = 0; i < real_copies; i++) {
			for (int j = 0; j < (*qps[queue_j]->h_open_p).size(); j++)
			{
				if (h_open_array[i].behaviorIdx == (*qps[queue_j]->h_open_p)[j].behaviorIdx) {
					erase_indices.push_back(j);
					break;
				}
			}
		}

		std::sort(erase_indices.begin(), erase_indices.end());

		for (int k = erase_indices.size() - 1; k >= 0; k--)
		{
			//printf("erase_indices[%d] is %d\n", k, erase_indices[k]);

			(*qps[queue_j]->h_open_p).erase((*qps[queue_j]->h_open_p).begin() + erase_indices[k]);
		}
		
	}


	/* Copy necessary data to device memory */
	cudaMalloc(&d_open, sizeof(node) * real_copies);
	cudaMemcpy(d_open, h_open_array, sizeof(node) * real_copies, cudaMemcpyHostToDevice);
	cudaMalloc(&d_expanded, sizeof(node) * real_copies * DIR);


	/* Allocate proper blocks and threads */
	const dim3 blockSize(DIR, param->N, 1);
	const dim3 gridSize(real_copies, 1, 1);
	//const dim3 blockSize(real_copies, 1, 1);
	//const dim3 gridSize(dir, param->N, 1);

	/* Run the GPU code */
	k_expandStates << < gridSize, blockSize >> >(d_expanded, d_open, d_param, DIR, iteration, real_copies, queue_i);

	/* Copy back from GPU to CPU */
	cudaMemcpy(h_expanded, d_expanded, sizeof(node) * real_copies * DIR, cudaMemcpyDeviceToHost);

	/* Update open list with expanded nodes and update best_node */
	for (int ind = 0; ind < real_copies * DIR; ind++)
	{
		if (h_expanded[ind].isEmpty == 0 && h_expanded[ind].reached_destination == 0)
		{
			node curr = h_expanded[ind];
			//printf("IN HOST SEQUENCE_NUMEL IS %d\n", curr.sequence_numel);
			updateQueues(qps, curr, param, queue_i);
		}
		else if (h_expanded[ind].isEmpty == 0 && h_expanded[ind].reached_destination == 1
			&& h_expanded[ind].G < best_node_p->G) {
			memcpy(best_node_p, &h_expanded[ind], sizeof(node));
			printf("UPDATING BEST NODE!! cost is %f, sequence_numel is %d\n", best_node_p->G, best_node_p->sequence_numel);
			printf("printing sequence...\n");
			for (int i = 0; i <= best_node_p->sequence_numel; i++)
			{
				int index = best_node_p->behaviorIndices[i];
				//printf("hello world????\n");
				printf("%s, ", behavior_array[index]);
				//printf("%d ", index);
			}
			printf("\n");
		}
	}

	for (int i = 0; i < param->q_count; i++)
	{
		printf("Queue%d size is %d\n", i, (*qps[i]->h_open_p).size());
	}


	/* Sort the open_list */
	for (int i = 0; i < qps.size(); i++)
	{
		std::sort((*qps[i]->h_open_p).begin(), (*qps[i]->h_open_p).end(), [](node left, node right) {return left.F < right.F; });
	}



	/* Prepare for next iteration */
	qp->iteration++;
	delete(h_open_array);
	cudaFree(d_expanded);
	cudaFree(d_open);
	cudaFree(d_param);

}


/* returns @ string of behavior sequence (for now) behavior__behavior__behavior__....behavior*/
node SMHAstar(PARAM* param)
{
	clock_t start = clock();
	cout << "Starting IMHA star " << endl;

	std::vector<Queue_p*> qps = initializeQps(param);
	node result;
	result.isEmpty = 1;
	result.F = numeric_limits<float>::max();

	node* best_node_p = new node[1];
	best_node_p->reached_destination = 0;
	best_node_p->isEmpty = 1;
	best_node_p->F = numeric_limits<float>::max();
	best_node_p->G = numeric_limits<float>::max();
	best_node_p->N = param->N;


	while (!(*qps[0]->h_open_p).empty())
	{
		/* Exit if exceeded the amount of time */
		clock_t end = clock();
		float time_elapsed = float(end - start);
		if (time_elapsed > 120000000)
		{
			printf("Exceeded time limit of %f (ms)", time_elapsed);
			return result;
		}

		for (int queue_i = 1; queue_i < param->q_count; queue_i++)
		{
			//Since we will only have 2 queues, I will do some hardcoding
			//If there are no elements in queue1, then just expand queue0
			if ((*qps[queue_i]->h_open_p).empty()) {
				node minKey_0 = (*qps[0]->h_open_p)[0];
				printf("minKey_0.F %f \n", minKey_0.F);
				printf("best_i.G %f\n", best_node_p->G);

				if (best_node_p->G <= minKey_0.F)
				{
					if (best_node_p->G < numeric_limits<float>::max())
					{
						printf("DONE WITH SEARCH!! NOW PRINTING RESULTS\n");
						printf("Current best node cost %f, sequence_numel is %d\n", best_node_p->G, best_node_p->sequence_numel);
						//print_behavior_sequence(*qps[0]->closed_dict_p, *qps[0]->expanded_number_per_iter_p, qps[0]->best_node);
						print_distance_left(*best_node_p, param);

						printf("printing sequence...\n");
						for (int i = 0; i <= best_node_p->sequence_numel; i++)
						{
							int index = best_node_p->behaviorIndices[i];
							//printf("hello world????\n");
							printf("%s, ", behavior_array[index]);
							//printf("%d ", index);
						}
						printf("\n");

						memcpy(&result, best_node_p, sizeof(node));
						clock_t end = clock();
						float time_elapsed = float(end - start);
						cout << "Time to calculate the route (ms): " << time_elapsed << endl;
						printf("inside smha!!! result sequence_numel!!!%d\n", result.sequence_numel);
						return result;
					}
				}
				else
				{
					expandStates(qps, param, best_node_p, 0);
				}

				///////////////////////

				//Need to check if best_node has been updated; if so, go with the best node
				/*
				printf("queue %d is empty\n", queue_i);
				for (int queue_j = 0; queue_j < param->q_count; queue_j++)
				{
					if (qps[queue_j]->best_node.isEmpty == 0 && qps[queue_j]->best_node.F < result.F) memcpy(&result, &(qps[queue_j]->best_node), sizeof(node));
				}

				return result;
				*/
			}
			else {
				node minKey_i = (*qps[queue_i]->h_open_p)[0];
				node minKey_0 = (*qps[0]->h_open_p)[0];

				/* Debugging purposes.... */

				printf("minKey_i.F, minKey_0.F %f %f \n", minKey_i.F, minKey_0.F);
				printf("best_i.G %f\n", best_node_p->G);


				if (minKey_i.F <= param->H2 * minKey_0.F)
				{
					printf("QUEUE %d!!!\n", queue_i);
					//print_map(minKey_i, param);
					if (best_node_p->G <= minKey_i.F)
					{
						if (best_node_p->G < numeric_limits<float>::max())
						{
							//return path pointed by bpi(best_node)
							printf("DONE WITH SEARCH!! NOW PRINTING RESULTS\n");
							printf("Current best node cost %f, sequence_numel is %d\n", best_node_p->G, best_node_p->sequence_numel);

							printf("printing sequence...\n");
							for (int i = 0; i <= best_node_p->sequence_numel; i++)
							{
								int index = best_node_p->behaviorIndices[i];
								//printf("%d ", index);
								//printf("hello world?");
								printf("%s, ", behavior_array[index]);
							}
							printf("\n");


							//print_behavior_sequence(*qps[queue_i]->closed_dict_p, *qps[queue_i]->expanded_number_per_iter_p, qps[queue_i]->best_node);
							print_distance_left(*best_node_p, param);

							memcpy(&result, best_node_p, sizeof(node));
							clock_t end = clock();
							float time_elapsed = float(end - start);
							cout << "Time to calculate the route (ms): " << time_elapsed << endl;
							printf("inside smha!!! result sequence_numel!!!%d\n", result.sequence_numel);
							return result;

						}
					}
					else {
						expandStates(qps, param, best_node_p, queue_i);
					}
				}
				else
				{
					printf("queue 0!!!\n");
					//print_map(minKey_0, param);
					if (best_node_p->G <= minKey_0.F)
					{
						if (best_node_p->G < numeric_limits<float>::max())
						{
							printf("DONE WITH SEARCH!! NOW PRINTING RESULTS\n");
							printf("Current best node cost %f, sequence_numel is %d\n", best_node_p->G, best_node_p->sequence_numel);
							//print_behavior_sequence(*qps[0]->closed_dict_p, *qps[0]->expanded_number_per_iter_p, qps[0]->best_node);
							print_distance_left(*best_node_p, param);

							printf("printing sequence...\n");
							for (int i = 0; i <= best_node_p->sequence_numel; i++)
							{
								int index = best_node_p->behaviorIndices[i];
								//printf("hello world????\n");
								printf("%s, ", behavior_array[index]);
								//printf("%d ", index);
							}
							printf("\n");

							memcpy(&result, best_node_p, sizeof(node));
							clock_t end = clock();
							float time_elapsed = float(end - start);
							cout << "Time to calculate the route (ms): " << time_elapsed << endl;
							printf("inside smha!!! result sequence_numel!!!%d\n", result.sequence_numel);
							return result;
						}
					}
					else
					{
						expandStates(qps, param, best_node_p, 0);
					}
				}
			}
			
		}
		printf("\n\n");
	}

	for (int queue_j = 0; queue_j < param->q_count; queue_j++)
	{
		if (best_node_p->isEmpty == 0 && best_node_p->G < result.G) memcpy(&result, best_node_p, sizeof(node));
	}


	if (result.isEmpty == 1) printf("no route found\n");
	else {
		printf("printing sequence...\n");
		for (int i = 0; i <= result.sequence_numel; i++)
		{
			int index = result.behaviorIndices[i];
			//printf("%d ", index);
			//printf("hello world?");
			printf("%s, ", behavior_array[index]);
		}
		printf("\n");
	}


	clock_t end = clock();
	float time_elapsed = float(end - start);
	cout << "Time to calculate the route (ms): " << time_elapsed << endl;
	return result;


}








/////////////////////////////////////////////////////////////////////////////////////////////////////////

void initialize_parameters(PARAM* param)
{
	srand(time(0));

	param->N = 10;
	param->M = 3; //2
	param->H = 1;
	param->mapsize = 20;
	param->ti = 0;
	param->dt = 0.1;
	param->tf = 35;
	param->dT = 5;
	param->target_center[0] = 10;
	param->target_center[1] = 10;
	param->target_radius = 7;
	param->robot_radius = 0.5;
	param->q_count = 2;
	param->H2 = 1;

	/* Initialize robot_pos */
	float bottom_left_x = -param->mapsize + 1;
	float bottom_left_y = -param->mapsize + 1;
	float width = (param->target_radius) * 2;
	float height = (param->target_radius) * 2;

	int i;
	for (i = 0; i < param->N; i++)
	{
		param->robot_pos[i][0] = bottom_left_x + width * ((float)(rand() % 1000)) / 1000;
		param->robot_pos[i][1] = bottom_left_y + height * ((float)(rand() % 1000)) / 1000;
		param->robot_pos[i][2] = remainder(((float)(rand() % 1000)) / 1000, 2.0*M_PI);
	}

	/* Initialize obstacle_pos */
	float obstacle_min_radius = 2;
	float obstacle_max_radius = 5;

	for (i = 0; i < param->M; i++)
	{
		param->obstacle_pos[i][0] = fmod(((float)(rand() % 1000)) / 10, (2 * param->mapsize)) - param->mapsize;
		param->obstacle_pos[i][1] = fmod(((float)(rand() % 1000)) / 10, (2 * param->mapsize)) - param->mapsize;
		param->obstacle_pos[i][2] = fmod(((float)(rand() % 1000)) / 10, obstacle_max_radius);
		if (param->obstacle_pos[i][2] < obstacle_min_radius) param->obstacle_pos[i][2] += obstacle_min_radius;
	}

	return;

}



void fix_robot_positions(PARAM* param)
{
	param->N = 10;
	param->robot_pos[0][0] = -18;
	param->robot_pos[0][1] = -10;
	param->robot_pos[0][2] = 0;

	param->robot_pos[1][0] = -16;
	param->robot_pos[1][1] = -10;
	param->robot_pos[1][2] = 0;

	param->robot_pos[2][0] = -14;
	param->robot_pos[2][1] = -10;
	param->robot_pos[2][2] = 0;

	param->robot_pos[3][0] = -12;
	param->robot_pos[3][1] = -10;
	param->robot_pos[3][2] = 0;

	param->robot_pos[4][0] = -18;
	param->robot_pos[4][1] = -12;
	param->robot_pos[4][2] = 0;

	param->robot_pos[5][0] = -16;
	param->robot_pos[5][1] = -12;
	param->robot_pos[5][2] = 0;

	param->robot_pos[6][0] = -14;
	param->robot_pos[6][1] = -12;
	param->robot_pos[6][2] = 0;

	param->robot_pos[7][0] = -18;
	param->robot_pos[7][1] = -14;
	param->robot_pos[7][2] = 0;

	param->robot_pos[8][0] = -16;
	param->robot_pos[8][1] = -14;
	param->robot_pos[8][2] = 0;

	param->robot_pos[9][0] = -14;
	param->robot_pos[9][1] = -14;
	param->robot_pos[9][2] = 0;

	param->robot_pos[10][0] = -18;
	param->robot_pos[10][1] = -10;
	param->robot_pos[10][2] = 0;

	param->robot_pos[11][0] = -17;
	param->robot_pos[11][1] = -10;
	param->robot_pos[11][2] = 0;

	param->robot_pos[12][0] = -16;
	param->robot_pos[12][1] = -10;
	param->robot_pos[12][2] = 0;

	param->robot_pos[13][0] = -15;
	param->robot_pos[13][1] = -10;
	param->robot_pos[13][2] = 0;

	param->robot_pos[14][0] = -14;
	param->robot_pos[14][1] = -10;
	param->robot_pos[14][2] = 0;

	param->robot_pos[15][0] = -18;
	param->robot_pos[15][1] = -9;
	param->robot_pos[15][2] = 0;

	param->robot_pos[16][0] = -17;
	param->robot_pos[16][1] = -9;
	param->robot_pos[16][2] = 0;

	param->robot_pos[17][0] = -10;
	param->robot_pos[17][1] = -12;
	param->robot_pos[17][2] = 0;

	param->robot_pos[18][0] = -12;
	param->robot_pos[18][1] = -14;
	param->robot_pos[18][2] = 0;

	param->robot_pos[19][0] = -10;
	param->robot_pos[19][1] = -14;
	param->robot_pos[19][2] = 0;
}

void fix_obstacle_positions(PARAM* param)
{
	param->M = 3;
	param->obstacle_pos[0][0] = 0;
	param->obstacle_pos[0][1] = 0;
	param->obstacle_pos[0][2] = 5;

	param->obstacle_pos[1][0] = 0;
	param->obstacle_pos[1][1] = -10;
	param->obstacle_pos[1][2] = 5;

	param->obstacle_pos[2][0] = -15;
	param->obstacle_pos[2][1] = 15;
	param->obstacle_pos[2][2] = 3.5;

}


void processParam(std::vector<std::string> tokens, PARAM* param)
{
	// Assign parameters according to the parameter name
	if (tokens[0] == "N")
		param->N = std::stof(tokens[1]);
	else if (tokens[0] == "robot_radius")
		param->robot_radius = std::stof(tokens[1]);
	else if (tokens[0] == "M")
		param->M = std::stof(tokens[1]);
	else if (tokens[0] == "H")
		param->H = std::stof(tokens[1]);
	else if (tokens[0] == "H2")
		param->H2 = std::stof(tokens[1]);
	else if (tokens[0] == "q_count")
		param->q_count = std::stof(tokens[1]);
	else if (tokens[0] == "mapsize")
		param->mapsize = std::stof(tokens[1]);
	else if (tokens[0] == "ti")
		param->ti = std::stof(tokens[1]);
	else if (tokens[0] == "tf")
		param->tf = std::stof(tokens[1]);
	else if (tokens[0] == "dt")
		param->dt = std::stof(tokens[1]);
	else if (tokens[0] == "dT")
		param->dT = std::stof(tokens[1]);
	else if (tokens[0] == "target_radius")
		param->target_radius = std::stof(tokens[1]);

}


void loadParametersFromFile(std::string filename, PARAM* param)
{
	std::fstream file(filename);
	std::string str;
	unsigned int line = 1;
	size_t comment;

	// Get the parameters from specified file
	while (std::getline(file, str)) {
		comment = str.find('#', 0);
		if (comment != std::string::npos)
		{
			continue;
		}

		// String working variables
		std::istringstream iss(str);
		std::vector<std::string> tokens;

		// Place the parameter and value into the token array
		copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));

		// Ensure valid line before processing parameter
		if (tokens.size() == 2) {
			processParam(tokens, param);
		}

		/* Need to process robot_pos, obstacle_pos*/
		if (tokens.size() == 1) {
			if (tokens[0] == "robot_pos")
			{
				int count = 0;
				while (count < param->N)
				{
					std::getline(file, str);
					// String working variables
					std::istringstream iss(str);
					std::vector<std::string> tokens;

					// Place the parameter and value into the token array
					copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));
					if (tokens.size() != 3)
					{
						printf("Robot pos format wrong in file\n");
						return;
					}
					param->robot_pos[count][0] = std::stof(tokens[0]);
					param->robot_pos[count][1] = std::stof(tokens[1]);
					param->robot_pos[count][2] = std::stof(tokens[2]);
					count++;
				}
			}
			else if (tokens[0] == "obstacle_pos")
			{
				int count = 0;
				while (count < param->M)
				{
					std::getline(file, str);
					// String working variables
					std::istringstream iss(str);
					std::vector<std::string> tokens;

					// Place the parameter and value into the token array
					copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));
					if (tokens.size() != 3)
					{
						printf("obstacle pos format wrong in file\n");
						return;
					}
					param->obstacle_pos[count][0] = std::stof(tokens[0]);
					param->obstacle_pos[count][1] = std::stof(tokens[1]);
					param->obstacle_pos[count][2] = std::stof(tokens[2]);
					count++;
				}
			}
			else if (tokens[0] == "target_center")
			{
				std::getline(file, str);
				// String working variables
				std::istringstream iss(str);
				std::vector<std::string> tokens;

				// Place the parameter and value into the token array
				copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(tokens));
				if (tokens.size() != 2)
				{
					printf("target pos format wrong in file\n");
					return;
				}
				param->target_center[0] = std::stof(tokens[0]);
				param->target_center[1] = std::stof(tokens[1]);

			}
		}

		// Move to the next line in the parameters file
		line++;
	}

	return;
}


__global__
void k_SAVE(POS_GROUP* d_poses, node* d_result, node* d_start, PARAM* d_param)
{
	int robot_index = threadIdx.x;
	/* Error checking */
	if (robot_index >= d_param->N) return;

	if (robot_index == 0) memcpy(d_poses[0].robot_pos, d_param->robot_pos, sizeof(float)*d_param->N * 3);

	__syncthreads();

	/* Expand the nodes */
	float dt = d_param->dt;
	float ti = d_param->ti;
	float dT = d_param->dT;
	float tf = d_param->tf;
	int steps = (int)dT / dt;

	int sequence_count = d_result->sequence_numel + 1;

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


void SAVE_launch(node result_node, std::string filename, PARAM* param)
{
	/* Open file and retrieve file id */
	FILE* output_f;
	std::stringstream output_fname;
	output_fname << filename;

	PARAM* d_param; /* device parameters */
	gpuErrchk(cudaMalloc(&d_param, sizeof(PARAM)));
	gpuErrchk(cudaMemcpy(d_param, param, sizeof(PARAM), cudaMemcpyHostToDevice));

	node* d_result;
	gpuErrchk(cudaMalloc(&d_result, sizeof(node)));
	gpuErrchk(cudaMemcpy(d_result, &result_node, sizeof(node), cudaMemcpyHostToDevice));

	node h_start;
	h_start.isEmpty = 0;
	h_start.N = param->N;
	h_start.sequence_numel = -1;
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
	int seq_n = result_node.sequence_numel + 1;
	int h_result_size = steps* seq_n + 1;
	printf("steps, seq_n, %d, %d\n", steps, seq_n);
	printf("h_result_size is ... %d\n", h_result_size);
	POS_GROUP* h_poses = new POS_GROUP[h_result_size];
	POS_GROUP* d_poses;
	gpuErrchk(cudaMalloc(&d_poses, sizeof(POS_GROUP) * h_result_size));




	const dim3 gridSize(1, 1, 1);
	const dim3 blockSize(param->N, 1, 1);
	k_SAVE << <gridSize, blockSize >> >(d_poses, d_result, d_start, d_param);

	/* Copy back from GPU to CPU */
	cudaMemcpy(h_poses, d_poses, sizeof(POS_GROUP) * h_result_size, cudaMemcpyDeviceToHost);

	/* Now save everything onto the txt */
	cout << "saving everything in txt file" << endl;


	output_f = fopen(output_fname.str().c_str(), "w");
	int times = 0;
	printf("H_RESULT_SIZE IS %d\n", h_result_size);
	for (int i = 0; i < h_result_size; i++)
	{
		for (int j = 0; j < param->N; j++)
		{
			fprintf(output_f, (std::to_string(h_poses[i].robot_pos[j][0]) + " " + std::to_string(h_poses[i].robot_pos[j][1])
				+ " " + std::to_string(h_poses[i].robot_pos[j][2]) + "\n").c_str());
			times++;
		}
		fprintf(output_f, "\n");
	}
	return;
}



int testmain(int isAided, std::vector<float> time_array, std::vector<int> sequence_array, 
             std::vector<uint8_t> isFixed)
{
	printf("starting smha_final...\n");
	PARAM* param = new PARAM[1]; /* Allocate initial parameters on heap */
	/* If input param file is not specified, go with the default */

	initialize_parameters(param);
	fix_robot_positions(param);
	fix_obstacle_positions(param);

	node result_node = SMHAstar(param);


	if (result_node.isEmpty) return 0;

	printf("result_node sequence numel!!!!!!! %d\n", result_node.sequence_numel);

	SAVE_launch(result_node, "output.txt", param);
	

	delete(param);

	return 0;
}
