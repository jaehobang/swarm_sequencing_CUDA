#include <vector>

const int DIR = 8; //TODO: directions I can take {rendezvous, flocking, north, south, west, east, no_move}
const int ARRAY_SIZE = 1024;
const int ROBOT_MAX = 10;
const int OBS_MAX = 10;
const int SEQ_MAX = 10;
const int QUEUE_SIZE = 1;
//const float TARGET_RATIO = 0.05;
const int MAPSIZE = 40;

typedef struct SeenMap {
  int seen[MAPSIZE*2][MAPSIZE*2];
  int seen_count;
  int mapsize;
} SeenMap;

typedef struct node
{
  int isEmpty;
  int reached_destination;
  bool optimal;
  int N;
  float coverage_ratio;
  float robot_pos[ROBOT_MAX][3]; //(x,y,theta)
  float F; //cost that includes priority
  float G; //just cost
  int behaviorIndices[SEQ_MAX]; //0->rendezvous, 1->flocking, 2->flock_east, 3->flock_north, 4->flock_west, 5->flock_south, 6->move_stop
  long long behaviorIdx;

  int sequence_numel;
  SeenMap seen_map;
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
  float time_array[SEQ_MAX];
  int time_array_count;
  int sequence_array[SEQ_MAX];
  int sequence_array_count;
  int fix_count;
  float coverage_ratio;

} PARAM;

typedef struct Queue
{
  std::vector<node> h_open;
  int iteration;
} Queue;

typedef struct POS
{
  float robot_pos[ROBOT_MAX][3];
} POS;

typedef struct POS_TWO
{
  float pos[2];
} POS_TWO;

typedef struct RETURN
{
  std::vector<POS> robot_positions;
  std::vector<int> sequence_end_indices;
  float cost_of_path;
  float coverage_ratio;
  uint8_t is_valid_path;
  uint8_t is_complete;
  uint8_t is_optimal;
  std::vector<std::string> sequence_string_array;
  SeenMap seen_map;
} RETURN;

