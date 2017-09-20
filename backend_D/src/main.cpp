#include "ros/ros.h"
#include "custom_messages/R2D.h"
#include <iostream>
#include <fstream>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"


/*
time stamp
int64 event_type 0 = screen_location, 1 = button_click, 2 = trajectory information (cost_of_path, validity)
float32[2] screen_pos
string button_name
float64 cost_of_path
bool is_valid_path
bool is_train
bool is_aided
int64 map_number
int[] time_array
string[] sequence_string_array


------------------------
int main( int argc, char* argv[] )
{
      ofstream myfile;
      myfile.open ("example.csv");
      myfile << "This is the first cell in the first column.\n";
      myfile << "a,b,c,\n";
      myfile << "c,s,v,\n";
      myfile << "1,2,3.456\n";
      myfile << "semi;colon";
      myfile.close();
      return 0;
}
*/

using namespace std;


//1 session is going to correspond to 1 person
ofstream myfile;
int iteration = 0; //when user start, it will be 1. So first attempt is 1st iteration
int isTraining = 0;
uint8_t isAided = 2;
int map_num = 0; //1st map they encounter is map1
string name = "";


/* Available buttons will be 
	1. generate - behavior_sequence - TODO: fix R2D.msg (string[] behavior_sequence)
	2. rendezvous
	3. flocking
	4. antirendezvous
	5. flock_north
	6. flock_east
	7. flock_south
	8. flock_west 
	9. time_array - just a thought for interface, whenever you press a location on the bar, 
						a small tick appears and that i
	*/
	


void callBack(const custom_messages::R2D::ConstPtr& msg)
{
	string col_name = ""; //NAME
  string col_aided = "";
	string col_train = ""; //TRAIN OR TEST
	string col_map = ""; //Map number
	string col_iter = ""; //Map iteration
	string col_event = ""; //Map event
	string col_time = ""; //Map time
	string col_cost = "";

	boost::posix_time::ptime my_posix_time = msg->stamp.toBoost();
	string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
	col_time = iso_time_str;

	if(name != msg->name) {
		name = msg->name;
		col_name = msg->name;
	}
  if(isAided != msg->is_aided){
    isAided = msg->is_aided;
    if(isAided == 1) col_aided = "aided";
    else col_aided = "unaided";
  }
	if(msg->is_train != isTraining) {
		isTraining = msg->is_train;
		if(isTraining == 0) col_train = "test";
		else col_train = "train";
	}
	if((int)msg->map_number != map_num){
		map_num = (int) msg->map_number;
		col_map = "map" + std::to_string(map_num);
		iteration++;
		col_iter = "iter" + std::to_string(iteration);
	}

	if(msg->event_type == 0){
		col_event = "Pressed " + std::to_string(msg->screen_pos[0]) + " " 
				+ std::to_string(msg->screen_pos[1]) + "on map";
	}

	else if(msg->event_type == 1){ //button_click
		if(msg->button_name == "generate"){
			for(int i = 0; i < msg->sequence_string_array.size(); i++)
			{
				col_event += msg->sequence_string_array[i] + " ";
			}
		}
		if(msg->button_name == "rendezvous" || "flocking" || "flock_west" || "flock_east" || "flock_south" || 
								"flock_north" || "antirendezvous")
		{
			col_event = msg->button_name;
		}
		if(msg->button_name == "time_array")
		{
			for(int i = 0; i < msg->time_array.size(); i++)
			{
				col_event = std::to_string(msg->time_array[i]) + " ";
			}
		}

	}
	else if(msg->event_type == 2){
		col_event = "Generated path is ";
		if(msg->is_valid_path) col_event += "valid";
		else col_event += "invalid";
	}

	myfile << "NAME, AIDED VS UNAIDED, TRAIN VS TEST, MAP_NUM, ITER_NUM, EVENT, COST, TIME\n";
	string row = col_name + "," + col_aided + "," + col_train + "," + col_map + "," + col_iter + "," + col_event + ","
				+ col_cost + "," + col_time + "\n";
	myfile << row;
	myfile.close();
	return ;



}



int main(int argc, char** argv)
{

	myfile.open("/home/jaeho-linux/hri2017/src/backend_D/data_storage/backend_D.csv", fstream::app);


	ros::init(argc, argv, "backend_D");
	ros::NodeHandle n;
	ROS_INFO("Starting backend_D node..\n");
	ros::Subscriber d_subscriber = n.subscribe<custom_messages::R2D>("/hsi/R2D", 10000, callBack);

	ros::spin();
	return 0;
}
