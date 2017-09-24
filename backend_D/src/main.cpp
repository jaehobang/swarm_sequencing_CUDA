#include "ros/ros.h"
#include "custom_messages/R2D.h"
#include <iostream>
#include <fstream>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"
using namespace std;

#define TIME 0
#define ID 1
#define MAP 2
#define ITER 3
#define DESC 4
#define TARR 5
#define SARR 6 
#define COST 7
#define VAL 8
#define COMP 9
#define COL_NUM 10


//1 session is going to correspond to 1 person
ofstream myfile;




void callBack(const custom_messages::R2D::ConstPtr& msg)
{
  string info_arr[COL_NUM];
  for(int i = 0; i < COL_NUM; i++)
  {
    info_arr[i] = "";
  }

	boost::posix_time::ptime my_posix_time = msg->stamp.toBoost();
	string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
	info_arr[TIME] = iso_time_str;
  info_arr[ID] = msg->id;
  info_arr[MAP] = msg->map_number;
  info_arr[ITER] = msg->iteration;
  info_arr[DESC] = msg->description;
  info_arr[TARR] = msg->switchtime_string;
  info_arr[SARR] = msg->sequence_string;
  info_arr[COST] = msg->cost_of_path;
  
  info_arr[VAL] = msg->is_valid_path;
  info_arr[COMP] = msg->is_complete;
	
  string result = "";
  for(int i = 0; i < COL_NUM; i++)
  {
    result += info_arr[i];
    result += ",";
  }
  result += "\n";

  myfile << "TIME,ID,MAP NUMBER,ITERATION,DESCRIPTION,TIME ARRAY,SEQUENCE ARRAY,COST,VALID,COMPLETE\n";
	myfile << result;
	myfile.close();
	return;
}




int main(int argc, char** argv)
{

	myfile.open("/home/jaeho-linux/hri2017/src/backend_D/data_storage/backend_D.csv", fstream::app);


	ros::init(argc, argv, "backend_D");
	ros::NodeHandle n;
	ROS_INFO("Starting backend_D node..\n");
	ros::Subscriber d_subscriber = n.subscribe<custom_messages::R2D>("/hsi/R2D", 100, callBack);
  ros::spin();
	return 0;
}
