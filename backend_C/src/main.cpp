#include "ros/ros.h"
#include "custom_messages/C2R.h"
#include "custom_messages/R2C.h"
#include "custom_messages/POS.h"
#include "smha_headers.hpp"

RETURN testmain(int isAided, std::vector<double> time_array, std::vector<long int> sequence_array, std::vector<uint8_t> isFixed);
//int testmain();
RETURN return_struct; //I hope it knows where to find this......
ros::Publisher c_publisher;
int N = 16;


void callBack(const custom_messages::R2C::ConstPtr& msg)
{ 
  //TODO:
  //1. Retrieve the data in R2C
  //2. Use that data and run testmain
  //3. Package the result from testmain in C2R
  //4. publish that data

  //5. FIRST, I will just see if message sending and recieving works. 
  
  
  ROS_INFO("backend_C_node callback called...\n");
  
  int isAided = (int) msg->isAided;
  std::vector<double> time_array = msg->time_array;
  std::vector<long int> sequence_array = msg->sequence_array;
  std::vector<uint8_t> isFixed = msg->isFixed;


  //ERROR CHECKING!!!!
  if(time_array.size() > 10) {
    ROS_INFO("Too many switch times selected....\n");
    return;
  }
  if(isAided == 0 && sequence_array.size() != time_array.size() + 1)
  {
    ROS_INFO("Number of switch times does not match number of sequence for unaided case....\n");
    return;
  }

  return_struct = testmain(isAided, time_array, sequence_array, isFixed);  
  custom_messages::C2R c2r;
  custom_messages::POS pos;
  
  for(int i = 0; i < return_struct.sequence_end_indices.size(); i++)
  {
    printf("sequence_end_indices for [%d] is %d\n", i,
      return_struct.sequence_end_indices[i]);
  }


  int pos_size = return_struct.sequence_end_indices[return_struct.sequence_end_indices.size() - 1] + 1;

  printf("inside main.cpp printing pos_size %d\n", pos_size);

  for(int i = 0; i < pos_size; i++)
  {
    for(int j = 0; j < N; j++)
    {

      pos.x.push_back(return_struct.robot_positions[i].robot_pos[j][0]);
      pos.y.push_back(return_struct.robot_positions[i].robot_pos[j][1]);
      pos.r.push_back(return_struct.robot_positions[i].robot_pos[j][2]);  
      
    }
    c2r.robot_positions.push_back(pos);
    pos.x.clear();
    pos.y.clear();
    pos.r.clear();
  }
  
  c2r.sequence_length = return_struct.sequence_length;
  c2r.sequence_end_indices = return_struct.sequence_end_indices;
  c2r.cost_of_path = return_struct.cost_of_path;
  c2r.is_valid_path = return_struct.is_valid_path;
  c2r.sequence_names = return_struct.sequence_names;
  
  c_publisher.publish(c2r);
  ROS_INFO("End of callback function!\n");

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "backend_C");
  ros::NodeHandle n;
  ROS_INFO("Starting backend_C node..\n");
  c_publisher = n.advertise<custom_messages::C2R>("/hsi/C2R", 1000);
  ros::Subscriber c_subscriber = n.subscribe<custom_messages::R2C>("/hsi/R2C", 1000, callBack);


  ros::spin();
  //testmain();
  return 0;
}
