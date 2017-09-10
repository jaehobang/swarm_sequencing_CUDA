#include "ros/ros.h"
#include "custom_messages/C2R.h"
#include "custom_messages/R2C.h"
#include "custom_messages/POS.h"

int testmain(int isAided, std::vector<float64> time_array, std::vector<int64> sequence_array, std::vector<uint8_t> isFixed);
RETURN return_struct;
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
  
  


  //testmain();
  int isAided = (int) msg.isAided;
  std::vector<float64> time_array = msg.time_array;
  std::vector<int64> sequence_array = msg.sequence_array;
  std::vector<uint8_t> isFixed = msg.isFixed;

  return_struct = testmain(isAided, time_array, sequence_array, isFixed);  
  


  custom_messages::C2R c2r;
  custom_messages::POS pos;
  pos.x = 1;
  pos.y = 2;
  pos.r = 3;
  c2r.robot_positions.push_back(pos);
  c2r.sequence_length = 1;
  c2r.sequence_end_indices.push_back(10);
  c2r.cost_of_path = 123.4567;
  c2r.is_valid_path = 1;

  c_publisher.publish(c2r);


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "backend_C");
  ros::NodeHandle n;
  c_publisher = n.advertise<custom_messages::C2R>("/hsi/C2R", 1000);
  ros::Subscriber c_subscriber = n.subscribe<custom_messages::R2C>("/hsi/R2C", 1000, callBack);


  ros::spin();
  //testmain();
  return 0;
}
