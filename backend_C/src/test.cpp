#include "ros/ros.h"
#include "custom_messages/R2C.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "R2C_test");
  ROS_INFO("Starting backend_C test..\n");
  ros::NodeHandle n;
  ros::Publisher test = n.advertise<custom_messages::R2C>("/hsi/R2C", 1000);
  ros::Rate loop_rate(1);

  int count = 0;
  while(ros::ok()){
    custom_messages::R2C r2c;
    r2c.stamp = ros::Time::now();
    r2c.isAided = 1; //0 is unaided
    r2c.time_array.push_back(5); //
    r2c.time_array.push_back(10);
    r2c.time_array.push_back(20);
    r2c.sequence_array.push_back(0);
    r2c.sequence_array.push_back(1);
    r2c.sequence_array.push_back(2);
    r2c.isFixed.push_back(1);
    r2c.isFixed.push_back(1);
    r2c.isFixed.push_back(1);

    test.publish(r2c);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
    if(count == 2) break;
    
  }
  return 0;
}



