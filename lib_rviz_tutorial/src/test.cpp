

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "librviz_tutorial_test");

  ros::NodeHandle n;

  ros::Publisher p = n.advertise<geometry_msgs::PoseStamped>("/hello/world", 1000);
  ros::Rate loop_rate(1);
  geometry_msgs::PoseStamped pa;
  pa.header.seq = 0;
  pa.header.frame_id = "/map";
  pa.pose.position.x = 0;
  pa.pose.position.y = 0;

  while(ros::ok())
  {
     pa.header.seq += 1;
     pa.header.stamp = ros::Time::now();
     pa.pose.position.x += 1;
     pa.pose.position.y += 1; 
     



     p.publish(pa);
     ros::spinOnce();
     loop_rate.sleep();
   }




  return 0;




}
