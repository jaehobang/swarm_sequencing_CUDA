#include "full_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

FullPanel::FullPanel( QWidget* parent) : QWidget(parent)
{
}

FullPanel::~FullPanel()
{
}

void FullPanel::setHandle(ros::NodeHandle n)
{
  this->n = n;
  marker_arr_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("foo", 1);

}


void FullPanel::deleteMessage()
{
  printf("Inside deleteMEssages!!!\n");
  ros::Rate r(1);
  printf("mk_arrs.size() = %d\n", mk_arrs.size());

  visualization_msgs::MarkerArray mk_arr;

  for(int i = 0; i < mk_arrs.size(); i++){
    mk_arr = mk_arrs[i];
    for(int robot_i = 0;  robot_i < 10; robot_i++)
    {
      mk_arr.markers[robot_i].action = visualization_msgs::Marker::DELETEALL;
		}
    marker_arr_pub.publish(mk_arr);

    ros::spinOnce();
    r.sleep();
  }
  mk_arrs.clear();
  printf("end of delete message\n");
  return;
}


void FullPanel::publishMessage()
{
  ros::Rate r(1);
  printf("Inside publish message\n");
    // Set our initial shape type to be a cube
  uint32_t type = visualization_msgs::Marker::LINE_STRIP;

  std::vector<int> ids;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  geometry_msgs::Point point;
  visualization_msgs::MarkerArray marker_arr;


  while (ros::ok())
  {
    for(int robot_i = 0;  robot_i < 10; robot_i++)
    {
      xs.push_back(robot_i);
      ys.push_back(0);
      zs.push_back(0);
      ids.push_back(0);

      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "robot" + std::to_string(robot_i);
      marker.id = ids[robot_i];
      ids[robot_i]++;

      if(ids[robot_i] == 5) return;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = type;
			marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  
    //This is a full 6DOF pose relative to the frame/time specified in the header
      for(int i = 0; i < 2000; i++)
      {
        point.x = xs[robot_i];
        point.y = ys[robot_i];
        point.z = 0;
        xs[robot_i]+= 0.01;
        ys[robot_i]+= 0.01;
        marker.points.push_back(point);
      }

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.10;
      marker.scale.y = 0.10;
      marker.scale.z = 0.10;


      if(ids[0] == 0){

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      }
      else if(ids[0] == 1){
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      }
      else if(ids[0] == 2){
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      }
      else{
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      }


      marker.lifetime = ros::Duration();
      marker_arr.markers.push_back(marker);
		}
    marker_arr_pub.publish(marker_arr);
    mk_arrs.push_back(marker_arr);

    ros::spinOnce();
    r.sleep();
  }

  


  return;
}


void FullPanel::tmpPublish()
{
  ros::Rate i(1);
  visualization_msgs::Marker mk;
  int count = 0;
  while(ros::ok()){
  if(count == 5) break;
  printf("fml\n");
  mk.header.stamp = ros::Time::now();
  mk.header.frame_id = "map";
  mk.ns = "hello";
  mk.id = 0;
  mk.type = 1;
  mk.action = 0;
  mk.pose.position.x = 0;
  mk.pose.position.y = 0;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = 0;
  mk.pose.orientation.y = 0;
  mk.pose.orientation.z = 0;
  mk.pose.orientation.w = 1;
  mk.scale.x = 10;
  mk.scale.y = 10;
  mk.scale.z = 10;
  mk.color.r = (float)128 / 255;
  mk.color.g = (float) 128 / 255;
  mk.color.b = (float) 128 / 255;
  mk.color.a = 1;
  mk.lifetime = ros::Duration();

  marker_pub.publish(mk);
  ros::spinOnce();
  count++;
  i.sleep(); 
  }
  return;
}



