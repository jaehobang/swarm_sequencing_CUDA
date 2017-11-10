#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_arr_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  // Set our initial shape type to be a cube
  uint32_t type = visualization_msgs::Marker::CUBE_LIST;

  std::vector<int> ids;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  geometry_msgs::Point point;
  visualization_msgs::MarkerArray marker_arr;

  //publishes the white cubes
  printf("Inside publishMarkerCubes....\n");
  ros::Rate i_Rate(1);
  visualization_msgs::MarkerArray mk_arr;
  visualization_msgs::Marker mk;

  mk.header.stamp = ros::Time::now();
  mk.header.frame_id = "map";
  mk.ns = "coverage_map";
  mk.id = 0;  
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.scale.x = 10;
  mk.scale.y = 10;
  mk.scale.z = 10;
  mk.lifetime = ros::Duration();
  mk.color.r = (float) 1;
  mk.color.g = (float) 1;
  mk.color.b = (float) 1;
  mk.color.a = 1.0;
  int x_offset = -40;
  int y_offset = -40;
  for(int i = 0; i < 40*2; i++)
  {
    for(int j = 0; j < 40*2; j++)
    {
      geometry_msgs::Point p;
      if(i % 2 == 0 )
      {
        //TODO: check that this is correct
        p.y = i - y_offset;
        p.x = j - x_offset;
        p.z = 0;
        mk.points.push_back(p);
      }
    }
  }
  mk_arr.markers.push_back(mk);

  marker_arr_pub.publish(mk_arr);
  ros::spinOnce();
  i_Rate.sleep();

/*
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

			if(ids[robot_i] == 2) return 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = type;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  
    //This is a full 6DOF pose relative to the frame/time specified in the header
      for(int i = 0; i < 400; i++)
      {
        point.x = xs[robot_i];
        point.y = ys[robot_i];
        point.z = 0;
        xs[robot_i]+= 1;
        ys[robot_i]+= 1;
        marker.points.push_back(point);
      }

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;


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


    r.sleep();
  }
*/
}
