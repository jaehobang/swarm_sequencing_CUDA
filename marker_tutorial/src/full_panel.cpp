#include "full_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

FullPanel::FullPanel( QWidget* parent) : QWidget(parent)
{
  render_panel_ = new rviz::RenderPanel();
  
  QVBoxLayout* col = new QVBoxLayout();
  col->addWidget(render_panel_);
  this->setLayout(col);

  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();

  trajectory_ = manager_->createDisplay( "rviz/MarkerArray", "Marker Array", true);
  ROS_ASSERT (trajectory_ != NULL);

  trajectory_->subProp("Topic")->setValue("visualization_marker_array");
}

FullPanel::~FullPanel()
{
  delete manager_;
}

void FullPanel::setHandle(ros::NodeHandle n)
{
  this->n = n;
}


void FullPanel::publishMessage()
{
  ros::Rate r(1);
  ros::Publisher marker_arr_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

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


    r.sleep();
  }
  return;
}



