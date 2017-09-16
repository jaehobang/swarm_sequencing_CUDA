#ifndef MYVIZ_H
#define MYVIZ_H

#include <QWidget>
#include "ros/ros.h"
#include <string>

#include "custom_messages/POS.h"
#include "custom_messages/R2C.h"
#include "custom_messages/R2D.h"
#include "custom_messages/C2R.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

using namespace std;

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
Q_OBJECT
public:
  MyViz( QWidget* parent = 0);
  virtual ~MyViz();
  void callBack(const custom_messages::C2R::ConstPtr& msg);
  void setNodeHandle(ros::NodeHandle nn);

private Q_SLOTS:
  //void setThickness( int thickness_percent );
  //void setCellSize( int cell_size_percent );
  void sendR2C();


private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* trajectory_;
  ros::NodeHandle n;
  ros::Publisher i_publisher;
  ros::Publisher ic_publisher;
  ros::Publisher id_publisher;
  ros::Subscriber i_subscriber;
  std::vector<string> behavior_array = { "rendezvous", "flocking", "flock_east", "flock_north", "flock_west", "flock_south", "antirendezvous" };
  geometry_msgs::PoseArray pa;
};
// END_TUTORIAL
#endif // MYVIZ_H

