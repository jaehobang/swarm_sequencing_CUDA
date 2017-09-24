#ifndef FULLPANEL_H
#define FULLPANEL_H

#include <QtWidgets>
#include "ros/ros.h"
#include <string>

#include "visualization_msgs/MarkerArray.h"


using namespace std;

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}


class FullPanel: public QWidget
{
Q_OBJECT
public:
  FullPanel( QWidget* parent = 0);
  virtual ~FullPanel();
  void setHandle(ros::NodeHandle n);
  void publishMessage();
  void deleteMessage();
  void tmpPublish();

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* trajectory_;
  
  ros::NodeHandle n;
  ros::Publisher marker_arr_pub;
  ros::Publisher marker_pub;
  std::vector<visualization_msgs::MarkerArray> mk_arrs;

};

#endif






