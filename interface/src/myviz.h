#ifndef MYVIZ_H
#define MYVIZ_H

#include <QtWidgets>
#include "ros/ros.h"
#include <string>

#include "timerwidget.h"
#include "sequencewidget.h"
#include "switchtimewidget.h"
#include "consolewidget.h"
#include "consolewidget1.h"
#include "popupwidget.h"

#include "custom_messages/POS.h"
#include "custom_messages/R2C.h"
#include "custom_messages/R2D.h"
#include "custom_messages/C2R.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"



#define EVENT_SCREEN 0
#define EVENT_BUTTON 1
#define EVENT_TRAJ 2

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
  void setNameAndAided(string n, uint8_t isAided);

private Q_SLOTS:
  void generate();
  void next();
  void timerDone();

private:
  TimerWidget* tw;
  SequenceWidget* sw;
  SwitchTimeWidget* stw;
  ConsoleWidget1* cw;
  QPushButton* gb;
  QPushButton* nb;

  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* trajectory_;

  ros::NodeHandle n;
  ros::Publisher i_publisher;
  ros::Publisher ic_publisher;
  ros::Publisher id_publisher;
  ros::Subscriber i_subscriber;

  std::vector<string> behavior_array = { "rendezvous", "flocking", "flock_east", "flock_north", "flock_west", "flock_south", "antirendezvous" };
  std::vector<string> behavior_array_short = {"r", "i", "e", "n", "w", "s", "a"};
  std::vector<string> color_array = {"dark cyan", "cyan", "gray", "green", "light gray", "magneta", "dark magneta"};
  string name;
  uint8_t isAided;
  QString curr_sequence;
  QString curr_switchtime;
  QString curr_cost;
  QString curr_valid;


  std::vector<int> sequenceInputConvert(QString sequence);
  std::vector<float> switchtimeInputConvert(QString switchtime);
  int iterationConvert(QString iteration);
};

// END_TUTORIAL
#endif // MYVIZ_H

