#ifndef MYVIZ_H
#define MYVIZ_H

#include <QtWidgets>
#include "ros/ros.h"
#include <string>

#include "timerwidget.h"
//#include "sequencewidget.h"
//#include "switchtimewidget.h"
#include "consolewidget.h"
#include "consolewidget1.h"
#include "popupwidget.h"
#include "timehorizonwidget.h"
#include "sequencetimewidget.h"

#include "custom_messages/R2C.h"
#include "custom_messages/R2D.h"
#include "custom_messages/C2R.h"
#include "visualization_msgs/MarkerArray.h"



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
  void setNameAndAided(string n, uint8_t is_aided);

private Q_SLOTS:
  void generate();
  void next();
  void submit();
  void timerDone();

private:
  TimerWidget* tw;
  //SequenceWidget* sw;
  //SwitchTimeWidget* stw;
  ConsoleWidget1* cw;
  QPushButton* rb;
  QPushButton* nb;
  QPushButton* sb;
  TimeHorizonWidget* thw;
	SequenceTimeWidget* stw;
  QWidget* pw;

  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* trajectory_;

  ros::NodeHandle n;
  ros::Publisher ic_publisher;
  ros::Publisher id_publisher;
  ros::Subscriber ic_subscriber;

  std::vector<string> behavior_array = { "rendezvous", "flocking", "flock_east", "flock_north", "flock_west", "flock_south", "antirendezvous" };
  std::vector<string> behavior_array_short = {"r", "i", "e", "n", "w", "s", "a"};
  std::vector<QString> color_array = {"cyan", "purple", "blue", "brown", "green", "orange", "pink"};
  string name;
  uint8_t is_aided;
  QString curr_sequence;
  QString curr_switchtime;
  QString curr_cost;
  QString curr_valid;
  QString curr_complete;
  int curr_map_number;
  uint8_t eot_processed;


  std::vector<int> sequenceInputConvert(QString sequence);
  std::vector<float> switchtimeInputConvert(QString switchtime);
  int iterationConvert(QString iteration);
  void generateErrorPopup(QString label_val);
};

// END_TUTORIAL
#endif // MYVIZ_H

