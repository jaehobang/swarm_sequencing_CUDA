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
  void callBack2(const custom_messages::C2R::ConstPtr& msg);
  void setNodeHandle(ros::NodeHandle nn);
  void setNameAndAided(string n, int is_aided);

private Q_SLOTS:
  void generate();
  void next();
  void submit();
  void timerDone();
  void checkNext();
  void nextWrapper();
  void createHSIWidget();
  


private:
  TimerWidget* tw;
  //SequenceWidget* sw;
  //SwitchTimeWidget* stw;
  ConsoleWidget1* cw;
  QPushButton* rb;
  QPushButton* nb;
  QPushButton* sb;
  QPushButton* prb;
  TimeHorizonWidget* thw;
	SequenceTimeWidget* stw;
  QWidget* np;
  QWidget* pw;
  QWidget* pp;
  PopupWidget* ip;
  QLabel* vl; //valid label
  QLabel* cl; //complete label
  QLabel* ol; //optimal label
  QLabel* tl;
  QProgressBar* pb;
  

  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* trajectory_;

  ros::NodeHandle n;
  ros::Publisher ic_publisher;
  ros::Publisher id_publisher;
  ros::Subscriber ic_subscriber;
  ros::Subscriber ic2_subscriber;

  std::vector<string> behavior_array = { "Rendezvous", "Antirendezvous", "Flock East", "Flock North", "Flock West", "Flock South", "Line X", "Line Y" };
  std::vector<string> behavior_array_short = {"r", "a", "e", "n", "w", "s", "x", "y"};
  std::vector<QString> color_array = {"cyan", "purple", "gray", "brown", "green", "orange", "pink", "yellow"};
  string name;
  int is_aided;
  uint8_t received_C;
  uint8_t input_error;
  QString curr_sequence;
  QString curr_switchtime;
  QString curr_cost;
  QString curr_valid;
  QString curr_complete;
  QString aided_optimal_sequence;
  QString aided_optimal_cost;
  int curr_map_number;
  


  std::vector<int> sequenceInputConvert(QString sequence);
  std::vector<float> switchtimeInputConvert(QString switchtime);
  int iterationConvert(QString iteration);
  void generateErrorPopup(QString str);
  void generateProcessPopup();
};

// END_TUTORIAL
#endif // MYVIZ_H

