#include <QtWidgets>
#include "timerwidget.h"
#include "sequencewidget.h"
#include "consolewidget.h"
#include "consolewidget1.h"
#include "myviz.h"


#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"


MyViz::MyViz( QWidget* parent) : QWidget( parent )
{
  //Deal with Ros related stuff first
  ROS_INFO("starting interface_node...\n");
  ic_publisher = n.advertise<custom_messages::R2C>("/hsi/R2C", 1000);
  id_publisher = n.advertise<custom_messages::R2D>("/hsi/R2D", 1000);
  i_publisher = n.advertise<geometry_msgs::PoseArray>("/hsi/interface", 1000);
  i_subscriber = n.subscribe<custom_messages::C2R>("/hsi/C2R", 1000, &MyViz::callBack, this);
  pa.header.seq = 0;
  pa.header.frame_id = "map";


  tw = new TimerWidget(this);
  sw = new SequenceWidget(this);
  stw = new SwitchTimeWidget(this);
  cw = new ConsoleWidget1(this);

  gb = new QPushButton(QApplication::translate("childwidget", "Generate"), this);
  nb = new QPushButton(QApplication::translate("childwidget", "Next"), this); 

  render_panel_ = new rviz::RenderPanel();

  tw->setFixedSize(QSize(400, 50));
  sw->setFixedSize(QSize(400, 400));
  stw->setFixedSize(QSize(400, 70));
  cw->setFixedSize(QSize(800, 120));


  connect(tw, SIGNAL(signalDone()), this, SLOT(timerDone()));

  QVBoxLayout* col1 = new QVBoxLayout();
  col1->addWidget(render_panel_);
  col1->addWidget(cw);
  
  QHBoxLayout* rowTmp = new QHBoxLayout();
  rowTmp->addWidget(gb);
  rowTmp->addWidget(nb);

  QVBoxLayout* col2 = new QVBoxLayout();
  col2->addWidget(tw);
  col2->addWidget(sw);
  col2->addWidget(stw);
  col2->addLayout(rowTmp);

  QHBoxLayout* row = new QHBoxLayout();
  row->addLayout(col1);
  row->addLayout(col2);
  this->setLayout(row);


  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();

  trajectory_ = manager_->createDisplay( "rviz/PoseArray", "Trajectory Topic", true );
  ROS_ASSERT( trajectory_ != NULL );

  trajectory_->subProp("Topic")->setValue("/hsi/interface");


  ROS_INFO("Reached the end of myViz constructor");

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

void MyViz::timerDone()
{
  //TODO:
  //1. Make a popup screen that says Timer has reached zero and will be moving on to the next map
  //2. When user closes that window call this->next();
  return;
}



void MyViz::generate()
{  
   ROS_INFO("Inside function generate()...sending R2C message\n");
   custom_messages::R2C r2c;
   r2c.stamp = ros::Time::now();
   r2c.isAided = 1; //0 is unaided
   r2c.time_array.push_back(5); //
   r2c.time_array.push_back(10);
   r2c.time_array.push_back(20);
   r2c.sequence_array.push_back(0);
   r2c.sequence_array.push_back(1);
   r2c.sequence_array.push_back(2);
   r2c.sequence_array.push_back(3);
   r2c.isFixed.push_back(1);
   r2c.isFixed.push_back(1);
   r2c.isFixed.push_back(1);

   ic_publisher.publish(r2c);
   ros::spinOnce();

   return;
}

void MyViz::next()
{
    //Reset everything for every component and update the map to the next one
    tw->reset();
		sw->reset();
		stw->reset();
		cw->reset();
		
		//TODO: update map and also reset the trajectory (RVIZ)
		return;
}

void MyViz::setNodeHandle(ros::NodeHandle nn)
{
  n = nn;
}

void MyViz::callBack(const custom_messages::C2R::ConstPtr& msg)
{

  ROS_INFO("Callback function inside interface called!\n");
  //TODO:
  //1. First parse the information and turn it into something that the RVIZ can take in
  //2. Draw on RVIZ - each corresponding section needs to have drawn in different color
  //3. Send the information to backend_D - done

  //Update the rviz Display Panel.... Need some researching to do....
  ROS_INFO("backend_C_node callback called...\n");
  //1. Parse and Convert
  std::vector<custom_messages::POS> robot_positions = msg->robot_positions;
  int sequence_length = msg->sequence_length;
  std::vector<int> sequence_end_indices = msg->sequence_end_indices;
  float cost_of_path = msg->cost_of_path;
  int is_valid_path = (int) msg->is_valid_path;
  std::vector<int> sequence_names = msg->sequence_names;  
  pa.header.seq = 0;

  //2. Visualize
  ros::Rate i_rate(10);
  

  for(int i = 0; i < robot_positions.size(); i++)
  {
    pa.header.seq += 1;
    pa.header.stamp = ros::Time::now();

    geometry_msgs::Pose p;
    for(int j = 0 ; j < 16; j++)
    {
      // TODO: Currently the Pose doesn't have a way to represent orientation visually
      // For now, I will not be inserting the quaternion information - remember it is in radians
      p.position.x = robot_positions[i].x[j];
      p.position.y = robot_positions[i].y[j];
      p.position.z = 0;
      pa.poses.push_back(p);
    }
    i_publisher.publish(pa); 
    ros::spinOnce();
    i_rate.sleep();
  }

  //3. Send to D
  custom_messages::R2D r2d;
  r2d.stamp = ros::Time::now();
  r2d.event_type = EVENT_TRAJ; //2 = trajectory information
  r2d.cost_of_path = cost_of_path;
  r2d.is_valid_path = is_valid_path;
  r2d.is_train = 1;
  r2d.map_number = 20;
  r2d.name = "Tom";
  for(int i = 0; i < sequence_names.size(); i++)
  {
    r2d.behavior_sequences.push_back(behavior_array[sequence_names[i]]);
  }
  id_publisher.publish(r2d);

  return;

}

