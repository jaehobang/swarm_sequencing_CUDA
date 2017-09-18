#include <QtWidgets>
#include "timerwidget.h"
#include "sequencewidget.h"
#include "consolewidget.h"
#include "consolewidget1.h"
#include "popupwidget.h"
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


  this->name = "";
  this->isAided = 0;

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
  connect(gb, SIGNAL(released()), this, SLOT(generate()));
  connect(nb, SIGNAL(released()), this, SLOT(next()));


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

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

void MyViz::setNameAndAided(string n, uint8_t isAided)
{
  this->name = n;
  this->isAided = isAided;
  return;
}


void MyViz::timerDone()
{
  //1. Make a popup screen that says Timer has reached zero and will be moving on to the next map
  QWidget* popup = new QWidget();
  popup->setAttribute(Qt::WA_DeleteOnClose);
  QLabel* lab = new QLabel("Your time for this map has ended!!\nLet's move on to the next one!");
  QPushButton* but = new QPushButton("Close");

  connect(but, SIGNAL(released()), popup, SLOT(close()));

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(lab);
  layout->addWidget(but);

  popup->setLayout(layout);
  popup->show();

	//2. Call next function
  this->next();
  return;
}

/* This function will return vector with -1 as the first element
   if input is not formatted correctly */
std::vector<int> MyViz::sequenceInputConvert(QString sequence)
{
   std::vector<int> b_array;
   QRegExp rx("[,]");// match a comma or a space
   QStringList slist = sequence.split(rx, QString::SkipEmptyParts);

   if(slist.size() > 10){
     printf("Too many sequences given!!\n");
     b_array.push_back(-1);
     return b_array;
   }

   for(int i = 0; i < slist.size(); i++)
   {
      string s = slist.at(i).toStdString();
      int pos = behavior_array_short.size();
      for(int j = 0; j < behavior_array_short.size(); j++)
			{
				if(behavior_array_short[j] == s) pos = j;
      }
      if(pos >= behavior_array_short.size())
			{
				printf("Wrong sequence input!!\n");
			  b_array.push_back(-1);
				return b_array;
			}
			b_array.push_back(pos);
   }

   return b_array;

}


std::vector<float> MyViz::switchtimeInputConvert(QString switchtime)
{
   std::vector<float> s_array;
   QRegExp rx("[,]");// match a comma or a space
   QStringList slist = switchtime.split(rx, QString::SkipEmptyParts);
   bool correct_format;

   if(slist.size() > 9){
     printf("Too many switchtimes given!!\n");
     s_array.push_back(-1);
     return s_array;
   }

   for(int i = 0; i < slist.size(); i++)
   {
      float s = slist.at(i).toFloat(&correct_format);
			if(correct_format == 0){
				printf("Wrong switchtime input!!\n");
				s_array.push_back(-1);
				return s_array;
			}
			s_array.push_back(s);
   }

	 return s_array;
}


int MyViz::iterationConvert(QString iteration)
{
  bool ok;
  int i = iteration.toInt();
  return i+1;
}

void MyViz::generateAided()
{
	 QString sequence = sw->getInfo();
   QString switchtime = stw->getInfo();
   QString iteration = cw->getInfo();
	 
   this->curr_sequence = sequence;
   this->curr_switchtime = switchtime;

   //2. Running the input checker
	std::vector<float> switchtime_arr = this->switchtimeInputConvert(switchtime);
   int iter = this->iterationConvert(iteration);

   //3. if invalid
	
   if(switchtime_arr[0] == -1){
     //Generate a popup message and return without generating
     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QLabel* lab = new QLabel("Switch time you generated is invalid!!\nPlease try again!");
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
     layout->addWidget(lab);
     layout->addWidget(but);

     popup->setLayout(layout);
     popup->show();
		 return;
   }

   if(iter == -1){
     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QLabel* lab = new QLabel("Iteration is invalid!!\nPlease try again!");
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
     layout->addWidget(lab);
     layout->addWidget(but);

     popup->setLayout(layout);
     popup->show();
		 return;
   }



}



void MyViz::generateUnaided()
{


}


void MyViz::generate()
{  
   /* TODO:
			1. Collect Data from all the widgets 
         (make sure to update the console first)
			2. Run them through Input Checker
			3. If not valid input, generate a popup message
			4. If valid, create r2c message
			5. Publish the r2c message
			6. create a r2d message
			7. publish the r2d message
	 */
   //1. Collecting all the data
   std::vector<int> sequence_arr;
   std::vector<float> switchtime_arr;
   QString sequence;
   QString switchtime;
   QString iteration;
   int iter;

   if(this->isAided == 0)
   {
     sequence = sw->getInfo();
     this->curr_sequence = sequence;
 	   sequence_arr = this->sequenceInputConvert(sequence);
     //3. if invalid
  	 if(sequence_arr[0] == -1){
	     //Generate a popup message and return without generating
       QWidget* popup = new QWidget();
       popup->setAttribute(Qt::WA_DeleteOnClose);
       QLabel* lab = new QLabel("Sequence you generated is invalid!!\nPlease try again!");
       QPushButton* but = new QPushButton("Close");

       connect(but, SIGNAL(released()), popup, SLOT(close()));

       QVBoxLayout* layout = new QVBoxLayout();
       layout->addWidget(lab);
       layout->addWidget(but);

       popup->setLayout(layout);
       popup->show();
		   return;
     }
   }

   switchtime = stw->getInfo();
   iteration = cw->getInfo(); 	 
   this->curr_switchtime = switchtime;

   //2. Running the input checker
   switchtime_arr = this->switchtimeInputConvert(switchtime);
   iter = this->iterationConvert(iteration);

   if(switchtime_arr[0] == -1){
     //Generate a popup message and return without generating
     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QLabel* lab = new QLabel("Switch time you generated is invalid!!\nPlease try again!");
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
     layout->addWidget(lab);
     layout->addWidget(but);

     popup->setLayout(layout);
     popup->show();
		 return;
   }

   if(iter == -1){
     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QLabel* lab = new QLabel("Iteration is invalid!!\nPlease try again!");
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
     layout->addWidget(lab);
     layout->addWidget(but);

     popup->setLayout(layout);
     popup->show();
		 return;
   }
 
     
   //4. if valid, create and publish r2c
   ROS_INFO("Inside function generate()...sending R2C message\n");
   custom_messages::R2C r2c;
   r2c.stamp = ros::Time::now();
   r2c.isAided = this->isAided; //0 is unaided
   /* Debugging purposes.... */
   printf("Printing the switch times..\n");
   for(int i = 0; i < switchtime_arr.size(); i++)
   {
     printf("%f ", switchtime_arr[i]);
   }
   printf("\n");
   r2c.time_array = switchtime_arr; 
   if(this->isAided == 0) 
	 {
     printf("Printing the sequence times...\n");
     for(int i = 0; i<sequence_arr.size(); i++)
     {
       printf("%d ", sequence_arr[i]);
     }
     printf("\n");
   }
   r2c.sequence_array = sequence_arr; 
   ic_publisher.publish(r2c);
   ros::spinOnce();


   //5. create and publish r2d message
   ROS_INFO("Inside function generate()...sending R2D message\n");
   std::vector<string> b_sequences;
   for(int i = 0; i < sequence_arr.size(); i++)
   {
     b_sequences.push_back(behavior_array[sequence_arr[i]]);
   }
   custom_messages::R2D r2d;
   r2d.stamp = ros::Time::now();
   r2d.event_type = EVENT_BUTTON;
   r2d.button_name = "Generate";
   r2d.switch_times = switchtime_arr;
   r2d.behavior_sequences = b_sequences;
   r2d.name = this->name;
   r2d.is_aided = this->isAided;
   id_publisher.publish(r2d);
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
  r2d.name = this->name;
  for(int i = 0; i < sequence_names.size(); i++)
  {
    r2d.behavior_sequences.push_back(behavior_array[sequence_names[i]]);
  }
  id_publisher.publish(r2d);
  

  //4. Update the Console Widget
  this->curr_cost = QString::number(cost_of_path);
  if(is_valid_path == 1) this->curr_valid = "VALID";
  else this->curr_valid = "INVALID";
  cw->update(this->curr_sequence, this->curr_switchtime, 
						 this->curr_cost, this->curr_valid);


  return;

}

