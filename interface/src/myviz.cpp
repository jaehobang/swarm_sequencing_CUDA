#include "myviz.h"

#include <QtDebug>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/yaml_config_reader.h"
#include "rviz/config.h"
#include "rviz/yaml_config_writer.h"
#include "rviz/ogre_helpers/axes.h"

MyViz::MyViz( QWidget* parent) : QWidget( parent )
{
  //Deal with Ros related stuff first
  
  ROS_INFO("starting interface_node...\n");
  ic_publisher = n.advertise<custom_messages::R2C>("/hsi/R2C", 1000);
  id_publisher = n.advertise<custom_messages::R2D>("/hsi/R2D", 1000);
  ic_subscriber = n.subscribe<custom_messages::C2R>("/hsi/C2R", 1000, &MyViz::callBack, this);
  ic2_subscriber = n.subscribe<custom_messages::C2R>("/hsi/C2R2", 1000, &MyViz::callBack2, this);  

  this->name = "";
  this->is_aided = 0;
  this->curr_map_number = 0;
  this->received_C = 1;

  tw = new TimerWidget(this);
 // sw = new SequenceWidget(this);
 // stw = new SwitchTimeWidget(this);
  cw = new ConsoleWidget1(this);
  thw = new TimeHorizonWidget(this);
  stw = new SequenceTimeWidget(this);
  stw->setPublisher(id_publisher);

  rb = new QPushButton(QApplication::translate("childwidget", "Run"), this);
  nb = new QPushButton(QApplication::translate("childwidget", "Next"), this); 
  sb = new QPushButton(QApplication::translate("childwidget", "Submit"), this);
  prb = new QPushButton(QApplication::translate("childwidget", "Pause/Resume"), this);

  vl = new QLabel("Valid");
  cl = new QLabel("Complete");
  ol = new QLabel("Optimal");
  vl->setStyleSheet("QLabel {color : black; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
  cl->setStyleSheet("QLabel {color : black; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
  ol->setStyleSheet("QLabel {color : black; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");


  tl = new QLabel("Test Progress");

  pb = new QProgressBar();
  pb->setTextVisible( false );
  pb->setRange(0,17);
  pb->setValue(curr_map_number);

  compw = new Compare_Widget(0, "Cost");

  render_panel_ = new rviz::RenderPanel();

  tw->setFixedSize(QSize(450, 80));
  stw->setFixedSize(QSize(450, 360));
  cw->setFixedSize(QSize(450, 100));
  thw->setFixedSize(QSize(450, 80));
  pb->setFixedSize(QSize(450, 20));
  compw->setFixedSize(QSize(450, 80));
  //sb->setFixedSize(QSize(220, 100));
  connect(tw, SIGNAL(signalDone()), this, SLOT(timerDone()));
  connect(rb, SIGNAL(released()), this, SLOT(generate()));
  connect(nb, SIGNAL(released()), this, SLOT(checkNext()));
	connect(sb, SIGNAL(released()), this, SLOT(submit()));
  connect(prb, SIGNAL(released()), tw, SLOT(pauseResume()));

  QVBoxLayout* col1 = new QVBoxLayout();
  col1->addWidget(render_panel_);
 
  QHBoxLayout* rowTmp = new QHBoxLayout();
  rowTmp->addWidget(rb);
  rowTmp->addWidget(nb);

  QHBoxLayout* rowTmp2 = new QHBoxLayout();
  QLabel* labelTmp = new QLabel();
  rowTmp2->addWidget(labelTmp);
  rowTmp2->addWidget(sb);

  QHBoxLayout* rowTmp3 = new QHBoxLayout();
  rowTmp3->addWidget(vl);
  rowTmp3->addWidget(cl);
  rowTmp3->addWidget(ol);

  QVBoxLayout* col2 = new QVBoxLayout();
  col2->addWidget(prb);
  col2->addWidget(tw);
  //col2->addWidget(sw);
  col2->addWidget(stw);

  col2->addLayout(rowTmp2);
  col2->addWidget(compw);
  col2->addWidget(thw);
  col2->addLayout(rowTmp3);
  col2->addWidget(cw);
 
  col2->addLayout(rowTmp);
  col2->addWidget(tl);
  col2->addWidget(pb);


  QHBoxLayout* row = new QHBoxLayout();
  row->addLayout(col1);
  row->addLayout(col2);
  this->setLayout(row);

  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();
  

  rviz::Config config;
  
  rviz::YamlConfigReader* reader = new rviz::YamlConfigReader();
  reader->readFile( config, "/home/jaeho-linux/hri2017/src/tmp1.rviz" );
  //Check that config file is loading what I am expecting.....
  printf("config object validity check %d\n", (int) config.isValid());


  manager_->load( config );

  printf("returning from init\n");

  this->ip = new PopupWidget();
  printf("Finished initializing popup...\n");
  this->ip->resize(200,200);
  this->ip->show();
  printf("Finished showing...\n");

  connect(this->ip, SIGNAL(widgetClosed()), this, SLOT(createHSIWidget()));


  rviz::Axes* axes_ = new rviz::Axes(manager_->getSceneManager(), 0, 5.0f, 0.5f);
  Ogre::Vector3* position = new Ogre::Vector3(-38,33,0);
  axes_->setPosition(*position);
/*
  rviz::Axes* axes_1 = new rviz::Axes(manager_->getSceneManager(), 0, 5.0f, 0.1f);
  Ogre::Vector3* position1 = new Ogre::Vector3(2,3,4);
  axes_1->setPosition(*position1);

  rviz::Axes* axes_2 = new rviz::Axes(manager_->getSceneManager(), 0, 5.0f, 0.1f);
  Ogre::Vector3* position2 = new Ogre::Vector3(3,4,5);
  axes_2->setPosition(*position2);
*/

/*
  rviz::Display* axes_ = manager_->createDisplay("rviz/Axes", "xyz", true);
  ROS_ASSERT( axes_ != NULL );

  axes_->subProp("Length")->setValue(5);
  axes_->subProp("Radius")->setValue(0.1);
  Ogre::Vector3 pos = new Ogre::Vector3(1,2,3);
  axes_->subProp("Position")->setValue(pos);

  // Try to set the location elsewhere
  qDebug("%s", axes_->getName());
*/
/*
  trajectory_ = manager_->createDisplay( "rviz/MarkerArray", "Marker Array", true );
  ROS_ASSERT( trajectory_ != NULL );

  trajectory_->subProp("Marker Topic")->setValue("visualization_marker_array");
  trajectory_->subProp("Queue Size")->setValue(1); 

	rviz::Display* map_ = manager_->createDisplay( "rviz/Marker", "Marker", true);
  ROS_ASSERT( map_ != NULL );

  map_->subProp("Marker Topic")->setValue("visualization_marker");
  map_->subProp("Queue Size")->setValue(10);

  //rviz::Display* grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true);
  //ROS_ASSERT( grid_ != NULL);
  //grid_->subProp("Plane Cell Count")->setValue(40);
  //grid_->subProp("Line Style")->setValue("Lines");
  //grid_->subProp("Plane")->setValue("XY");

  rviz::Display* axes_ = manager_->createDisplay( "rviz/Axes", "xyz", true);
  ROS_ASSERT( axes_ != NULL);

  axes_->subProp("Length")->setValue(5);
  axes_->subProp("Radius")->setValue(0.1);
*/
/*
  rviz::YamlConfigWriter* writer = new rviz::YamlConfigWriter();
  manager_->save(config);
  writer->writeFile(config, "/home/jaeho-linux/hri2017/tmp1.rviz");
  if(writer->error()) {qInfo() << writer->errorMessage();}  
*/
  

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}


void MyViz::createHSIWidget()
{
  string name = this->ip->getName().toStdString();
  int isAided = this->ip->getIsAided();
  this->setNameAndAided(name, isAided);
  this->resize(1400, 600);
  this->show();
  return;
}


void MyViz::setNameAndAided(string n, int is_aided)
{
  this->name = n;
  this->is_aided = is_aided;
  return;
}


void MyViz::timerDone()
{
  //1. Make a popup screen that says Timer has reached zero and will be moving on to the next map
  QWidget* popup = new QWidget();
  popup->setAttribute(Qt::WA_DeleteOnClose);
  QRect rec = QApplication::desktop()->screenGeometry();
  int height = rec.height();
  int width = rec.width();
  popup->move(width / 2, height / 2);
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
   if(sequence == "") return b_array;

   QRegExp rx("[,]");// match a comma or a space
   QStringList slist = sequence.split(rx, QString::SkipEmptyParts);

   for(int i = 0; i < slist.size(); i++)
   {
      string s = slist.at(i).toStdString();
      int index = std::find(behavior_array_short.begin(), 
				behavior_array_short.end(), s) - behavior_array_short.begin();
			b_array.push_back(index);
   }

   return b_array;

}


std::vector<float> MyViz::switchtimeInputConvert(QString switchtime)
{
   std::vector<float> s_array;
   if(switchtime == "") return s_array;
   QRegExp rx("[,]");// match a comma or a space
   QStringList slist = switchtime.split(rx, QString::SkipEmptyParts);
   bool correct_format;

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


void MyViz::submit()
{
	//TODO: retrieve the behaviors and times given and update the
  //time horizon
   std::vector<int> sequence_arr;
   std::vector<float> switchtime_arr;
   QString sequence;
   QString switchtime;
 
   ROS_INFO("Inside submit()");

   sequence = stw->getSequence();
   qInfo() << "Sequence is" << stw->getSequence();
  
   if(sequence == "-5"){
     QString str = QString("ERROR!\nMisspelled behavior in Sequence!!");
     this->generateErrorPopup(str);

     custom_messages::R2D r2d;
  	 r2d.stamp = ros::Time::now();
   	 r2d.id = this->name;
  	 r2d.map_number = "";
  	 r2d.iteration = "";
  	 r2d.event_type = EVENT_BUTTON;
  	 r2d.description = "Submit Clicked, but misspelled behavior in sequence";
  	 r2d.switchtime_string = "";
  	 r2d.sequence_string = "";
  	 id_publisher.publish(r2d);
  	 ros::spinOnce();
     this->input_error = 1;
		 return;
   }


    //ROS_INFO("sequence is %s", sequence.toStdString().c_str());
   this->curr_sequence = sequence;
 	 sequence_arr = this->sequenceInputConvert(sequence);
     //3. if invalid
     //Generate a popup message and return without generating
    
  
   switchtime = stw->getSwitchTime();
   this->curr_switchtime = switchtime;
   qInfo() << this->curr_switchtime;

   
  //Generate a popup message and return without generating
	 if(switchtime == "-1" || switchtime == "-2" ){
   
     QString lab_text;
     if(switchtime == "-1") lab_text = QString("ERROR!\nInput is not a int or floating point number.");
     else lab_text = QString("ERROR!\nTotal time input exceeded maximum simulation time (60 seconds)!");
     this->generateErrorPopup(lab_text);

	   custom_messages::R2D r2d;
	   r2d.stamp = ros::Time::now();
	   r2d.id = this->name;
	   r2d.map_number = "";
	   r2d.iteration = (cw->getInfo()).toStdString();
	   r2d.event_type = EVENT_BUTTON;
	   r2d.description = "Submit clicked! But input time exceeded max sim time";
	   r2d.switchtime_string = this->curr_switchtime.toStdString();
	   r2d.sequence_string = this->curr_sequence.toStdString();
	   id_publisher.publish(r2d);
	   ros::spinOnce();

     this->input_error = 1;
     return;
   }   

   //2. Running the input checker
   switchtime_arr = this->switchtimeInputConvert(switchtime);

   if(sequence_arr.size() > switchtime_arr.size()){
     printf("sequence size = %d, switchtime size = %d\n");

     QString lab_text = QString("ERROR\nMore behaviors given than durations!!");
     this->generateErrorPopup(lab_text);

     custom_messages::R2D r2d;
     r2d.stamp = ros::Time::now();
	   r2d.id = this->name;
  	 r2d.map_number = "";
  	 r2d.iteration = "";
  	 r2d.event_type = EVENT_BUTTON;
  	 r2d.description = "Submit Clicked! More behaviors given than durations";
  	 r2d.switchtime_string = "";
  	 r2d.sequence_string = "";
  	 id_publisher.publish(r2d);
  	 ros::spinOnce();

     this->input_error = 1;
   	 return;

   }
   
   if(is_aided == 0 && sequence_arr.size() != switchtime_arr.size()){
     QString lab_text = QString("ERROR\nNumber of Behaviors must match number of durations!!");
     this->generateErrorPopup(lab_text);

	   custom_messages::R2D r2d;
 	   r2d.stamp = ros::Time::now();
   	 r2d.id = this->name;
  	 r2d.map_number = "";
  	 r2d.iteration = "";
  	 r2d.event_type = EVENT_BUTTON;
  	 r2d.description = "Submit Clicked! Number of behaviors does not match durations";
  	 r2d.switchtime_string = "";
  	 r2d.sequence_string = "";
  	 id_publisher.publish(r2d);
  	 ros::spinOnce();
     this->input_error = 1;
    return;

   }

   if(is_aided == 2 && sequence_arr.size() != 0){
     QString lab_text = QString("ERROR\nMust not fill up Sequence column!!");
     this->generateErrorPopup(lab_text);

     custom_messages::R2D r2d;
     r2d.stamp = ros::Time::now();
     r2d.id = this->name;
     r2d.map_number = "";
     r2d.iteration = "";
     r2d.event_type = EVENT_BUTTON;
     r2d.description = "Submit Clicked! Behaviors must not be filled";
     r2d.switchtime_string = "";
     r2d.sequence_string = "";
     id_publisher.publish(r2d);
     ros::spinOnce();
     this->input_error = 1;
    return;

   }


   if(switchtime_arr.size() == 0)
   {
     QString lab_text = QString("ERROR\nNo Switchtimes given!");
     this->generateErrorPopup(lab_text);


	   custom_messages::R2D r2d;
 		 r2d.stamp = ros::Time::now();
 	 	 r2d.id = this->name;
 	   r2d.map_number = "";
 	 	 r2d.iteration = "";
 	 	 r2d.event_type = EVENT_BUTTON;
 	 	 r2d.description = "Submit Clicked! No switchtimes were given";
 	 	 r2d.switchtime_string = "";
 	 	 r2d.sequence_string = "";
 	 	 id_publisher.publish(r2d);
 	 	 ros::spinOnce();
     this->input_error = 1;

     return;
   }


	 //Debugging....
	 ROS_INFO("switchtime_arr.size() = %d, sequence_arr.size() = %d", switchtime_arr.size(), sequence_arr.size());

   //change switchtime_arr to std::vector<int> 10x times needed for each value
   //fill up std::vector<QColor> with colors that correspond to sequence_arr indices
   std::vector<int> switchtime_timehorizon;
   
   for(int i = 0; i < switchtime_arr.size(); i++)
	 {
     switchtime_timehorizon.push_back( (int) switchtime_arr[i] * 10 );
   }

   std::vector<QString> color_timehorizon;
   for(int i = 0; i < sequence_arr.size(); i++)
   {
     
		QString color = this->color_array[sequence_arr[i]];
		color_timehorizon.push_back( color );
   }
   
	 int offset = 0;
   while(sequence_arr.size() + offset != switchtime_arr.size()){
     color_timehorizon.push_back(QString("black"));
     offset++;
   }

	 this->thw->setValues(switchtime_timehorizon, color_timehorizon);
   
   custom_messages::R2D r2d;
   r2d.stamp = ros::Time::now();
   r2d.id = this->name;
   r2d.map_number = "";
   r2d.iteration = cw->getInfo().toStdString();
   r2d.event_type = EVENT_BUTTON;
   r2d.description = "Submit Clicked";
   r2d.switchtime_string = this->curr_switchtime.toStdString();
   r2d.sequence_string = this->curr_sequence.toStdString();
   id_publisher.publish(r2d);
   ros::spinOnce();


	  
	 return;

}


void MyViz::generateErrorPopup(QString str)
{
  QWidget* popup = new QWidget();
  popup->setAttribute(Qt::WA_DeleteOnClose);
  QRect rec = QApplication::desktop()->screenGeometry();
  int height = rec.height();
  int width = rec.width();
  printf("height, width of application desktop is %d %d\n", height, width);
  popup->move(width / 2, height / 2);
  QLabel* lab = new QLabel(str);
  QPushButton* but = new QPushButton("Close");

  QObject::connect(but, SIGNAL(released()), popup, SLOT(close()));

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(lab);
  layout->addWidget(but);

  popup->setLayout(layout);
  popup->show();
  return;
}

void MyViz::generateProcessPopup()
{
  pp = new QWidget();
  QString str = QString("The system is processing request, please wait....");
  pp->setAttribute(Qt::WA_DeleteOnClose);
  QRect rec = QApplication::desktop()->screenGeometry();
  int height = rec.height();
  int width = rec.width();
  printf("height, width of application desktop is %d %d\n", height, width);
  pp->move(width / 2, height / 2);
  QLabel* lab = new QLabel(str);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(lab);

  pp->setLayout(layout);
  pp->show();
  return;
}



void MyViz::generate()
{  
   this->submit(); //Run this in case sequence changed after user pressed submit
 
   if(this->input_error)
   {
     this->input_error = 0;
     return;
   }

   //1. Collecting all the data
   std::vector<int> sequence_arr;
   std::vector<float> switchtime_arr;
   QString iteration;
   int iter;

   
 	 sequence_arr = this->sequenceInputConvert(this->curr_sequence);
   
   iteration = cw->getInfo(); 	 
     
   //2. Running the input checker
   switchtime_arr = this->switchtimeInputConvert(this->curr_switchtime);
   iter = this->iterationConvert(iteration);
   //4. if valid, create and publish r2c
   ROS_INFO("Inside function generate()...sending R2C message\n");
   //if(this->received_C == 0) return; //TODO: enable this function again later
   custom_messages::R2C r2c;
   r2c.stamp = ros::Time::now();
   r2c.is_aided = this->is_aided; //0 is unaided
   /* Debugging purposes.... */
   printf("Printing the switch times..\n");
   for(int i = 0; i < switchtime_arr.size(); i++)
   {
     printf("%f ", switchtime_arr[i]);
   }
   printf("\n");
   r2c.time_array = switchtime_arr; 
   if(this->is_aided == 0) 
	 {
     printf("Printing the sequence times...\n");
     for(int i = 0; i<sequence_arr.size(); i++)
     {
       printf("%d ", sequence_arr[i]);
     }
     printf("\n");
   }
   r2c.sequence_int_array = sequence_arr; 
   ic_publisher.publish(r2c);
   ros::spinOnce();
   this->received_C = 0;


   //5. create and publish r2d message
   ROS_INFO("Inside function generate()...sending R2D message\n");
   custom_messages::R2D r2d;
   r2d.stamp = ros::Time::now();
   r2d.id = this->name;
   r2d.map_number = "";
   r2d.iteration = cw->getInfo().toStdString();
   r2d.event_type = EVENT_BUTTON;
   r2d.description = "Run Clicked";
   r2d.switchtime_string = this->curr_switchtime.toStdString();
   r2d.sequence_string = this->curr_sequence.toStdString();
   id_publisher.publish(r2d);
   ros::spinOnce();

   ROS_INFO("Done with generate()");
   QApplication::setOverrideCursor(Qt::WaitCursor);
   
   this->generateProcessPopup();
   
   return;
}


void MyViz::checkNext()
{
   np = new QWidget();
   np->setAttribute(Qt::WA_DeleteOnClose);
   QRect rec = QApplication::desktop()->screenGeometry();
   int height = rec.height();
   int width = rec.width();
   np->move(width / 2, height / 2);

   QLabel* lab = new QLabel("Are you sure you want to move to the next map?");
   QPushButton* yes_but = new QPushButton("Yes");
   QPushButton* no_but = new QPushButton("No");

   connect(yes_but, SIGNAL(released()), this, SLOT(nextWrapper()));
   connect(no_but, SIGNAL(released()), np, SLOT(close()));

	 QHBoxLayout* layouth = new QHBoxLayout();
   layouth->addWidget(yes_but);
   layouth->addWidget(no_but);

   QVBoxLayout* layout = new QVBoxLayout();
   layout->addWidget(lab);
   layout->addLayout(layouth);

   np->setLayout(layout);
   np->show();
  
}

void MyViz::nextWrapper()
{
    np->close();
    this->next();
    return;
}


void MyViz::next()
{
    //Reset everything for every component and update the map to the next one
    tw->reset(240);
    stw->reset();
    cw->reset();
    thw->reset();
    vl->setStyleSheet("QLabel {color : black; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
    cl->setStyleSheet("QLabel {color : black; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
		ol->setStyleSheet("QLabel {color : black; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
    
    this->curr_map_number++;
    pb->setValue(this->curr_map_number);

		/*TODO
      Need to generate popup if map number is 5 
      for aided, just tell them that test session is beginning
      for unaided, must tell them that test session is beginning 
      along with the ideal value with given times */
 
		if(curr_map_number == 17){
      QWidget* popup = new QWidget();
  	  popup->setAttribute(Qt::WA_DeleteOnClose);
   	  QRect rec = QApplication::desktop()->screenGeometry();
      int height = rec.height();
      int width = rec.width();
      popup->move(width / 2, height / 2);
    	QLabel* lab = new QLabel("You have reached the end!! Thank you!!");
    	QPushButton* but = new QPushButton("Close");

    	connect(but, SIGNAL(released()), popup, SLOT(close()));

    	QVBoxLayout* layout = new QVBoxLayout();
    	layout->addWidget(lab);
    	layout->addWidget(but);

    	popup->setLayout(layout);
    	popup->show();
    }


		if(curr_map_number <= 5 && is_aided == 0)
		{
    	pw = new QWidget();
			pw->setAttribute(Qt::WA_DeleteOnClose);
      QRect rec = QApplication::desktop()->screenGeometry();
  		int height = rec.height();
  		int width = rec.width();
  		pw->move(width / 2, height / 2);

      QString lab_val = QString("For given durations,\nYour last sequence: ");
      lab_val += this->curr_sequence;
      lab_val += "\nYour cost: " + this->curr_cost;
      lab_val += "\nOptimal sequence: " + this->aided_optimal_sequence;
      lab_val += "\nOptimal cost: " + this->aided_optimal_cost;
     
			QLabel* lab = new QLabel(lab_val);
      QPushButton* but = new QPushButton("Close");

      QObject::connect(but, SIGNAL(released()), pw, SLOT(close()));   

			QVBoxLayout* layout = new QVBoxLayout();
			layout->addWidget(lab);
      layout->addWidget(but);
    
      pw->setLayout(layout);
			pw->show();

      //publish this information to data node
      custom_messages::R2D r2d;
      r2d.stamp = ros::Time::now();
	    r2d.id = this->name;
   		r2d.map_number = "";
   		r2d.iteration = "";
   		r2d.event_type = EVENT_BUTTON;
   		r2d.description = "Optimal sequence for given time";
   		r2d.switchtime_string = "";
   		r2d.sequence_string = (this->aided_optimal_sequence).toStdString();
      r2d.cost_of_path = (this->aided_optimal_cost).toStdString();
   		id_publisher.publish(r2d);
   		ros::spinOnce();
    }


    if(curr_map_number == 5){

	    QWidget* popup = new QWidget();
  	  popup->setAttribute(Qt::WA_DeleteOnClose);
   	  QRect rec = QApplication::desktop()->screenGeometry();
      int height = rec.height();
      int width = rec.width();
      popup->move(width / 2, height / 2);

     	QLabel* lab = new QLabel("Test Session is about to Begin!!");
    	QPushButton* but = new QPushButton("Close");

    	connect(but, SIGNAL(released()), popup, SLOT(close()));

    	QVBoxLayout* layout = new QVBoxLayout();
    	layout->addWidget(lab);
    	layout->addWidget(but);

    	popup->setLayout(layout);
    	popup->show();
    }
    

		custom_messages::R2C r2c;
		r2c.next = 1;
		ic_publisher.publish(r2c);
		ros::spinOnce();
    


    // Send r2d
   custom_messages::R2D r2d;
   r2d.stamp = ros::Time::now();
   r2d.id = this->name;
   r2d.map_number = "";
   r2d.iteration = "";
   r2d.event_type = EVENT_BUTTON;
   r2d.description = "Next Clicked";
   r2d.switchtime_string = "";
   r2d.sequence_string = "";
   id_publisher.publish(r2d);
   ros::spinOnce();



		return;
}

void MyViz::setNodeHandle(ros::NodeHandle nn)
{
  n = nn;
}


void MyViz::callBack2(const custom_messages::C2R::ConstPtr& msg)
{
  ROS_INFO("Callback2 function inside interface");
  std::vector<string> sequence_string_array = msg->sequence_string_array;
  float cost_of_path = msg->cost_of_path;
  string s = "";
  for(int i = 0; i < sequence_string_array.size(); i++)
  {
		s += sequence_string_array[i];
    if( i != sequence_string_array.size() - 1 ) s += "->";
  }
  this->aided_optimal_sequence = QString::fromStdString(s);
  this->aided_optimal_cost = QString::number(cost_of_path);
  return;

}




void MyViz::callBack(const custom_messages::C2R::ConstPtr& msg)
{
  QApplication::restoreOverrideCursor();
  pp->close();

  if(msg->error == 1)
  {
    QString str = QString("A rare bug occurred! Please try again");
    generateErrorPopup(str);
    return;
  }

  ROS_INFO("Callback function inside interface called!\n");
  //1. Parse and Convert
  std::vector<string> sequence_string_array = msg->sequence_string_array;
  float cost_of_path = msg->cost_of_path;
  int is_valid_path = (int) msg->is_valid_path;
  int is_complete = (int) msg->is_complete;
  int is_optimal = (int) msg->is_optimal;
 
  printf("Interface valid, complete, optimal %d, %d, %d\n", is_valid_path, is_complete, is_optimal);

  
  //4. Update the Console Widget
  ROS_INFO("Done sending R2D, now updating console");
  string s = "";
  for(int i = 0; i < sequence_string_array.size(); i++)
  {
		s += sequence_string_array[i];
    if( i != sequence_string_array.size() - 1 ) s += "->";
  }
  this->curr_sequence = QString::fromStdString(s);
     

  this->curr_cost = QString::number(cost_of_path);
  if(is_valid_path == 1) this->curr_valid = "VALID";
  else this->curr_valid = "INVALID";
  if(is_complete) this->curr_complete = "Path arrives at destination";
  else this->curr_complete = "Path does not arrive at destination";
  cw->update(this->curr_sequence, this->curr_switchtime, 
						 this->curr_cost, this->curr_valid, this->curr_complete);

  ROS_INFO("Console is done being updated!");

  //4.5 Update the Label Colors
  if(is_valid_path) vl->setStyleSheet("QLabel {color : green; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
  else vl->setStyleSheet("QLabel {color : red; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
 
  if(is_complete) cl->setStyleSheet("QLabel {color : green; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
  else cl->setStyleSheet("QLabel {color : red; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");

  if(is_optimal) ol->setStyleSheet("QLabel {color : green; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");
  else ol->setStyleSheet("QLabel {color : red; font-size: 20px; font-style: bold; qproperty-alignment: AlignCenter }");

  this->received_C = 1;

  compw->updateValue(cost_of_path);

  //Update time horizon and sequence table and console widget
  if(is_aided == 1 || is_aided == 2){

    std::vector<float> switchtime_arr;
    std::vector<int> sequence_arr;
    std::vector<int> switchtime_timehorizon;
    std::vector<QString> color_timehorizon;
    std::vector<QString> sequence_arr_string;
		QString curr_sequence_short = QString("");

		for(int i = 0; i < sequence_string_array.size(); i++)
    {
      int index = std::find(behavior_array.begin(), behavior_array.end(), sequence_string_array[i]) - behavior_array.begin();
      curr_sequence_short += QString::fromStdString(behavior_array_short[index]);
      if(i != sequence_string_array.size() - 1) curr_sequence_short += ",";

      sequence_arr_string.push_back(QString::fromStdString(sequence_string_array[i]));
    }


    switchtime_arr = this->switchtimeInputConvert(this->curr_switchtime);
    sequence_arr = this->sequenceInputConvert(curr_sequence_short);

    for(int i = 0; i < switchtime_arr.size(); i++)
  	{
      switchtime_timehorizon.push_back( (int) switchtime_arr[i] * 10 );
    }

    for(int i = 0; i < sequence_arr.size(); i++)
    {
     
	  	QString color = this->color_array[sequence_arr[i]];
  		color_timehorizon.push_back( color );
    }
   

    thw->setValues(switchtime_timehorizon, color_timehorizon);
  
    //We must fill up the table with new sequences!!
    stw->setSequence(sequence_arr_string);
  	  

    //Newly Added!!
    if(is_optimal == 0 || is_complete == 0)
    {
      QString mes = QString("This is not the optimal sequence, simply the best attempt.\n Try to expand the durations to get a better result.");
      this->generateErrorPopup(mes);

    }
  }

  

  // Send r2d data
  custom_messages::R2D r2d;
  r2d.stamp = ros::Time::now();
  r2d.id = this->name;
  r2d.map_number = std::to_string(msg->map_number);
  r2d.iteration = (cw->getInfo()).toStdString();
  r2d.event_type = 1; //trajectory information
  r2d.description = "Sequence Generated";
  r2d.switchtime_string = this->curr_switchtime.toStdString();
  r2d.sequence_string = this->curr_sequence.toStdString();
  r2d.cost_of_path = std::to_string(msg->cost_of_path);
  if(msg->is_valid_path) r2d.is_valid_path = "VALID";
  else r2d.is_valid_path = "INVALID";
  if(msg->is_complete) r2d.is_complete = "COMPLETE";
  else r2d.is_complete = "INCOMPLETE";

  id_publisher.publish(r2d);
  ros::spinOnce();

  return;

}

