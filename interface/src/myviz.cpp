#include "myviz.h"


#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/yaml_config_reader.h"
#include "rviz/config.h"
#include "rviz/yaml_config_writer.h"

MyViz::MyViz( QWidget* parent) : QWidget( parent )
{
  //Deal with Ros related stuff first
  ROS_INFO("starting interface_node...\n");
  ic_publisher = n.advertise<custom_messages::R2C>("/hsi/R2C", 1000);
  id_publisher = n.advertise<custom_messages::R2D>("/hsi/R2D", 1000);
  ic_subscriber = n.subscribe<custom_messages::C2R>("/hsi/C2R", 1000, &MyViz::callBack, this);
  
  this->name = "";
  this->is_aided = 0;
  this->curr_map_number = 0;
  this->eot_processed = 0;


  tw = new TimerWidget(this);
 // sw = new SequenceWidget(this);
 // stw = new SwitchTimeWidget(this);
  cw = new ConsoleWidget1(this);
  thw = new TimeHorizonWidget(this);
  stw = new SequenceTimeWidget(this);


  rb = new QPushButton(QApplication::translate("childwidget", "Generate"), this);
  nb = new QPushButton(QApplication::translate("childwidget", "Next"), this); 
  sb = new QPushButton(QApplication::translate("childwidget", "Submit"), this);

  render_panel_ = new rviz::RenderPanel();

  tw->setFixedSize(QSize(450, 100));
  stw->setFixedSize(QSize(450, 400));
  cw->setFixedSize(QSize(450, 120));
  thw->setFixedSize(QSize(450, 100));

  connect(tw, SIGNAL(signalDone()), this, SLOT(timerDone()));
  connect(rb, SIGNAL(released()), this, SLOT(generate()));
  connect(nb, SIGNAL(released()), this, SLOT(checkNext()));
	connect(sb, SIGNAL(released()), this, SLOT(submit()));

  QVBoxLayout* col1 = new QVBoxLayout();
  col1->addWidget(render_panel_);
 
  QHBoxLayout* rowTmp = new QHBoxLayout();
  rowTmp->addWidget(rb);
  rowTmp->addWidget(nb);

  QHBoxLayout* rowTmp2 = new QHBoxLayout();
  QLabel* labelTmp = new QLabel();
  rowTmp2->addWidget(labelTmp);
  rowTmp2->addWidget(sb);

  QVBoxLayout* col2 = new QVBoxLayout();
  col2->addWidget(tw);
  //col2->addWidget(sw);
  col2->addWidget(stw);

  col2->addLayout(rowTmp2);
  col2->addWidget(thw);
  col2->addWidget(cw);
 
  col2->addLayout(rowTmp);

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

void MyViz::setNameAndAided(string n, uint8_t is_aided)
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

     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QLabel* lab = new QLabel("ERROR!\nMisspelled behavior in Sequence!!");
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
     layout->addWidget(lab);
     layout->addWidget(but);

     popup->setLayout(layout);
     popup->show();
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
   
     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QString lab_text;
     if(switchtime == "-1") lab_text = QString("ERROR!\nInput is not a int or floating point number.");
     else lab_text = QString("ERROR!\nTotal time input exceeded maximum simulation time (50 seconds)!");
     QLabel* lab = new QLabel(lab_text);
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
   	 layout->addWidget(lab);
   	 layout->addWidget(but);

   	 popup->setLayout(layout);
     popup->show();
     return;
   }   

   //2. Running the input checker
   switchtime_arr = this->switchtimeInputConvert(switchtime);

   if(sequence_arr.size() > switchtime_arr.size()){
     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QString lab_text = QString("");
     QLabel* lab = new QLabel("ERROR\nMore behaviors given than durations!!");
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
     layout->addWidget(lab);
     layout->addWidget(but);

     popup->setLayout(layout);
     popup->show();
     return;

   }
   
   if(is_aided == 0 && sequence_arr.size() != switchtime_arr.size()){
     QWidget* popup = new QWidget();
     popup->setAttribute(Qt::WA_DeleteOnClose);
     QString lab_text = QString("");
     QLabel* lab = new QLabel("ERROR\nNumber of Behaviors must match number of durations!!");
     QPushButton* but = new QPushButton("Close");

     connect(but, SIGNAL(released()), popup, SLOT(close()));

     QVBoxLayout* layout = new QVBoxLayout();
     layout->addWidget(lab);
     layout->addWidget(but);

     popup->setLayout(layout);
     popup->show();
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
   
   
	  
	 return;

}


void MyViz::generate()
{  
   this->submit(); //Run this in case sequence changed after user pressed submit
 
   //1. Collecting all the data
   std::vector<int> sequence_arr;
   std::vector<float> switchtime_arr;
   QString sequence;
   QString switchtime;
   QString iteration;
   int iter;

   if(this->is_aided == 0)
   {
     sequence = stw->getSequence();
     this->curr_sequence = sequence;
 	   sequence_arr = this->sequenceInputConvert(sequence);
     //3. if invalid
  	 if(sequence_arr.size() != 0 && sequence_arr[0] == -1){
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

   switchtime = stw->getSwitchTime();
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
   r2d.button_name = "Run";
   r2d.time_array = switchtime_arr;
   r2d.sequence_string_array = b_sequences;
   r2d.name = this->name;
   r2d.is_aided = this->is_aided;
   id_publisher.publish(r2d);
   ros::spinOnce();

   ROS_INFO("Done with generate()");
   return;
}


void MyViz::checkNext()
{
   np = new QWidget();
   np->setAttribute(Qt::WA_DeleteOnClose);
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
    tw->reset();
		stw->reset();
		cw->reset();
    thw->reset();
		this->curr_map_number++;

		/*TODO
      Need to generate popup if map number is 5 
      for aided, just tell them that test session is beginning
      for unaided, must tell them that test session is beginning 
      along with the ideal value with given times */
 
		if(curr_map_number == 20){
      QWidget* popup = new QWidget();
  	  popup->setAttribute(Qt::WA_DeleteOnClose);
   		QLabel* lab = new QLabel("You have reached the end!! Thank you!!");
    	QPushButton* but = new QPushButton("Close");

    	connect(but, SIGNAL(released()), popup, SLOT(close()));

    	QVBoxLayout* layout = new QVBoxLayout();
    	layout->addWidget(lab);
    	layout->addWidget(but);

    	popup->setLayout(layout);
    	popup->show();
    }

		if(curr_map_number == 5 && eot_processed == 0 && is_aided == 0)
		{
    	pw = new QWidget();
			pw->setAttribute(Qt::WA_DeleteOnClose);
			QLabel* lab = new QLabel("Please wait......");
      QPushButton* but = new QPushButton("Close");

			QVBoxLayout* layout = new QVBoxLayout();
			layout->addWidget(lab);
			pw->show();
			
     //2. Running the input checker
      std::vector<float> switchtime_arr = 
					this->switchtimeInputConvert(this->curr_switchtime);
    
      custom_messages::R2C r2c;
      r2c.stamp = ros::Time::now();
      r2c.is_aided = this->is_aided; //0 is unaided
      r2c.eot = 1;
      /* Debugging purposes.... */
      printf("Printing the switch times..\n");
      r2c.time_array = switchtime_arr;
      ic_publisher.publish(r2c);
      ros::spinOnce();
      return;
    }

    if(curr_map_number == 5 && is_aided){

	    QWidget* popup = new QWidget();
  	  popup->setAttribute(Qt::WA_DeleteOnClose);
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
    
		return;
}

void MyViz::setNodeHandle(ros::NodeHandle nn)
{
  n = nn;
}

void MyViz::callBack(const custom_messages::C2R::ConstPtr& msg)
{

  ROS_INFO("Callback function inside interface called!\n");
  //1. Parse and Convert
  std::vector<string> sequence_string_array = msg->sequence_string_array;
  float cost_of_path = msg->cost_of_path;
  int is_valid_path = (int) msg->is_valid_path;
  int is_complete = (int) msg->is_complete;
 


  if(msg->eot) {
    pw->close();
    QWidget* popup = new QWidget();
    popup->setAttribute(Qt::WA_DeleteOnClose);
		QString label_string = "Test Session is about to begin!\n";
    if((int) cost_of_path == (int) curr_cost.toFloat()){
      label_string += "Just for reference, for the given times, computer generated the optimal sequence that is same as yours!";
    }
    else{
      label_string += "Just for reference, for the given times, computer generated the optimal sequence that is lower in cost than yours!\n";
      label_string += "The sequence is ";
    	for(int i = 0; i < sequence_string_array.size(); i++)
   		{
      	label_string += QString::fromStdString(sequence_string_array[i]);
      	if(i != sequence_string_array.size() -1 ) label_string += ", ";
    	}
	  }
    QLabel* lab = new QLabel(label_string);
    QPushButton* but = new QPushButton("Close");

    connect(but, SIGNAL(released()), popup, SLOT(close()));

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(lab);
    layout->addWidget(but);

    popup->setLayout(layout);
    popup->show();

		this->eot_processed = 1;
		return;
  }


  
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

  if(is_aided){

    std::vector<float> switchtime_arr;
    std::vector<int> sequence_arr;
    std::vector<int> switchtime_timehorizon;
    std::vector<QString> color_timehorizon;

		QString curr_sequence_short = QString("");

		for(int i = 0; i < sequence_string_array.size(); i++)
    {
      int index = std::find(behavior_array.begin(), behavior_array.end(), sequence_string_array[i]) - behavior_array.begin();
      curr_sequence_short += QString::fromStdString(behavior_array_short[index]);
      if(i != sequence_string_array.size() - 1) curr_sequence_short += ",";
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
  }

  return;

}

