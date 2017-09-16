#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QPushButton>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "myviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent)
  : QWidget( parent )
{

  //Deal with Ros related stuff first
  ROS_INFO("starting interface_node...\n");
  ic_publisher = n.advertise<custom_messages::R2C>("/hsi/R2C", 1000);
  id_publisher = n.advertise<custom_messages::R2D>("/hsi/R2D", 1000);
  i_publisher = n.advertise<geometry_msgs::PoseArray>("/hsi/interface", 1000);
  i_subscriber = n.subscribe<custom_messages::C2R>("/hsi/C2R", 1000, &MyViz::callBack, this);
  pa.header.seq = 0;
  pa.header.frame_id = "global";
  // Construct and lay out labels and slider controls.
  /*
  QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  */
  QPushButton* start_button = new QPushButton("My Button", this);
  start_button->setGeometry(QRect(QPoint(100,100), QSize(200, 50)));
  start_button->setText("Start");
  start_button->resize(100,100);
  
  QGridLayout* controls_layout = new QGridLayout();
  //controls_layout->addWidget( thickness_label, 0, 0 );
  //controls_layout->addWidget( thickness_slider, 0, 1 );
  //controls_layout->addWidget( cell_size_label, 1, 0 );
  //controls_layout->addWidget( cell_size_slider, 1, 1 );
  controls_layout->addWidget(start_button, 2, 0);

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  //connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  //connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));
  connect(start_button, SIGNAL(released()), this, SLOT(sendR2C()));


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

  // Create a Grid display.
  trajectory_ = manager_->createDisplay( "rviz/PoseArray", "Trajectory Topic", true );
  ROS_ASSERT( trajectory_ != NULL );

  // Configure the GridDisplay the way we like it.
  //grid_->subProp( "Line Style" )->setValue( "Billboards" );
  //grid_->subProp( "Color" )->setValue( QColor(Qt::blue) );
  trajectory_->subProp("Topic")->setValue("/hsi/interface");


  // Initialize the slider values.
  //thickness_slider->setValue( 25 );
  //cell_size_slider->setValue( 10 );


  //TODO: Not sure if I can do this and everything else will function 
  //ros::spin();
  ROS_INFO("Reached the end of myViz constructor");

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
/*
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}


// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}
*/

void MyViz::sendR2C()
{  
   ROS_INFO("Start button release detected...sending R2C message\n");
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

 
  }

  //3. Send to D
  custom_messages::R2D r2d;
  r2d.stamp = ros::Time::now();
  r2d.event_type = 2; //2 = trajectory information
  r2d.cost_of_path = cost_of_path;
  r2d.is_valid_path = is_valid_path;
  for(int i = 0; i < sequence_names.size(); i++)
  {
    r2d.behavior_sequences.push_back(behavior_array[sequence_names[i]]);
  }
  id_publisher.publish(r2d);

  return;

}

