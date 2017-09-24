#include "full_panel.h"
#include <QApplication>
#include "ros/ros.h"


int main(int argc, char **argv)
{

  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "panel", ros::init_options::AnonymousName );
  }



  ros::NodeHandle n;
  QApplication a(argc, argv);

  FullPanel w;
  w.setHandle(n);
  w.show();
  w.tmpPublish();
  
  for(int i = 0; i < 3; i++)
  {
    w.publishMessage();
    w.deleteMessage();
  }
  
  return a.exec();


}

