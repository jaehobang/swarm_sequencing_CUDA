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
  w.publishMessage();

  return a.exec();


}

