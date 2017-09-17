#include <QApplication>
#include <ros/ros.h>
#include "myviz.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
  }

  ros::NodeHandle n;

  QApplication app( argc, argv );

  MyViz* myviz = new MyViz();
  myviz->setNodeHandle(n);
  myviz->resize(700, 700);
  myviz->show();

  app.exec();

  delete myviz;
}
