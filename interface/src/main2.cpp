#include <QApplication>
#include <ros/ros.h>
#include "myviz.h"
#include "popupwidget.h"
#include "customapplication.h"

int main(int argc, char **argv)
{
  /*

  QApplication app( argc, argv );

  string name = "Tom";
  uint8_t isAided = 1;

  MyViz* myviz = new MyViz();
  myviz->setNodeHandle(n);
  myviz->setNameAndAided(name, isAided);
  myviz->resize(700, 700);
  myviz->show();

  app.exec();

  delete myviz;
  */

  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
  }

  ros::NodeHandle n;


  MyApplication app(argc, argv);
  printf("before set node handler...\n");
  app.setNodeHandler(n);
  printf("after set node handler...\n");
  app.exec();
  printf("After app exec...\n");
  

}
