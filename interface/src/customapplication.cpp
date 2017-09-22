#include <QtWidgets>
#include <QApplication>
#include "customapplication.h"
#include "myviz.h"
#include "popupwidget.h"
#include <ros/ros.h>
#include <string>

MyApplication::MyApplication(int argc, char** argv) : QApplication(argc, argv)
{
  this->myviz = new MyViz();
  this->popup = new PopupWidget();
  this->popup->resize(200,200);
  this->popup->show();
  

  connect(this->popup, SIGNAL(widgetClosed()), this, SLOT(createHSIWidget()));

  return;

}

MyApplication::~MyApplication()
{
  delete this->myviz;
  delete this->popup;
}

void MyApplication::setNodeHandler(ros::NodeHandle n)
{
  this->n = n;
}

void MyApplication::createHSIWidget()
{
  string name = this->popup->getName().toStdString();
  uint8_t isAided = this->popup->getIsAided();
  this->myviz->setNodeHandle(this->n);
  this->myviz->setNameAndAided(name, isAided);
  this->myviz->resize(1200, 600);
  this->myviz->show();
  return;
}


