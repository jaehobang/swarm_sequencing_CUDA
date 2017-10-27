#include <QtWidgets>
#include <QApplication>
#include "customapplication.h"
#include "myviz.h"
#include "popupwidget.h"
#include <ros/ros.h>
#include <string>

MyApplication::MyApplication(int argc, char** argv) : QApplication(argc, argv)
{
  printf("Starting myviz...\n");
  this->myviz = new MyViz();
  printf("Starting popup...\n");
  this->popup = new PopupWidget();
  printf("Finished initializing popup...\n");
  this->popup->resize(200,200);
  this->popup->show();
  printf("Finished showing...\n");

  connect(this->popup, SIGNAL(widgetClosed()), this, SLOT(createHSIWidget()));
  printf("returning from constructor of myapplication\n");
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
  int isAided = this->popup->getIsAided();
  this->myviz->setNodeHandle(this->n);
  this->myviz->setNameAndAided(name, isAided);
  this->myviz->resize(1400, 600);
  this->myviz->show();
  return;
}


