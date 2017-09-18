#include <QApplication>
#include <QtWidgets>
#include <ros/ros.h>
#include "myviz.h"
#include "popupwidget.h"


class MyApplication  : public QApplication
{
    Q_OBJECT
public:
    MyApplication(int argc ,char** argv);
    virtual ~MyApplication(); 
    void setNodeHandler(ros::NodeHandle n);

private Q_SLOTS:
   void createHSIWidget();

private:
   MyViz* myviz;
   PopupWidget* popup;
   ros::NodeHandle n;
 
};
