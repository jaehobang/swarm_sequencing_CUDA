#include <QtWidgets>
#include "timer.h"

Timer::Timer(QWidget *parent, int default_time) : QLCDNumber(parent)
{
    d_time = default_time;
    time_left = default_time;
    setSegmentStyle(Filled);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(showTime()));
    timer->start(1000);

    showTime();


}



void Timer::showTime()
{
    QString text = QString::number(time_left);
    if(time_left != 0) time_left--;
    else {
			timer->stop();
    	this->signalDone();
		}
    display(text);
}

void Timer::reset()
{
    time_left = d_time;
		timer->start(1000);
		showTime();
}



