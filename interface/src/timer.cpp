#include <QtWidgets>
#include "timer.h"

Timer::Timer(QWidget *parent, int default_time) : QLCDNumber(parent)
{
    time_left = default_time;
    setSegmentStyle(Filled);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(showTime()));
    timer->start(1000);
    is_paused = 0;
    showTime();


}



void Timer::showTime()
{
   if(is_paused == 0 && time_left != 0) time_left--;
    else if(is_paused == 0 && time_left == 0){
			timer->stop();
    	this->signalDone();
		}
    QString text = QString::number(time_left);
 
    display(text);
}

void Timer::reset(int time_limit)
{
    time_left = time_limit + 1;
		timer->start(1000);
		showTime();
}

void Timer::pauseResume()
{
  //If pause == 1, it means stop the timer, 
  //If pause == 0, it means start the timer, 
  this->is_paused = !this->is_paused; 


}

