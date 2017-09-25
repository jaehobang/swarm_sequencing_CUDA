#ifndef TIMERWIDGET_H
#define TIMERWIDGET_H

#include <QtWidgets>
#include "timer.h"

class TimerWidget : public QWidget
{
    Q_OBJECT

public:
    TimerWidget(QWidget *parent = 0); //3 minutes per map
		void reset(int time_limit);

Q_SIGNALS:
		void signalDone();


private Q_SLOTS:
		void slotDone();
    void pauseResume();

private:
    Timer* timer;
    QLabel* timer_label;

};


#endif // TIMERWIDGET_H
