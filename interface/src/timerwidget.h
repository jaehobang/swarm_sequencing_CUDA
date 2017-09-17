#ifndef TIMERWIDGET_H
#define TIMERWIDGET_H

#include <QtWidgets>
#include "timer.h"

class TimerWidget : public QWidget
{
    Q_OBJECT

public:
    TimerWidget(QWidget *parent = 0); //3 minutes per map
		void reset();


Q_SIGNALS:
		void signalDone();


private Q_SLOTS:
		void slotDone();


private:
    Timer* timer;
    QLabel* timer_label;

};


#endif // TIMERWIDGET_H
