#ifndef TIMER_H
#define TIMER_H

#include <QLCDNumber>

class Timer : public QLCDNumber
{
    Q_OBJECT

public:
    Timer(QWidget *parent = 0, int time_limit = 18000); //3 minutes per map
    void reset(int time_limit);
    void pauseResume();

Q_SIGNALS:
    void signalDone();

private Q_SLOTS:
    void showTime();

private:
		QTimer* timer;
    int time_left;
    bool is_paused;

};


#endif // TIMER_H
