#include <QtWidgets>
#include <QApplication>
#include "timer.h"
#include "timerwidget.h"

TimerWidget::TimerWidget(QWidget *parent) : QWidget( parent )
{

    timer = new Timer(this);
    timer_label = new QLabel("Time Left (sec):", this);

		connect(timer, SIGNAL(signalDone()), this, SLOT(slotDone()));

    QHBoxLayout* layout_timer = new QHBoxLayout();
    layout_timer->addWidget(timer_label);
    layout_timer->addWidget(timer);

    this->setLayout(layout_timer);

}

void TimerWidget::reset()
{
		timer->reset();
    return;
}

void TimerWidget::slotDone()
{
		this->signalDone();
    return;
}

