#include <QtWidgets>
#include <QApplication>
#include "switchtimewidget.h"

SwitchTimeWidget::SwitchTimeWidget(QWidget *parent) : QWidget( parent )
{
    switchtime_label = new QLabel("Input Switch Times Below(0 - 50.0):", this);
    switchtime_text = new QLineEdit("ex)5.1,9.9,21.3", this); /* sequence_text->text() -- retrieves the text
                                       sequence_text->setText("adsfadf") -- sets the text */


    QVBoxLayout* switchtime_layout = new QVBoxLayout();
    switchtime_layout->addWidget(switchtime_label);
    switchtime_layout->addWidget(switchtime_text);

    this->setLayout(switchtime_layout);


}


QString SwitchTimeWidget::getInfo()
{
    return switchtime_text->text();
}

void SwitchTimeWidget::reset()
{
    switchtime_text->setText("ex)5.1,9.9,21.3");
		return;
}
