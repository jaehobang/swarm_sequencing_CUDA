#include <QtWidgets>
#include <QApplication>
#include "consolewidget.h"

ConsoleWidget::ConsoleWidget(QWidget *parent) : QWidget( parent )
{

    sequence_val = "";
    switchtime_val = "";
    cost_val = "";
    validity_val = "";

    sequence_label = new QLabel("Sequence Requested:", this);
    sequence_label_ = new QLabel(sequence_val, this);
    switchtime_label = new QLabel("Switch Times Requested:", this);
    switchtime_label_ = new QLabel(switchtime_val, this);
    cost_label = new QLabel("Cost of Path:", this);
    cost_label_ = new QLabel(cost_val, this);
    validity_label = new QLabel("Validity of Path:", this);
    validity_label_ = new QLabel(validity_val, this);



    QVBoxLayout* console_layoutc1 = new QVBoxLayout();
    console_layoutc1->addWidget(sequence_label);
    console_layoutc1->addWidget(switchtime_label);
    console_layoutc1->addWidget(cost_label);
    console_layoutc1->addWidget(validity_label);

    QVBoxLayout* console_layoutc2 = new QVBoxLayout();
    console_layoutc2->addWidget(sequence_label_);
    console_layoutc2->addWidget(switchtime_label_);
    console_layoutc2->addWidget(cost_label_);
    console_layoutc2->addWidget(validity_label_);

    QHBoxLayout* console_layoutr = new QHBoxLayout();
    console_layoutr->addLayout(console_layoutc1);
    console_layoutr->addLayout(console_layoutc2);

    this->setLayout(console_layoutr);
}

void ConsoleWidget::setSequence(QString sequence){
    sequence_label_->setText(sequence);
}


void ConsoleWidget::setSwitchTime(QString switchtime){
    switchtime_label_->setText(switchtime);
}


void ConsoleWidget::setCostofPath(QString cost){
    cost_label_->setText(cost);
}
void ConsoleWidget::setValidityofPath(QString valid){
    validity_label_->setText(valid);
}



