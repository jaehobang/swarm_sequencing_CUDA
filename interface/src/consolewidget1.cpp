#include <QtWidgets>
#include <QApplication>
#include "consolewidget1.h"

ConsoleWidget1::ConsoleWidget1(QWidget *parent) : QPlainTextEdit( parent )
{
    iteration = 0;
    sequence_val = "";
    switchtime_val = "";
    cost_val = "";
    validity_val = "";
    text_info = "";

    this->setReadOnly(true);
    //this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
}

void ConsoleWidget1::update(QString sequence, QString switchtime,
                           QString cost, QString valid)
{
    iteration++;
    sequence_val = sequence;
    switchtime_val = switchtime;
    cost_val = cost;
    validity_val = valid;


    QString i = "Iteration: ";
    QString s = "Sequence: ";
    QString sw = "Switch Times: ";
    QString c = "Cost of Path: ";
    QString v = "Validity of Path: ";
    QString st = "\n---------------------------------------------\n\n";

    text_info += st;
    text_info += i + QString::number(iteration) + "\n";
    text_info += s + sequence_val + "\n";
    text_info += sw + switchtime_val + "\n";
    text_info += c + cost_val + "\n";
    text_info += v + validity_val + "\n";

    this->setPlainText(text_info);
    //text_box->setT
}

QString ConsoleWidget1::getInfo()
{
    return QString(iteration);
}


void ConsoleWidget1::reset()
{
    iteration = 0;
		sequence_val = "";
		switchtime_val = "";
		cost_val = "";
		validity_val = "";
		text_info = "";
		this->setPlainText(text_info);
		return;
}

