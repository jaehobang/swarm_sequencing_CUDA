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
		complete_val = "";
    text_info = "";

    this->setReadOnly(true);
    //this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
}

void ConsoleWidget1::update(QString sequence, QString switchtime,
                           QString cost, QString valid, QString complete)
{
    iteration++;
    sequence_val = sequence;
    switchtime_val = switchtime;
    cost_val = cost;
    validity_val = valid;
		complete_val = complete;


    QString i = "Iteration: ";
    QString s = "Sequence: ";
    QString sw = "Switch Times: ";
    QString c = "Cost of Path: ";
    QString v = "Validity of Path: ";
    QString co = "Objective Reached: ";
    QString st = "\n---------------------------------------------\n\n";
		QString text_info_i = "";

    text_info_i += i + QString::number(iteration) + "\n";
    text_info_i += s + sequence_val + "\n";
    text_info_i += sw + switchtime_val + "\n";
    text_info_i += c + cost_val + "\n";
    text_info_i += v + validity_val + "\n";
    text_info_i += co + complete_val + "\n";
		text_info_i += st;		
		
		text_info = text_info_i + text_info;

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
    complete_val = "";
		text_info = "";
		this->setPlainText(text_info);
		return;
}

