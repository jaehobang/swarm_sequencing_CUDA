#ifndef CONSOLEWIDGET1_H
#define CONSOLEWIDGET1_H

#include <QtWidgets>

class ConsoleWidget1 : public QPlainTextEdit
{
    Q_OBJECT

public:
    ConsoleWidget1(QWidget *parent = 0); //3 minutes per map
    void update(QString sequence, QString switchtime, QString cost, QString valid, QString complete);
		QString getInfo();
		void reset();


private Q_SLOTS:


private:

    QString text_info;
    QString sequence_val;
    QString switchtime_val;
    QString cost_val;
    QString validity_val;
		QString complete_val;

    int iteration;
};


#endif // CONSOLEWIDGET1_H
