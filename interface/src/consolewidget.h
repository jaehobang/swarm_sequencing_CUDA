#ifndef CONSOLEWIDGET_H
#define CONSOLEWIDGET_H

#include <QtWidgets>

class ConsoleWidget : public QWidget
{
    Q_OBJECT

public:
    ConsoleWidget(QWidget *parent = 0); //3 minutes per map
    void setSequence(QString sequence);
    void setSwitchTime(QString switchtime);
    void setCostofPath(QString cost);
    void setValidityofPath(QString valid);

private Q_SLOTS:


private:
    QLabel* sequence_label;
    QLabel* sequence_label_;
    QLabel* switchtime_label;
    QLabel* switchtime_label_;
    QLabel* cost_label;
    QLabel* cost_label_;
    QLabel* validity_label;
    QLabel* validity_label_;

    QString sequence_val;
    QString switchtime_val;
    QString cost_val;
    QString validity_val;
};




#endif // CONSOLEWIDGET_H
