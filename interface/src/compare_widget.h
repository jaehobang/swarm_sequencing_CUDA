#ifndef COMPARE_WIDGET_H
#define COMPARE_WIDGET_H

#include <QtWidgets>

class Compare_Widget : public QWidget
{
    Q_OBJECT

public:
    Compare_Widget(QWidget *parent = 0, QString name = "FOO!");
    void updateValue(float curr);

private:
    float prev_value;
    float curr_value;
    QString prev_label_name;
    QString curr_label_name;
    QSlider* prev_slider;
    QSlider* curr_slider;
    QLabel* prev_label;
    QLabel* curr_label;
    QLabel* prev_value_label;
    QLabel* curr_value_label;
};

#endif // COMPARE_WIDGET_H
