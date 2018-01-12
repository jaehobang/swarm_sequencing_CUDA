#include "compare_widget.h"
#include <algorithm>
#include <QtWidgets>

Compare_Widget::Compare_Widget(QWidget *parent , QString name) : QWidget( parent )
{
    prev_value = 0;
    curr_value = 0;
    prev_label_name = name + " prev";
    curr_label_name = name + " curr";
    prev_slider = new QSlider(Qt::Horizontal);
    curr_slider = new QSlider(Qt::Horizontal);

    prev_slider->setMinimum(0);
    curr_slider->setMinimum(0);

    prev_label = new QLabel(prev_label_name);
    curr_label = new QLabel(curr_label_name);

    prev_value_label = new QLabel("0");
    curr_value_label = new QLabel("0");

    QHBoxLayout* h1 = new QHBoxLayout();
    QHBoxLayout* h2 = new QHBoxLayout();

    h1->addWidget(prev_label);
    h1->addWidget(prev_slider);
    h1->addWidget(prev_value_label);
    h2->addWidget(curr_label);
    h2->addWidget(curr_slider);
    h2->addWidget(curr_value_label);

    QVBoxLayout* v1 = new QVBoxLayout();
    v1->addLayout(h1);
    v1->addLayout(h2);

    this->setLayout(v1);


}



void Compare_Widget::updateValue(float curr)
{
    prev_value = curr_value;
    curr_value = curr;

    /* Decide on the maximum */
    float max_value = std::max(prev_value, curr_value) * 1.2;
    prev_slider->setMaximum( (int) max_value );
    curr_slider->setMaximum( (int) max_value );

    prev_slider->setValue( (int) prev_value);
    curr_slider->setValue( (int) curr_value);

    prev_value_label->setText(QString::number(prev_value));
    curr_value_label->setText(QString::number(curr_value));





}
