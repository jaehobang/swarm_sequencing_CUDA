#ifndef SWITCHTIMEWIDGET_H
#define SWITCHTIMEWIDGET_H

#include <QtWidgets>

class SwitchTimeWidget : public QWidget
{
    Q_OBJECT

public:
    SwitchTimeWidget(QWidget *parent = 0); 
    QString getInfo();
		void reset();

private Q_SLOTS:


private:
    QLabel* switchtime_label;
    QLineEdit* switchtime_text;
};


#endif // SWITCHTIMEWIDGET_H
