#ifndef SEQUENCEWIDGET_H
#define SEQUENCEWIDGET_H

#include <QtWidgets>

class SequenceWidget : public QWidget
{
    Q_OBJECT

public:
    SequenceWidget(QWidget *parent = 0); //3 minutes per map
    QString getInfo();
		void reset();

private Q_SLOTS:


private:
    QLabel* sequence_label;
    QLineEdit* sequence_text;
    QTableView* sequence_view;

};



#endif // SEQUENCEWIDGET_H
