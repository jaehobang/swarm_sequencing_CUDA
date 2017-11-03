#ifndef SEQUENCETIMEWIDGET_H
#define SEQUENCETIMEWIDGET_H

#include <QtWidgets>
#include <ros/ros.h>
#include "custom_messages/R2D.h"

#define SEQ_COL 2
#define TIME_COL 1


class SequenceTimeWidget : public QWidget
{

    Q_OBJECT

public:
    SequenceTimeWidget(QWidget *parent = 0);
    QString getSequence();
    QString getSwitchTime();
    void reset();
    void setPublisher(ros::Publisher id_publisher);
		void setSequence(std::vector<QString> sequence);

private Q_SLOTS:
    void addButtonClicked();
    void deleteButtonClicked();
    void sequenceCellClicked(int row, int col);
    void behaviorCellClicked(int row, int col);

private:

    QTableWidget* behavior_list;
    QTableWidget* sequence_list;
    QToolButton* add_button;
    QToolButton* delete_button;
    QPushButton* submit_button;
    QString durationtime_text;
    std::vector<QString> behavior_array = { "Rendezvous", "Antirendezvous", "Flock East", "Flock North","Flock West", "Flock South", "Line X", "Line Y" };
    std::vector<QString> behavior_array_short = {"r", "a", "e", "n", "w", "s", "x", "y"};
    std::vector<QString> behavior_color_array = {"cyan", "purple", "gray", "brown","green", "orange", "pink", "yellow"};
    std::vector<QString> sequence_array;
    int curr_selected_sequence_row;
    QString curr_selected_behavior;
    ros::Publisher id_publisher;


    void generateBehaviorList();
    void generateSequenceList();
};



#endif // SEQUENCETIMEWIDGET_H
