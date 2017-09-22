#ifndef SEQUENCETIMEWIDGET_H
#define SEQUENCETIMEWIDGET_H

#include <QtWidgets>

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
    std::vector<QString> behavior_array = { "Rendezvous", "Flocking", "Flock East", "Flock North","Flock West", "Flock South", "Antirendezvous" };
    std::vector<QString> behavior_array_short = {"r", "i", "e", "n", "w", "s", "a"};
    std::vector<QString> behavior_color_array = {"cyan", "purple", "blue", "brown","green", "orange", "pink"};
    std::vector<QString> sequence_array;
    int curr_selected_sequence_row;
    QString curr_selected_behavior;

    void generateBehaviorList();
    void generateSequenceList();

};



#endif // SEQUENCETIMEWIDGET_H
