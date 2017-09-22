#ifndef TIMEHORIZONWIDGET_H
#define TIMEHORIZONWIDGET_H


#include <QtWidgets>


class TimeHorizonWidget : public QWidget
{
    Q_OBJECT
public:
    TimeHorizonWidget ( QWidget* parent = 0 );
    void setValues(std::vector<int> switchtimes, std::vector<QString> colors );
		void reset();

private:
    std::vector< QProgressBar* > progress_bars;
    QStackedLayout* layout;
    void insertBar();


};




#endif // TIMEHORIZONWIDGET_H
