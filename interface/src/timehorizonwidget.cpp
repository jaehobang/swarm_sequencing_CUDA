#include "timehorizonwidget.h"


TimeHorizonWidget::TimeHorizonWidget ( QWidget* parent ) : QWidget( parent )
{
    layout = new QStackedLayout();
    layout->setMargin( 0 );
    layout->setStackingMode( QStackedLayout::StackAll );

    for(int i = 0; i < 10; i++)
    {
        insertBar();
    }
    std::reverse( progress_bars.begin(), progress_bars.end() );

    QLabel* timehorizon_label = new QLabel();
    timehorizon_label->setText("Time Horizon (in seconds)");

    QLabel* time_label = new QLabel();
    QString num_of_spaces = "  ";
    QString text = "";
    for(int i = 0; i < 13; i++)
    {
        text += QString::number(i * 5);
        text += "    ";
        if(i != 10) text += num_of_spaces;
    }

    time_label->setText(text);

    QVBoxLayout* outer_layout = new QVBoxLayout();
    outer_layout->addWidget(timehorizon_label);
    outer_layout->addLayout(layout);
    outer_layout->addWidget(time_label);
    this->setLayout(outer_layout);

}

void TimeHorizonWidget::insertBar()
{
    QProgressBar* bar = new QProgressBar();
    bar->setTextVisible( false );
    bar->setRange(0, 600); //granularity of 0.1 seconds
    layout->addWidget( bar );
    progress_bars.push_back( bar );
}

void TimeHorizonWidget::setValues(std::vector<int> switchtimes, std::vector<QString> colors)
{
    //Update the color of the bar
		qInfo() << "Inside timehorizonwidget setvalues...";
    for(uint i = 0; i < progress_bars.size(); i++){
      	if ( i < colors.size() ){





					QString style = QString("QProgressBar{"
                           "border: 3px solid transparent;"
                           "border-radius: 10px;"
                           "background-color: rgba(0, 0, 0, 0);"
                               "}"
                           "QProgressBar::chunk{"
                           "background-color: ");
 	        style += colors[i];
          style += ";}";
			
	    		progress_bars[i]->setStyleSheet(style);
			  
          progress_bars[i]->setValue( switchtimes[i] );
        }
        else progress_bars[i]->setValue( 0 );
    }

}

void TimeHorizonWidget::reset()
{
  for(int i = 0; i < progress_bars.size(); i++)
  {

    progress_bars[i]->setValue( 0 );

  }
  return;
}

