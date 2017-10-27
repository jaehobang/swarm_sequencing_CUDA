#include "sequencetimewidget.h"


SequenceTimeWidget::SequenceTimeWidget(QWidget *parent) : QWidget( parent )
{
    curr_selected_behavior = "";
    curr_selected_sequence_row = 0;

    //dw = new DurationWidget();
    this->generateBehaviorList();
    this->generateSequenceList();
    add_button = new QToolButton();
    add_button->setArrowType(Qt::RightArrow);
    delete_button = new QToolButton();
    delete_button->setArrowType(Qt::LeftArrow);

    connect(add_button, SIGNAL(released()), this, SLOT(addButtonClicked()));
    connect(delete_button, SIGNAL(released()), this, SLOT(deleteButtonClicked()));

    QVBoxLayout* arrow_layout = new QVBoxLayout();
    arrow_layout->addWidget(add_button);
    arrow_layout->addWidget(delete_button);


    QHBoxLayout* sequence_layouth = new QHBoxLayout();
    sequence_layouth->addWidget(behavior_list);
    sequence_layouth->addLayout(arrow_layout);
    sequence_layouth->addWidget(sequence_list);


    this->setLayout(sequence_layouth);
}


void SequenceTimeWidget::generateSequenceList()
{
    sequence_list = new QTableWidget(this);
    sequence_list->setRowCount(10);
    sequence_list->setColumnCount(3);

    QStringList m_TableHeader;
    m_TableHeader<<""<<"Time (s)"<<"Sequence";

    sequence_list->setHorizontalHeaderLabels(m_TableHeader);
    sequence_list->verticalHeader()->setVisible(false);
    //sequence_list->setEditTriggers(QAbstractItemView::NoEditTriggers);
    sequence_list->setSelectionBehavior(QAbstractItemView::SelectItems);
    sequence_list->setSelectionMode(QAbstractItemView::SingleSelection);
    sequence_list->setShowGrid(true);
    sequence_list->setStyleSheet("QTableView {selection-background-color: blue;}");

    sequence_list->setItem(0, 0, new QTableWidgetItem("1"));
    sequence_list->setItem(1, 0, new QTableWidgetItem("2"));
    sequence_list->setItem(2, 0, new QTableWidgetItem("3"));
    sequence_list->setItem(3, 0, new QTableWidgetItem("4"));
    sequence_list->setItem(4, 0, new QTableWidgetItem("5"));
    sequence_list->setItem(5, 0, new QTableWidgetItem("6"));
    sequence_list->setItem(6, 0, new QTableWidgetItem("7"));
    sequence_list->setItem(7, 0, new QTableWidgetItem("8"));
    sequence_list->setItem(8, 0, new QTableWidgetItem("9"));
    sequence_list->setItem(9, 0, new QTableWidgetItem("10"));

    sequence_list->setItem(0, 1, new QTableWidgetItem(""));
    sequence_list->setItem(1, 1, new QTableWidgetItem(""));
    sequence_list->setItem(2, 1, new QTableWidgetItem(""));
    sequence_list->setItem(3, 1, new QTableWidgetItem(""));
    sequence_list->setItem(4, 1, new QTableWidgetItem(""));
    sequence_list->setItem(5, 1, new QTableWidgetItem(""));
    sequence_list->setItem(6, 1, new QTableWidgetItem(""));
    sequence_list->setItem(7, 1, new QTableWidgetItem(""));
    sequence_list->setItem(8, 1, new QTableWidgetItem(""));
    sequence_list->setItem(9, 1, new QTableWidgetItem(""));


		for(int i = 0; i < sequence_list->rowCount(); i++)
		{
	    QTableWidgetItem *item = new QTableWidgetItem("");
  	  item->setFlags(item->flags() ^ Qt::ItemIsEditable);
			sequence_list->setItem(i, 2, item);
		}


    connect( sequence_list, SIGNAL( cellClicked (int, int) ),
     this, SLOT( sequenceCellClicked( int, int ) ) );

    sequence_list->setColumnWidth(0,30);
    sequence_list->setColumnWidth(TIME_COL,70);
    sequence_list->setFixedSize(QSize(210, 320));
}


void SequenceTimeWidget::generateBehaviorList()
{

    behavior_list = new QTableWidget(this);
    behavior_list->setRowCount(8);
    behavior_list->setColumnCount(2);

    QStringList m_TableHeader;
    m_TableHeader<<"Behavior"<<"Color";

    behavior_list->setHorizontalHeaderLabels(m_TableHeader);
    behavior_list->verticalHeader()->setVisible(false);
    behavior_list->setEditTriggers(QAbstractItemView::NoEditTriggers);
    behavior_list->setSelectionBehavior(QAbstractItemView::SelectItems);
    behavior_list->setSelectionMode(QAbstractItemView::SingleSelection);
    behavior_list->setShowGrid(true);
    behavior_list->setStyleSheet("QTableView {selection-background-color: blue;}");

    behavior_list->setItem(0, 0, new QTableWidgetItem("Rendezvous"));
    behavior_list->setItem(1, 0, new QTableWidgetItem("Antirendezvous"));
    behavior_list->setItem(2, 0, new QTableWidgetItem("Flock East"));
    behavior_list->setItem(3, 0, new QTableWidgetItem("Flock North"));
    behavior_list->setItem(4, 0, new QTableWidgetItem("Flock West"));
    behavior_list->setItem(5, 0, new QTableWidgetItem("Flock South"));
    behavior_list->setItem(6, 0, new QTableWidgetItem("Line X"));
    behavior_list->setItem(7, 0, new QTableWidgetItem("Line Y"));
    


    for(int i = 0; i < (int) behavior_color_array.size(); i++)
    {
        QTableWidgetItem* item = new QTableWidgetItem("");
        if(QColor::isValidColor(behavior_color_array[i]))
            item->setBackgroundColor(QColor(behavior_color_array[i]));
        behavior_list->setItem(i, 1, item);
    }


    connect( behavior_list, SIGNAL( cellClicked (int, int) ),
     this, SLOT( behaviorCellClicked( int, int ) ) );

    behavior_list->setFixedSize(QSize(200, 320));


}

void SequenceTimeWidget::sequenceCellClicked(int row, int col)
{
    curr_selected_sequence_row = row;
    qInfo() << "curr_selected_sequence_Row is updated to" << row;

    /*For debugging*/
    qInfo() << sequence_list->item(row, col)->text();
    
		custom_messages::R2D r2d;
    r2d.stamp = ros::Time::now();
    r2d.id = "";
    r2d.map_number = "";
    r2d.iteration = "";
    r2d.event_type = 2;
    r2d.description = "Sequence table clicked! Item is "; 
    if(col == TIME_COL)
      r2d.description += "Duration for (row,col) (" + std::to_string(row) + "," + std::to_string(col) + ")";
    else
      r2d.description += sequence_list->item(row, col)->text().toStdString();
    r2d.switchtime_string = "";
    r2d.sequence_string = "";
    r2d.cost_of_path = "";


    id_publisher.publish(r2d);
    ros::spinOnce();

    return;
}

void SequenceTimeWidget::behaviorCellClicked(int row, int col)
{
    curr_selected_behavior = behavior_list->item(row, 0)->text();
    qInfo() << "Row = " << row << "Col = " << col;
    qInfo() << behavior_list->item(row,0)->text();
    qInfo() << "curr_selected_behavior is" << curr_selected_behavior;

   //send r2d data
    custom_messages::R2D r2d;
    r2d.stamp = ros::Time::now();
    r2d.id = "";
    r2d.map_number = "";
    r2d.iteration = "";
    r2d.event_type = 2; //trajectory information
    r2d.description = "Behavior table clicked! Item is ";
    r2d.description += curr_selected_behavior.toStdString();
    r2d.switchtime_string = "";
    r2d.sequence_string = "";
    r2d.cost_of_path = "";


    id_publisher.publish(r2d);
    ros::spinOnce();



    return;
}

void SequenceTimeWidget::addButtonClicked()
{
    //TODO: retrieve clicked row on behavior_list
    //      write that content onto the sequence_list - need to remember where to write it
    qInfo() << "inside addButtonClicked";
    qInfo() << "curr_selected_behavior is" << curr_selected_behavior;
    int empty_row = 0;
    qInfo() << sequence_list->item(empty_row, SEQ_COL)->text();
    while(sequence_list->item(empty_row, SEQ_COL)->text() != "") {
      if(empty_row == sequence_list->rowCount() - 1) return;
			empty_row++;
    }
			
    qInfo() << "finished while loop empty_row is " << empty_row;
    sequence_list->item(empty_row, SEQ_COL)->setText(curr_selected_behavior);
    qInfo() << "updated sequence_list";
    sequence_array.push_back(curr_selected_behavior);
    qInfo() << "updated sequence_array";


    custom_messages::R2D r2d;
    r2d.stamp = ros::Time::now();
    r2d.id = "";
    r2d.map_number = "";
    r2d.iteration = "";
    r2d.event_type = 2; //trajectory information
    r2d.description = "Add button clicked! Item is ";
    r2d.description += curr_selected_behavior.toStdString();
    r2d.switchtime_string = "";
    r2d.sequence_string = "";
    r2d.cost_of_path = "";


    id_publisher.publish(r2d);
    ros::spinOnce();



    return;
}

void SequenceTimeWidget::deleteButtonClicked()
{
    //TODO: retrieve clicked row on the sequence_list
    //      erase the content on that row and move everything below that row up
    qInfo() << "inside deleteButtonClicked";
    qInfo() << "curr_selected_sequence_row is " << curr_selected_sequence_row;
    int clear_row = curr_selected_sequence_row;
    if( clear_row >= sequence_array.size() ) return;
    QString selected_behavior = sequence_array[clear_row];
 
    sequence_array.erase(sequence_array.begin() + clear_row);

    for(int i = clear_row; i < sequence_list->rowCount(); i++)
    {
        if(i < (int) sequence_array.size())
            sequence_list->item(i, SEQ_COL)->setText(sequence_array[i]);
        else
            sequence_list->item(i, SEQ_COL)->setText("");
    }
 
    custom_messages::R2D r2d;
    r2d.stamp = ros::Time::now();
    r2d.id = "";
    r2d.map_number = "";
    r2d.iteration = "";
    r2d.event_type = 2; //trajectory information
    r2d.description = "Delete button clicked! Item is ";
    r2d.description += selected_behavior.toStdString();
    r2d.switchtime_string = "";
    r2d.sequence_string = "";
    r2d.cost_of_path = "";


    id_publisher.publish(r2d);
    ros::spinOnce();

    return;



}

//"-5" means sequence modified by user so wrong input
QString SequenceTimeWidget::getSequence()
{
		qInfo() << "Inside get sequence function";
    QString text = "";
    for (int i = 0; i < (int) sequence_array.size(); i++)
    {
				qInfo() << "Inside Loop" << i;
        int index = std::find(behavior_array.begin(), behavior_array.end(),
                              sequence_array[i]) - behavior_array.begin();
				if(index == behavior_array.size()) return QString("-5");
        text += behavior_array_short[index];
        if(i != (int) sequence_array.size() - 1) text += ",";
    }

    return text;
}


//"-1" signifies one or more inputs not a number
//"-2" signifies time summation over 60
QString SequenceTimeWidget::getSwitchTime()
{
    qInfo() << "Inside get switchtime...";
    QString text = "";
    float cumulative_time = 0;
    for(int i = 0; i < (int) sequence_list->rowCount(); i++)
    {
				if(sequence_list->item(i,TIME_COL)->text() == "") break;
        bool ok;
        float duration = sequence_list->item(i,TIME_COL)->text().toFloat(&ok);
        qInfo() << "duration is" << duration;
        if(ok == 0){
          return QString("-1");
        }
        cumulative_time += sequence_list->item(i,TIME_COL)->text().toFloat();
        text += QString::number(cumulative_time);
        text += ",";
    }
    qInfo() << "made text is " << text;
    text.remove(text.size() - 1, 1);
		if(cumulative_time > 60) return QString("-2");
    return text;
}

void SequenceTimeWidget::reset()
{
    sequence_array.clear();
    int rows = sequence_list->rowCount();
    for(int i = 0; i < rows; i++)
    {
        sequence_list->item(i,TIME_COL)->setText("");
        sequence_list->item(i,SEQ_COL)->setText("");
    }
}

void SequenceTimeWidget::setSequence(std::vector<QString> sequence)
{
  sequence_array.clear();
  for(int i = 0; i < sequence.size(); i++)
  {
    sequence_list->item(i, SEQ_COL)->setText(sequence[i]);
    sequence_array.push_back(sequence[i]);
  }
  return;
}


void SequenceTimeWidget::setPublisher(ros::Publisher id_publisher)
{
  this->id_publisher = id_publisher;
  return;
}



