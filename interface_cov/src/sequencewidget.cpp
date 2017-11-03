#include <QtWidgets>
#include <QApplication>
#include "sequencewidget.h"

SequenceWidget::SequenceWidget(QWidget *parent) : QWidget( parent )
{
    sequence_label = new QLabel("Input Sequence:", this);
    sequence_text = new QLineEdit("ex)n,s,w", this); /* sequence_text->text() -- retrieves the text
                                       sequence_text->setText("adsfadf") -- sets the text */

    sequence_view = new QTableView(this);
    QStandardItemModel* model = new QStandardItemModel(this);
    model->setHorizontalHeaderLabels(
                QStringList() << QApplication::translate("nestedlayouts", "Sequence Name")
                              << QApplication::translate("nestedlayouts", "Abbrev")
                              << QApplication::translate("nestedlayouts", "Color"));
    QList<QStringList> rows = QList<QStringList>()
        << (QStringList() << "Flocking " << "i" << "cyan")
        << (QStringList() << "Flock North" << "n" << "green")
        << (QStringList() << "Flock South" << "s" << "magenta")
        << (QStringList() << "Flock East" << "e" << "gray")
        << (QStringList() << "Flock West" << "w" << "light gray")
        << (QStringList() << "Rendezvous" << "r" << "dark cyan")
        << (QStringList() << "Anti Rendezvous" << "a" << "dark magneta");

    int count_i = rows.count();
    for (int i = 0; i < count_i; i++)
    {
      QStringList row = rows.at(i);
      QList<QStandardItem *> items;
      int count_j = row.count();
      for (int j = 0; j < count_j; j++)
      {
        QString text = row.at(j);
        items.append(new QStandardItem(text));
      }
      model->appendRow(items);
    }
    sequence_view->setModel(model);
    sequence_view->verticalHeader()->hide();
    sequence_view->horizontalHeader()->setStretchLastSection(true);




    QHBoxLayout* sequence_layouth = new QHBoxLayout();
    sequence_layouth->addWidget(sequence_label);
    sequence_layouth->addWidget(sequence_text);

    QVBoxLayout* sequence_layoutv = new QVBoxLayout();
    sequence_layoutv->addLayout(sequence_layouth);
    sequence_layoutv->addWidget(sequence_view);

    this->setLayout(sequence_layoutv);
}



QString SequenceWidget::getInfo()
{
    return sequence_text->text();
}


void SequenceWidget::reset()
{
    sequence_text->setText("ex)n,s,w");
    return;
}







