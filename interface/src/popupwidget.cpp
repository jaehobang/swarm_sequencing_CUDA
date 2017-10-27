#include <QtWidgets>
#include "popupwidget.h"




PopupWidget::PopupWidget(QWidget *parent) : QWidget( parent )
{
    //popup->setAttribute(Qt::WA_DeleteOnClose);
    name_val = "";
    isAided = 3;
    name_ = new QLabel("ID:");
    name = new QLineEdit(); 
    aid_but = new QPushButton("Aided");
    unaid_but = new QPushButton("Unaided");
    onlyaid_but = new QPushButton("Only Aided");
    connect(aid_but, SIGNAL(released()), this, SLOT(setIsAided()));
    connect(unaid_but, SIGNAL(released()), this, SLOT(setIsUnaided()));
    connect(onlyaid_but, SIGNAL(released()), this, SLOT(setIsOnlyAided()));

    QHBoxLayout* row1 = new QHBoxLayout();
    QHBoxLayout* row2 = new QHBoxLayout();
	QVBoxLayout* col = new QVBoxLayout();

    row1->addWidget(name_);
    row1->addWidget(name);
    row2->addWidget(aid_but);
    row2->addWidget(unaid_but);
    row2->addWidget(onlyaid_but);

    col->addLayout(row1);
    col->addLayout(row2);
    this->setLayout(col);

}

void PopupWidget::setIsAided(){
    isAided = 1;
    name_val = name->text();
    this->widgetClosed();
    this->close();
    return;
}

void PopupWidget::setIsUnaided(){
    isAided = 0;
    name_val = name->text();
    this->widgetClosed();
    this->close();
    return;
}

void PopupWidget::setIsOnlyAided(){
    isAided = 2;
    name_val = name->text();
    this->widgetClosed();
    this->close();
    return;
}

int PopupWidget::getIsAided()
{
    return this->isAided;
}

QString PopupWidget::getName()
{
    return this->name_val;
}


