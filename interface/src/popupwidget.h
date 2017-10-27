#ifndef POPUPWIDGET_H
#define POPUPWIDGET_H

#include <QtWidgets>

class PopupWidget : public QWidget
{
    Q_OBJECT

public:
    PopupWidget(QWidget *parent = 0); //3 minutes per map
    int getIsAided();
    QString getName();


Q_SIGNALS:
		void widgetClosed();

private Q_SLOTS:
    void setIsAided();
    void setIsUnaided();
    void setIsOnlyAided();

private:
	QLabel* name_;
    QLineEdit* name;
    QPushButton* aid_but;
    QPushButton* unaid_but;
    QPushButton* onlyaid_but;
    QString name_val;
    int isAided;

};


#endif // POPUPWIDGET_H
