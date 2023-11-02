#pragma once

#include <QtWidgets/QDialog>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include "ui_interface.h"

class interface : public QDialog
{
	Q_OBJECT

public:
	interface(QString& strInput, QString& output, QWidget *parent = Q_NULLPTR);
	~interface();

public slots:
	void OnBtnOk(); 
	void OnBtnBrowse1();
	void OnBtnBrowse2();

private:
	Ui::interfaceClass ui;

public:
	QString in;
	QString out;
	double rate;
	double threshold;
};
