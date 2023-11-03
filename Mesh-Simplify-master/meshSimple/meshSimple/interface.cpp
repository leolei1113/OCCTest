#include "interface.h"

interface::interface(QString& strInput, QString& output, QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_3, SIGNAL(clicked()), this, SLOT(OnBtnOk()));
	connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(OnBtnBrowse1()));
	connect(ui.pushButton_2, SIGNAL(clicked()), this, SLOT(OnBtnBrowse2()));
	ui.lineEdit_3->setText("0.1");
	ui.lineEdit_4->setText("10");
	this->exec();
}

interface::~interface()
{

}

void interface::OnBtnOk()
{
	rate = ui.lineEdit_3->text().toDouble();
	threshold = ui.lineEdit_4->text().toDouble();
	this->close();
}

void interface::OnBtnBrowse1()
{
	in = QFileDialog::getOpenFileName(this, QObject::tr("Select file"), "/", "OBJ Files(*.obj)");
	ui.lineEdit->setText(in);
}

void interface::OnBtnBrowse2()
{
	out = QFileDialog::getSaveFileName(this, QObject::tr("Save path"), "/", "OBJ Files(*.obj)");
	ui.lineEdit_2->setText(out);
}