//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/14
//
//=============================================================================
#include "iogStpImportDlg.h"
#include "ui_iogStpImportDlg.h"

#define Stp_Import_MaxHoleArea   "iog_param/parameter/import_stp/holearea"
#define Stp_Import_MaxFilletRad   "iog_param/parameter/import_stp/filletrad"
#define Stp_Import_Unit   "iog_param/parameter/import_stp/unit"
#define Stp_Import_UnitType "iog_param/parameter/import_stp/unittype"
#define Stp_Import_DiscreType "iog_param/parameter/import_stp/discretype"
#define Stp_Import_DiscreLinear   "iog_param/parameter/import_stp/discretlinear"
#define Stp_Import_DiscreAngle   "iog_param/parameter/import_stp/discretangle"
#define Stp_Import_SewingTolerance   "iog_param/parameter/import_stp/sewingtolerance"

///////////////////////////////////////////////////////////////////////////////
//
iogStpImportDlg::iogStpImportDlg(QPopDialog *parent)
	: QPopDialog(parent)
{
	ui = new Ui::iogStpImportDlg();
	ui->setupUi(this);

	this->InitStpDlg();

	//ui->filletTolerance->setVisible(false);
	//ui->filletRad->setVisible(false);

	connect(ui->cmbDiscre, SIGNAL(currentIndexChanged(int)), this, SLOT(ChangeIdx()));
	connect(ui->btnOk, SIGNAL(clicked()), this, SLOT(OnOk()));
	connect(ui->btnCancel, SIGNAL(clicked()), this, SLOT(OnCancel()));
}

iogStpImportDlg::~iogStpImportDlg()
{
	delete ui;
}

///////////////////////////////////////////////////////////////////////////////
//

void iogStpImportDlg::InitStpDlg()
{
	this->SetTips();
	this->SetValid();

	/*int nLengthSubs = ConfigGetSubUnitSize(0);
	for (int i = 0; i < nLengthSubs; ++i)
	{
		const char * pLengthName = ConfigGetUnitSubName(0, i);
		QString strLengthName(pLengthName);
		ui->cmbUnits->addItem(strLengthName);
	}
	ui->cmbUnits->setCurrentIndex(2);*/

	ui->cmbDiscre->addItem("Extreme Rough");
	ui->cmbDiscre->addItem("Rough");
	ui->cmbDiscre->addItem("Medium");
	ui->cmbDiscre->addItem("Fine");
	ui->cmbDiscre->addItem("Extreme Fine");
	ui->cmbDiscre->setCurrentIndex(2);

	ui->LinearDelVal->setText("0.004");
	ui->AngleDelVal->setText("0.22");

	ui->sewTol->setText("0.1");

	this->ReadXMLStlPara();
}

void iogStpImportDlg::ChangeIdx()
{
	if (ui->cmbDiscre->currentIndex() == 0)
	{
		ui->LinearDelVal->setText("0.016");
		ui->AngleDelVal->setText("0.88");

		//ui->LinearDelVal->setDisabled(true);
		//ui->AngleDelVal->setDisabled(true);
	}
	else if (ui->cmbDiscre->currentIndex() == 1)
	{
		ui->LinearDelVal->setText("0.008");
		ui->AngleDelVal->setText("0.44");

		//ui->LinearDelVal->setDisabled(true);
		//ui->AngleDelVal->setDisabled(true);
	}
	else if (ui->cmbDiscre->currentIndex() == 2)
	{
		ui->LinearDelVal->setText("0.004");
		ui->AngleDelVal->setText("0.22");

		//ui->LinearDelVal->setDisabled(true);
		//ui->AngleDelVal->setDisabled(true);
	}
	else if (ui->cmbDiscre->currentIndex() == 3)
	{
		ui->LinearDelVal->setText("0.002");
		ui->AngleDelVal->setText("0.11");

		//ui->LinearDelVal->setDisabled(true);
		//ui->AngleDelVal->setDisabled(true);
	}
	else if (ui->cmbDiscre->currentIndex() == 4)
	{
		ui->LinearDelVal->setText("0.001");
		ui->AngleDelVal->setText("0.06");

		//ui->LinearDelVal->setDisabled(true);
		//ui->AngleDelVal->setDisabled(true);
	}
}

void iogStpImportDlg::ReadXMLStlPara()
{
	GetXmlValueD(Stp_Import_MaxHoleArea, m_dHoleRad);

	if (m_dHoleRad == 0)
	{
		ui->holeArea->setText("0.0");
	}
	else
		ui->holeArea->setText(QString::number(m_dHoleRad, 'f', 6));
}

bool iogStpImportDlg::IsEffect()
{
	QString strTol = ui->holeArea->text();

	bool bTrans = false;
	double dTol = strTol.toDouble(&bTrans);

	m_dHoleRad = dTol;

	QString strLinearDelVal = ui->LinearDelVal->text();
	m_dLinearDelVal = strLinearDelVal.toDouble(&bTrans);

	QString strAngleDelVal = ui->AngleDelVal->text();
	m_dAngleDelVal = strAngleDelVal.toDouble(&bTrans);

	QString strSewVal = ui->sewTol->text();
	m_dSewVal = strSewVal.toDouble(&bTrans);

	//int nUnitSelIndex = ui->cmbUnits->currentIndex();
	//m_nUnitType = nUnitSelIndex;
	//int nCurLength = ConfigGetUnitCurrentValue(0);

	//if (nUnitSelIndex == nCurLength)
	//{
	//	m_dLenTransFac = 1;
	//	return true;
	//}

	//double dCurMeter = ConfigGetUnitFactor(0, nCurLength);//current unit to meter
	//double dSelMeter = ConfigGetUnitFactor(0, nUnitSelIndex);//select unit to meter
	//m_dLenTransFac = dSelMeter / dCurMeter;

	m_nDiscreType = ui->cmbDiscre->currentIndex();
	return true;
}

void iogStpImportDlg::SetTips()
{
	QString strHoleTol = QObject::tr("Auto fill hole radius");
	QString strFilletTol = QObject::tr("Maxium allowed auto remove Fillet radius");
	QString strUnit = QObject::tr("Units");
	QString strOk = QObject::tr("OK");
	QString strCancel = QObject::tr("Cancel");
	QString strGrid = QObject::tr("Quality of the grid");

	//ui->filletTolerance->setToolTip(strFilletTol);
	ui->holeTolerance->setToolTip(strHoleTol);
	//ui->labelUnits->setToolTip(strUnit);
	ui->btnCancel->setToolTip(strCancel);
	ui->btnOk->setToolTip(strOk);
	ui->labelDiscre->setToolTip(strGrid);
}

void iogStpImportDlg::SetValid()
{
	QRegExp regExp1("^0?|(?!0\\d)\\d+(\\.\\d*)?((E|e)(\\+|\\-)?)?\\d{0,2}$");
	ui->holeArea->setValidator(new QRegExpValidator(regExp1, this));
}

void iogStpImportDlg::WriteXMLPara()
{
	SetXmlValueD(Stp_Import_MaxFilletRad, m_dFilletRad);
	SetXmlValueD(Stp_Import_MaxHoleArea, m_dHoleRad);

	SetXmlValueD(Stp_Import_DiscreLinear, m_dLinearDelVal);
	SetXmlValueD(Stp_Import_DiscreAngle, m_dAngleDelVal);
	SetXmlValueD(Stp_Import_SewingTolerance, m_dSewVal);
	//SetXmlValueD(Stp_Import_Unit, m_dLenTransFac);
	SetXmlValueN(Stp_Import_UnitType, m_nUnitType);
	SetXmlValueN(Stp_Import_DiscreType, m_nDiscreType);
}
///////////////////////////////////////////////////////////////////////////////
//
void iogStpImportDlg::OnOk()
{
	bool bEffect = IsEffect();
	if (bEffect)
	{
		this->WriteXMLPara();

		QPopDialog::accept();
	}
	else
	{
		QString strMsg = QObject::tr("The file was imported failed for invalid parameter");
		TheExe.PrintMessage(strMsg, apiExe::MSG_ERROR);

		QPopDialog::reject();
	}
}

void iogStpImportDlg::OnCancel()
{
	QPopDialog::reject();
}

double iogStpImportDlg::GetAngleDelVal()
{
	return m_dAngleDelVal;
}

double iogStpImportDlg::GetLinearDelVal()
{
	return m_dLinearDelVal;
}