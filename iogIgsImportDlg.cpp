//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/14
//
//=============================================================================
#include "iogIgsImportDlg.h"
#include "ui_iogIgsImportDlg.h"

#define Igs_Import_MaxHoleArea   "iog_param/parameter/import_igs/holearea"
#define Igs_Import_MaxFilletRad   "iog_param/parameter/import_igs/filletrad"
#define Igs_Import_Unit   "iog_param/parameter/import_igs/unit"
#define Igs_Import_UnitType   "iog_param/parameter/import_igs/unittype"
#define Igs_Import_DiscreType   "iog_param/parameter/import_igs/discretype"
#define Igs_Import_DiscreLinear   "iog_param/parameter/import_igs/discretlinear"
#define Igs_Import_DiscreAngle   "iog_param/parameter/import_igs/discretangle"
#define Igs_Import_SewingTolerance   "iog_param/parameter/import_igs/sewingtolerance"

///////////////////////////////////////////////////////////////////////////////
//
iogIgsImportDlg::iogIgsImportDlg(QPopDialog *parent) : QPopDialog(parent)
{
	ui = new Ui::iogIgsImportDlg();
	ui->setupUi(this);

	this->InitIgsDlg();

	connect(ui->cmbDiscre, SIGNAL(currentIndexChanged(int)), this, SLOT(ChangeIdx()));
	connect(ui->btnOk, SIGNAL(clicked()), this, SLOT(OnOk()));
	connect(ui->btnCancel, SIGNAL(clicked()), this, SLOT(OnCancel()));
}

iogIgsImportDlg::~iogIgsImportDlg()
{
	delete ui;
}
///////////////////////////////////////////////////////////////////////////////
//

void iogIgsImportDlg::InitIgsDlg()
{
	this->SetTips();
	this->SetValid();

	int nLengthSubs = ConfigGetSubUnitSize(0);
	for (int i = 0; i < nLengthSubs; ++i)
	{
		const char * pLengthName = ConfigGetUnitSubName(0, i);
		QString strLengthName(pLengthName);
		ui->cmbUnits->addItem(strLengthName);
	}

	int nCurLength = ConfigGetUnitCurrentValue(0);//
	ui->cmbUnits->setCurrentIndex(0);

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

void iogIgsImportDlg::ChangeIdx()
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

void iogIgsImportDlg::ReadXMLStlPara()
{
	GetXmlValueD(Igs_Import_MaxHoleArea, m_dHoleRad);

	if (m_dHoleRad == 0)
	{
		ui->holeArea->setText("0.0");
	}
	else
		ui->holeArea->setText(QString::number(m_dHoleRad, 'f', 6));
}

bool iogIgsImportDlg::IsEffect()
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

	int nUnitSelIndex = ui->cmbUnits->currentIndex();
	m_nUnitType = nUnitSelIndex;
	int nCurLength = ConfigGetUnitCurrentValue(0);

	if (nUnitSelIndex == nCurLength)
	{
		m_dLenTransFac = 1;
	}

	double dCurMeter = ConfigGetUnitFactor(0, nCurLength);//current unit to meter
	double dSelMeter = ConfigGetUnitFactor(0, nUnitSelIndex);//select unit to meter
	m_dLenTransFac = dSelMeter / dCurMeter;

	m_nDiscreType = ui->cmbDiscre->currentIndex();

	return true;
}

void iogIgsImportDlg::SetTips()
{
	QString strHoleTol = QObject::tr("Auto fill hole radius");
	QString strFilletTol = QObject::tr("Maxium allowed auto remove Fillet radius");
	QString strUnit = QObject::tr("Units");
	QString strOk = QObject::tr("OK");
	QString strCancel = QObject::tr("Cancel");
	QString strGrid = QObject::tr("Quality of the grid");

	//ui->filletTolerance->setToolTip(strFilletTol);
	ui->holeTolerance->setToolTip(strHoleTol);
	ui->labelUnits->setToolTip(strUnit);
	ui->btnCancel->setToolTip(strCancel);
	ui->btnOk->setToolTip(strOk);
	ui->labelDiscre->setToolTip(strGrid);
}

void iogIgsImportDlg::SetValid()
{
	QRegExp regExp1("^0?|(?!0\\d)\\d+(\\.\\d*)?((E|e)(\\+|\\-)?)?\\d{0,2}$");
	ui->holeArea->setValidator(new QRegExpValidator(regExp1, this));
}

void iogIgsImportDlg::WriteXMLPara()
{
	SetXmlValueD(Igs_Import_MaxFilletRad, m_dFilletRad);
	SetXmlValueD(Igs_Import_MaxHoleArea, m_dHoleRad);
	SetXmlValueD(Igs_Import_Unit, m_dLenTransFac);
	SetXmlValueD(Igs_Import_DiscreLinear, m_dLinearDelVal);
	SetXmlValueD(Igs_Import_DiscreAngle, m_dAngleDelVal);
	SetXmlValueD(Igs_Import_SewingTolerance, m_dSewVal);

	SetXmlValueN(Igs_Import_UnitType, m_nUnitType);
	SetXmlValueN(Igs_Import_DiscreType, m_nDiscreType);
}
///////////////////////////////////////////////////////////////////////////////
//
void iogIgsImportDlg::OnOk()
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

void iogIgsImportDlg::OnCancel()
{
	QPopDialog::reject();
}

double iogIgsImportDlg::GetAngleDelVal()
{
	return m_dAngleDelVal;
}

double iogIgsImportDlg::GetLinearDelVal()
{
	return m_dLinearDelVal;
}
