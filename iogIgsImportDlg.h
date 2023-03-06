//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/14
//
//=============================================================================
#pragma once

#include "core.h"
#include "comobj.h"
#include "appinterface.h"
#include "qui.h"
#include "config.h"

namespace Ui { class iogIgsImportDlg; };

class iogIgsImportDlg : public QPopDialog
{
	Q_OBJECT

public:
	iogIgsImportDlg(QPopDialog *parent = Q_NULLPTR);
	~iogIgsImportDlg();

	void InitIgsDlg();

	double GetLinearDelVal();
	double GetAngleDelVal();

protected:
	void SetTips();
	void SetValid();
	bool IsEffect();
	void ReadXMLStlPara();
	void WriteXMLPara();
private slots:
	void OnOk();
	void OnCancel();
	void ChangeIdx();

private:
	Ui::iogIgsImportDlg* ui;

	double m_dHoleRad;
	double m_dFilletRad;
	double m_dLenTransFac;
	int m_nUnitType;
	int m_nDiscreType;

	double m_dLinearDelVal;
	double m_dAngleDelVal;
	double m_dSewVal;
};
