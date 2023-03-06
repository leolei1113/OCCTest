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

namespace Ui { class iogStpImportDlg; };

class iogStpImportDlg : public QPopDialog
{
	Q_OBJECT

public:
	iogStpImportDlg(QPopDialog *parent = Q_NULLPTR);
	~iogStpImportDlg();

	void InitStpDlg();
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
	Ui::iogStpImportDlg* ui;

	double m_dHoleRad;
	double m_dFilletRad;
	double m_dLenTransFac;
	int m_nUnitType;
	int m_nDiscreType;

	double m_dLinearDelVal;
	double m_dAngleDelVal;
	double m_dSewVal;
};
