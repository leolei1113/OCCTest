//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/05
//
//=============================================================================
#pragma once
#include "string.h"
#include "stdlib.h"

#include "iogModule.h"
#include "iogOccInclude.h"
#include "iogBasicTools.h"


class iogStpReader
{
public:
	iogStpReader(const char * pFilename);
	~iogStpReader();

	bool ReadSTP();
	bool GetSTPFile();

private:
	char * m_pFileName;
	double m_dTolerance;
	double m_nUnit;
	double m_dHoleRad;
	double m_dFilletRad;

	int m_nDiscreType;
	double m_dLinearDelVal;
	double m_dAngleDelVal;
	double m_dSewingTol;
};

