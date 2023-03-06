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

class iogIgesReader
{
public:
	iogIgesReader(const char * pFilename);
	~iogIgesReader();

	bool ReadIGES();
	bool GetIGESFile();

private:
	char * m_pFileName;
	double m_dHoleRad;
	double m_nUnit;
	int m_nDiscreType;
	double m_dLinearDelVal;
	double m_dAngleDelVal;
	double m_dSewingTol;
};

