#pragma once
#include "TopoDS_Shape.hxx"
#include <vector>

struct YZData
{
	int m_n;		//ÐÇÐÍ½ÇÊý
	double m_L;		//
	double m_L0;	//
	double m_D;		//
	double m_e;		//
	double m_l;		//
	double m_r;		//
	double m_a;		//
	double m_r1;	//
	double m_b;		//
	double m_lmd;	// ¦Ë

	std::vector<double> m_InletColor;
	std::vector<double> m_OutletColor;

};

class ClassYZModeling
{
public:
	ClassYZModeling();
	ClassYZModeling(YZData tYZData);
	~ClassYZModeling() {};

	bool GenerateModeling();
	TopoDS_Shape GetShape() { return m_Shape; }

private:
	YZData mYZData;
	TopoDS_Shape m_Shape;

};

