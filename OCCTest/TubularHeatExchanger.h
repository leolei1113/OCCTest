#pragma once
#include "TopoDS_Shape.hxx"



enum StationaryHeadType
{
	tCyliner = 0,
	tCircular,
	tEllipse,
	tParabolic
};

enum Direction
{
	tLeft = 0,
	tRight
};



//圆管
class CCylinder
{
public:
	CCylinder();
	~CCylinder();

public:
	double mDistance;
	double mLength;
	double mDiameter;
	double mThick;

public:
	double GetDistance() { return mDistance; }
	double GetLength() { return mLength; }
	double GetDiameter() { return mDiameter; }
	double GetThick() { return mThick; }

	bool SetDistance(double Distance) {mDistance = Distance; }
	bool SetLength(double Length) {mLength = Length; }
	bool SetDiameter(double Diameter) {mDiameter = Diameter; }
	bool SetThick(double Thick) {mThick = Thick; }
};


//外接口
class CNozzle :public CCylinder
{
public:
	CNozzle();
	CNozzle(
		double tAxisAngle,
		double tTengAngle,
		double tDistance,
		double tLength,
		double tDiameter,
		double tThick);
	~CNozzle();

public:
	double mAxisAngle;
	double mTengAngle;
};

//管箱
class CStationaryHead : public CCylinder
{
public:
	CStationaryHead();
	CStationaryHead(
		StationaryHeadType tType,
		Direction tDirection,
		double tDistance,
		double tLength,
		double tDiameter,
		double tThick,
		int tNozzleNum,
		CNozzle* tInletNozzle,
		CNozzle* tOutletNozzle);
	~CStationaryHead();

public:
	StationaryHeadType mType;
	Direction mDirection;
	int mNozzleNum;
	CNozzle* mInletNozzle;
	CNozzle* mOutletNozzle;

public:
	StationaryHeadType GetType() { return mType; }
	Direction GetDirection() { return mDirection; }
	int GetNozzleNum() { return mNozzleNum; }

	bool SetType(StationaryHeadType SHType) { mType = SHType; }
	bool SetDirection(Direction Direct) { mDirection = Direct; }
	bool SetNozzleNum(int NozzNum) { mNozzleNum = NozzNum; }

	TopoDS_Shape GenerateStationaryHead();


};


//管式换热器
class CTubularHeatExchanger
{
public:
	CTubularHeatExchanger();
	~CTubularHeatExchanger();

};

