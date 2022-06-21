#include "TubularHeatExchanger.h"

#include <Geom_TrimmedCurve.hxx>
#include <Geom_OffsetCurve.hxx>

#include <GC_MakeArcOfCircle.hxx>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Compound.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>

#include <BRep_Builder.hxx>

#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Dir.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

CCylinder::CCylinder()
{
	mDistance = 0;
	mLength = 0;
	mDiameter = 0;
	mThick = 0;
}

CCylinder::~CCylinder()
{
}


CNozzle::CNozzle()
{
	mAxisAngle = 0;
	mTengAngle = 0;
}

CNozzle::CNozzle(double tAxisAngle, double tTengAngle, double tDistance, double tLength, double tDiameter, double tThick)
{
	mAxisAngle = tAxisAngle;
	mTengAngle = tTengAngle;
	mDistance = tDistance;
	mLength = tLength;
	mDiameter = tDiameter;
	mThick = tThick;
}

CNozzle::~CNozzle()
{
}


CStationaryHead::CStationaryHead()
{
	mType = tCircular;
	mDirection = tLeft;
	mNozzleNum = 0;
	//mInletNozzle = new CNozzle();
	//mOutletNozzle = new CNozzle();
}

CStationaryHead::CStationaryHead(
	StationaryHeadType tType, 
	Direction tDirection, 
	double tDistance, 
	double tLength, 
	double tDiameter, 
	double tThick, 
	int tNozzleNum, 
	CNozzle* tInletNozzle, 
	CNozzle* tOutletNozzle)
{
	mType = tType;
	mDirection = tDirection;
	mDistance = tDistance;
	mLength = tLength;
	mDiameter = tDiameter;
	mThick = tThick;
	mNozzleNum = tNozzleNum;

	if (tInletNozzle != NULL)
	{
		mInletNozzle = tInletNozzle;
	}

	if (tOutletNozzle != NULL)
	{
		mOutletNozzle = tOutletNozzle;
	}
}

CStationaryHead::~CStationaryHead()
{

}

TopoDS_Shape CStationaryHead::GenerateStationaryHead()
{
	Standard_Real SHDistance = mDistance;
	Standard_Real SHDiameter = mDiameter;
	Standard_Real SHThick = mThick;
	Standard_Real SHLength = mLength;
	Standard_Real SHInletNozzAxisAng = mInletNozzle->mAxisAngle;
	Standard_Real SHInletNozzTenAng = mInletNozzle->mTengAngle;

	if (mType == tCircular)
	{
		SHLength = mDiameter / 2.0;
	}

	gp_Pnt SHPnt1, SHPnt2, SHPnt3, SHPnt4, SHPnt5, SHPnt6;

	SHPnt1 = gp_Pnt(SHDistance, 0, SHDiameter / 2.0);
	SHPnt2 = gp_Pnt(SHDistance + SHLength, 0, 0);

	Standard_Real Len = sqrt(1.0 / (pow(sin(SHInletNozzAxisAng * M_PI / 180) / SHLength, 2)
		+ pow(cos(SHInletNozzAxisAng * M_PI / 180) / (SHDiameter / 2.0), 2)));

	SHPnt3 = gp_Pnt(SHDistance + Len * sin(SHInletNozzAxisAng * M_PI / 180), 0, Len * cos(SHInletNozzAxisAng * M_PI / 180));


	Handle(Geom_TrimmedCurve) anArc1 = GC_MakeArcOfCircle(SHPnt1, SHPnt3, SHPnt2);
	TopoDS_Edge InsideArc = BRepBuilderAPI_MakeEdge(anArc1);

	//Handle(Geom_OffsetCurve) OC = new Geom_OffsetCurve((Handle_Geom_Curve)anArc1, SHThick);

	//Standard_Boolean result = OC->IsCN(2);


	//TopoDS_Edge Edge1 = BRepBuilderAPI_MakeEdge(SHPnt1, SHPnt4);
	//TopoDS_Edge Edge2 = BRepBuilderAPI_MakeEdge(SHPnt2, SHPnt5);

	//TopoDS_Wire aWire = BRepBuilderAPI_MakeWire(InsideArc, Edge1, OutsideArc, Edge2);


	//TopoDS_Face myFaceProfile = BRepBuilderAPI_MakeFace(aWire);
	//gp_Ax1 xAxis = gp::OX();
	//TopoDS_Shape myBody = BRepPrimAPI_MakeRevol(myFaceProfile, xAxis);





	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);
	aBuilder.Add(aRes, InsideArc);


	return aRes;

}

CTubularHeatExchanger::CTubularHeatExchanger()
{
}


CTubularHeatExchanger::~CTubularHeatExchanger()
{
}
