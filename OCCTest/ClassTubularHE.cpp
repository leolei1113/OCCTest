#include "ClassTubularHE.h"
#include "OCCBasicTools.h"

#include <Geom_TrimmedCurve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Surface.hxx>

#include <GC_MakeArcOfCircle.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Compound.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>


#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>

#include <BRepOffsetAPI_MakePipe.hxx>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Section.hxx>

#include <BRep_Builder.hxx>
#include <BRepAdaptor_Surface.hxx>

#include <BRepTools.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

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
#include <gp_Circ.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Torus.hxx>

#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>
#include <STEPControl_Controller.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI_Writer.hxx>

#include <TopExp_Explorer.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <Poly_Triangulation.hxx>

#include <Vector>


ClassTubularHE::ClassTubularHE()
{
	mTurbineSide = 0;

	mTSHEData.mDtin = 0;
	mTSHEData.mTtin = 0;
	mTSHEData.mAtin = 0;
	mTSHEData.mHtin = 0;
	mTSHEData.mDtout = 0;
	mTSHEData.mTtout = 0;
	mTSHEData.mAtout = 0;
	mTSHEData.mHtout = 0;
	mTSHEData.mdt = 0;
	mTSHEData.mTt = 0;
	mTSHEData.mNt = 0;
	mTSHEData.mLt = 0;
	mTSHEData.mLtin = 0;
	mTSHEData.mLtout = 0;
	mTSHEData.mRt = 0;
	mTSHEData.mTE = 0;
	mTSHEData.mlE = 0;

	mTSHEData.mDsin = 0;
	mTSHEData.mTsin = 0;
	mTSHEData.mBsin = 0;
	mTSHEData.mHsin = 0;
	mTSHEData.mDsout = 0;
	mTSHEData.mTsout = 0;
	mTSHEData.mBsout = 0;
	mTSHEData.mHsout = 0;
	mTSHEData.mTzlb = 0;
	mTSHEData.mhqk = 0;
	mTSHEData.mLzlb = 0;
	mTSHEData.mTgb = 0;
	mTSHEData.mLs = 0;
	mTSHEData.mTs = 0;
	mTSHEData.mRs = 0;
	mTSHEData.mDs = 0;
	mTSHEData.mDzlb = 0;
	mTSHEData.mDkk = 0;
}

ClassTubularHE::ClassTubularHE(int tTurbineSide, TSHEData tTSHEData, const std::string& ExportPath)
{
	mTurbineSide = tTurbineSide;
	mExportPath = ExportPath;

	mTSHEData.mNt		= tTSHEData.mNt;	
	mTSHEData.mDtin	= tTSHEData.mDtin	;
	mTSHEData.mTtin	= tTSHEData.mTtin	;
	mTSHEData.mAtin	= tTSHEData.mAtin	;
	mTSHEData.mHtin	= tTSHEData.mHtin	;
	mTSHEData.mDtout	= tTSHEData.mDtout;	
	mTSHEData.mTtout	= tTSHEData.mTtout;	
	mTSHEData.mAtout	= tTSHEData.mAtout;	
	mTSHEData.mHtout	= tTSHEData.mHtout;	
	mTSHEData.mdt		= tTSHEData.mdt	;	
	mTSHEData.mTt		= tTSHEData.mTt	;	
	mTSHEData.mLt		= tTSHEData.mLt	;	
	mTSHEData.mLtin	= tTSHEData.mLtin	;
	mTSHEData.mLtout	= tTSHEData.mLtout;	
	mTSHEData.mRt		= tTSHEData.mRt	;	
	mTSHEData.mTE		= tTSHEData.mTE	;	
	mTSHEData.mlE		= tTSHEData.mlE	;	
	mTSHEData.mDsin	= tTSHEData.mDsin	;
	mTSHEData.mTsin	= tTSHEData.mTsin	;
	mTSHEData.mBsin	= tTSHEData.mBsin	;
	mTSHEData.mHsin	= tTSHEData.mHsin	;
	mTSHEData.mDsout	= tTSHEData.mDsout;	
	mTSHEData.mTsout	= tTSHEData.mTsout;	
	mTSHEData.mBsout	= tTSHEData.mBsout;	
	mTSHEData.mHsout	= tTSHEData.mHsout;	
	mTSHEData.mTzlb	= tTSHEData.mTzlb	;
	mTSHEData.mhqk	= tTSHEData.mhqk	;
	mTSHEData.mLzlb	= tTSHEData.mLzlb	;
	mTSHEData.mTgb	= tTSHEData.mTgb	;
	mTSHEData.mLs		= tTSHEData.mLs	;	
	mTSHEData.mTs		= tTSHEData.mTs	;	
	mTSHEData.mRs		= tTSHEData.mRs	;	
	mTSHEData.mDs		= tTSHEData.mDs	;	
	mTSHEData.mDzlb	= tTSHEData.mDzlb	;
	mTSHEData.mDkk	= tTSHEData.mDkk	;


	mTSHEData.mNzlb = (mTSHEData.mLs - mTSHEData.mBsin - mTSHEData.mDsin / 2.0 - mTSHEData.mBsout - mTSHEData.mDsout / 2.0 - mTSHEData.mTzlb - 1) / mTSHEData.mLzlb;

	if (mTurbineSide == 1)
	{
		mTSHEData.mNlayer = (mTSHEData.mRs - 1 - mTSHEData.mTt - mTSHEData.mdt / 2.0) / mTSHEData.mLt;
		mTSHEData.mNtmax = 3 * mTSHEData.mNlayer * mTSHEData.mNlayer + 3 * mTSHEData.mNlayer + 1;
	}
	if (mTurbineSide == 2)
	{
		mTSHEData.mNlayer = (mTSHEData.mRs - 1 - mTSHEData.mlE / 2.0 - mTSHEData.mTt - mTSHEData.mdt / 2.0) / mTSHEData.mLt;
		mTSHEData.mNtmax = 3 * mTSHEData.mNlayer * mTSHEData.mNlayer + 5 * mTSHEData.mNlayer + 2;
	}

	if (mTSHEData.mNt > mTSHEData.mNtmax)
	{
		mTSHEData.mNt = mTSHEData.mNtmax;
	}
}


ClassTubularHE::~ClassTubularHE()
{
}


Standard_Boolean DesignTubineConfiguration(int tTurbineSide, TSHEData& tTSHEData, 
	std::vector<gp_Pnt>& TubePosition, std::vector<gp_Pnt>& LGPosition)
{

	std::vector<gp_Pnt> SecPosition;



	if (tTurbineSide == 1)
	{
		TubePosition.push_back(gp_Pnt(0, 0, 0));

		int RestNum = (tTSHEData.mNt - 1) % 6;
		int SectionNum = (tTSHEData.mNt - 1) / 6.0;
		tTSHEData.mNlayer = ceil(sqrt((4 * tTSHEData.mNt - 1) / 12.0) - 0.5);


		int NN = 1;
		int TransStep1 = NN;
		int TransStep2 = 0;
		gp_Vec TransVec1(0, tTSHEData.mLt, 0);
		gp_Vec TransVec2(0, -0.5 * tTSHEData.mLt, 0.5 * sqrt(3.0) * tTSHEData.mLt);

		for (int i = 0; i < SectionNum; ++i)
		{
			TransStep1 = NN;

			SecPosition.push_back(gp_Pnt(0, 0, 0).Translated(TransVec1 * TransStep1).Translated(TransVec2 * TransStep2));

			++TransStep2;

			if (i + 1 == 0.5 * (NN + 1) * NN)
			{
				++NN;
				TransStep2 = 0;
			}
		}
		TransStep1 = NN;

		gp_Pnt RestPnt = gp_Pnt(0, 0, 0).Translated(TransVec1 * TransStep1).Translated(TransVec2 * TransStep2);


		for (int i = 0; i < 6; ++i)
		{
			for (int j = 0; j < SecPosition.size(); ++j)
			{

				TubePosition.push_back(SecPosition[j].Rotated(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), i * 60 * M_PI / 180));
			}


			if (i < RestNum)
			{
				TubePosition.push_back(RestPnt.Rotated(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), i * 60 * M_PI / 180));
			}
		}

		int tNlayer = (int)(sqrt((4 * tTSHEData.mNt - 1) / 12.0) - 0.5);
		LGPosition.push_back(gp_Pnt(0, 0, 0).Translated(gp_Vec(0, 0.5 * tTSHEData.mLt, 0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
		LGPosition.push_back(gp_Pnt(0, 0, 0).Translated(gp_Vec(0, -0.5 * tTSHEData.mLt, 0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
		LGPosition.push_back(gp_Pnt(0, 0, 0).Translated(gp_Vec(0, -0.5 * tTSHEData.mLt, -0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
		LGPosition.push_back(gp_Pnt(0, 0, 0).Translated(gp_Vec(0, 0.5 * tTSHEData.mLt, -0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
	}

	if (tTurbineSide == 2)
	{
		int HalfNt = (tTSHEData.mNt + 1) / 2.0;
		tTSHEData.mNlayer = ceil((sqrt(12 * tTSHEData.mNt + 1) - 5) / 6.0);
		int SectionNum = (HalfNt - (tTSHEData.mNlayer + 1)) / 3.0;
		int RestNum = (HalfNt - (tTSHEData.mNlayer + 1)) % 3;

		int NN = 1;
		int TransStep1 = NN;
		int TransStep2 = 0;
		gp_Vec TransVec1(0, 0.5 * tTSHEData.mLt, 0.5 * sqrt(3.0) * tTSHEData.mLt);
		gp_Vec TransVec2(0, 0.5 * tTSHEData.mLt, -0.5 * sqrt(3.0) * tTSHEData.mLt);


		for (int i = 0; i <= tTSHEData.mNlayer; ++i)
		{
			TubePosition.push_back(gp_Pnt(0, 0 + tTSHEData.mLt * i, tTSHEData.mlE / 2.0));
		}


		for (int i = 0; i < SectionNum; ++i)
		{
			TransStep1 = NN;

			SecPosition.push_back(gp_Pnt(0, 0, tTSHEData.mlE / 2.0).Translated(TransVec1 * TransStep1).Translated(TransVec2 * TransStep2));

			++TransStep2;

			if (i + 1 == 0.5 * (NN + 1) * NN)
			{
				++NN;
				TransStep2 = 0;
			}
		}
		TransStep1 = NN;

		gp_Pnt RestPnt = gp_Pnt(0, 0, tTSHEData.mlE / 2.0).Translated(TransVec1 * TransStep1).Translated(TransVec2 * TransStep2);

		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < SecPosition.size(); ++j)
			{

				TubePosition.push_back(SecPosition[j].Rotated(gp_Ax1(gp_Pnt(0, 0, tTSHEData.mlE / 2.0), gp_Dir(1, 0, 0)), i * 60 * M_PI / 180));
			}


			if (i < RestNum)
			{
				TubePosition.push_back(RestPnt.Rotated(gp_Ax1(gp_Pnt(0, 0, tTSHEData.mlE / 2.0), gp_Dir(1, 0, 0)), i * 60 * M_PI / 180));
			}
		}

		int tNlayer = (int)((sqrt(12 * tTSHEData.mNt + 1) - 5) / 6.0);
		LGPosition.push_back(gp_Pnt(0, 0, tTSHEData.mlE / 2.0).Translated(gp_Vec(0, 0.5 * tTSHEData.mLt, 0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
		LGPosition.push_back(gp_Pnt(0, 0, tTSHEData.mlE / 2.0).Translated(gp_Vec(0, -0.5 * tTSHEData.mLt, 0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
		//LGPosition.push_back(gp_Pnt(0, 0, -tTSHEData.mlE / 2.0).Translated(gp_Vec(0, -0.5 * tTSHEData.mLt, -0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
		//LGPosition.push_back(gp_Pnt(0, 0, -tTSHEData.mlE / 2.0).Translated(gp_Vec(0, 0.5 * tTSHEData.mLt, -0.5 * sqrt(3.0) * tTSHEData.mLt) * tNlayer));
	}

	return Standard_True;

}

TopoDS_Shape GenerateLeftHead(int tTurbineSide, TSHEData tTSHEData)
{
	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);

	gp_Pnt LHPnt0(-tTSHEData.mLs / 2.0, 0, 0);

	gp_Pnt LHPnt1(-tTSHEData.mLs / 2.0, 0, tTSHEData.mRs);
	gp_Pnt LHPnt2(-tTSHEData.mLs / 2.0 - tTSHEData.mRs, 0, 0);
	gp_Pnt LHPnt3(-tTSHEData.mLs / 2.0 - tTSHEData.mRs * cos(tTSHEData.mAtin * M_PI / 180), 0, tTSHEData.mRs * sin(tTSHEData.mAtin * M_PI / 180));

	gp_Pnt LHPnt4(-tTSHEData.mLs / 2.0, 0, tTSHEData.mRs + tTSHEData.mTs);
	gp_Pnt LHPnt5(-tTSHEData.mLs / 2.0 - tTSHEData.mRs - tTSHEData.mTs, 0, 0);
	gp_Pnt LHPnt6(-tTSHEData.mLs / 2.0 - (tTSHEData.mRs + tTSHEData.mTs) * 
		cos(tTSHEData.mAtin * M_PI / 180), 0, (tTSHEData.mRs + tTSHEData.mTs) * sin(tTSHEData.mAtin * M_PI / 180));

	Handle(Geom_TrimmedCurve) anArc1 = GC_MakeArcOfCircle(LHPnt1, LHPnt3, LHPnt2);
	TopoDS_Edge Arc1 = BRepBuilderAPI_MakeEdge(anArc1);
	Handle(Geom_TrimmedCurve) anArc2 = GC_MakeArcOfCircle(LHPnt4, LHPnt6, LHPnt5);
	TopoDS_Edge Arc2 = BRepBuilderAPI_MakeEdge(anArc2);

	TopoDS_Edge Edge1 = BRepBuilderAPI_MakeEdge(LHPnt1, LHPnt4);
	TopoDS_Edge Edge2 = BRepBuilderAPI_MakeEdge(LHPnt2, LHPnt5);

	TopoDS_Wire Wire1 = BRepBuilderAPI_MakeWire(Arc1, Edge1, Arc2, Edge2);

	TopoDS_Face Face1 = BRepBuilderAPI_MakeFace(Wire1);
	gp_Ax1 xAxis = gp::OX();
	TopoDS_Shape LeftHeadBody = BRepPrimAPI_MakeRevol(Face1, xAxis);

	gp_Dir InLetDir(-cos(tTSHEData.mAtin * M_PI / 180), 0, sin(tTSHEData.mAtin * M_PI / 180));
	gp_Vec TmpVec = InLetDir;
	Standard_Real Len = sqrt(pow(tTSHEData.mRs + tTSHEData.mTs, 2) - pow(tTSHEData.mDtin / 2.0 + tTSHEData.mTtin, 2)) + tTSHEData.mHtin;
	gp_Pnt InLetPnt = gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0).Translated(TmpVec * Len);
	gp_Circ InLetCir1(gp_Ax2(InLetPnt, InLetDir), tTSHEData.mDtin / 2.0 + tTSHEData.mTtin);
	gp_Circ InLetCir2(gp_Ax2(InLetPnt, InLetDir), tTSHEData.mDtin / 2.0);

	TopoDS_Edge InLetEdge1 = BRepBuilderAPI_MakeEdge(InLetCir1);
	TopoDS_Edge InLetEdge2 = BRepBuilderAPI_MakeEdge(InLetCir2);
	
	TopoDS_Wire InLetWire1 = BRepBuilderAPI_MakeWire(InLetEdge1);
	TopoDS_Wire InLetWire2 = BRepBuilderAPI_MakeWire(InLetEdge2);
	
	TopoDS_Face InLetFace1 = BRepBuilderAPI_MakeFace(InLetWire1);
	TopoDS_Face InLetFace2 = BRepBuilderAPI_MakeFace(InLetWire2);

	TopoDS_Shape InLetBody1 = BRepPrimAPI_MakePrism(InLetFace1, -TmpVec * Len) ;
	TopoDS_Shape InLetBody2 = BRepPrimAPI_MakePrism(InLetFace2, -TmpVec * Len);

	TopoDS_Shape OutLetBody1;
	TopoDS_Shape OutLetBody2;
	TopoDS_Shape DevideBody;

	if (tTurbineSide == 2)
	{
		gp_Dir OutLetDir(-cos(tTSHEData.mAtout * M_PI / 180), 0, -sin(tTSHEData.mAtout * M_PI / 180));
		gp_Vec TmpVec = OutLetDir;
		Standard_Real Len = sqrt(pow(tTSHEData.mRs + tTSHEData.mTs, 2) - pow(tTSHEData.mDtout / 2.0 + tTSHEData.mTtout, 2)) + tTSHEData.mHtout;
		gp_Pnt OutLetPnt = gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0).Translated(TmpVec * Len);
		gp_Circ OutLetCir1(gp_Ax2(OutLetPnt, OutLetDir), tTSHEData.mDtout / 2.0 + tTSHEData.mTtout);
		gp_Circ OutLetCir2(gp_Ax2(OutLetPnt, OutLetDir), tTSHEData.mDtout / 2.0);

		TopoDS_Edge OutLetEdge1 = BRepBuilderAPI_MakeEdge(OutLetCir1);
		TopoDS_Edge OutLetEdge2 = BRepBuilderAPI_MakeEdge(OutLetCir2);

		TopoDS_Wire OutLetWire1 = BRepBuilderAPI_MakeWire(OutLetEdge1);
		TopoDS_Wire OutLetWire2 = BRepBuilderAPI_MakeWire(OutLetEdge2);

		TopoDS_Face OutLetFace1 = BRepBuilderAPI_MakeFace(OutLetWire1);
		TopoDS_Face OutLetFace2 = BRepBuilderAPI_MakeFace(OutLetWire2);

		OutLetBody1 = BRepPrimAPI_MakePrism(OutLetFace1, -TmpVec * Len);
		OutLetBody2 = BRepPrimAPI_MakePrism(OutLetFace2, -TmpVec * Len);

		Handle(Geom_TrimmedCurve) DevideAnCir = GC_MakeArcOfCircle(gp_Pnt(-tTSHEData.mLs / 2.0, tTSHEData.mRs + tTSHEData.mTs / 2.0, -tTSHEData.mTE / 2.0),
			gp_Pnt(-tTSHEData.mLs / 2.0 - tTSHEData.mRs - tTSHEData.mTs / 2.0, 0, -tTSHEData.mTE / 2.0), 
			gp_Pnt(-tTSHEData.mLs / 2.0, -tTSHEData.mRs - tTSHEData.mTs / 2.0, -tTSHEData.mTE / 2.0));
		TopoDS_Edge DivideArc = BRepBuilderAPI_MakeEdge(DevideAnCir);
		TopoDS_Edge DivideLine = BRepBuilderAPI_MakeEdge(gp_Pnt(-tTSHEData.mLs / 2.0, tTSHEData.mRs + tTSHEData.mTs / 2.0, -tTSHEData.mTE / 2.0),
			gp_Pnt(-tTSHEData.mLs / 2.0, -tTSHEData.mRs - tTSHEData.mTs / 2.0, -tTSHEData.mTE / 2.0));
		TopoDS_Wire DevideWire = BRepBuilderAPI_MakeWire(DivideArc, DivideLine);
		TopoDS_Face DevideFace = BRepBuilderAPI_MakeFace(DevideWire);
		DevideBody = BRepPrimAPI_MakePrism(DevideFace, gp_Vec(0, 0, tTSHEData.mTE));

		BRepPrimAPI_MakeSphere AnToolSphere(gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0), tTSHEData.mRs);
		TopoDS_Shape ToolSphere = AnToolSphere.Shape();

		DevideBody = BRepAlgoAPI_Common(DevideBody, ToolSphere);
		//aBuilder.Add(aRes, DevideBody);
	}



	TopoDS_Shape ToolSphere = BRepPrimAPI_MakeSphere(LHPnt0, tTSHEData.mRs).Shape();

	LeftHeadBody = BRepAlgoAPI_Fuse(LeftHeadBody, InLetBody1);
	if (tTurbineSide == 2)
	{
		LeftHeadBody = BRepAlgoAPI_Fuse(LeftHeadBody, OutLetBody1);
	}
	LeftHeadBody = BRepAlgoAPI_Cut(LeftHeadBody, ToolSphere);
	LeftHeadBody = BRepAlgoAPI_Cut(LeftHeadBody, InLetBody2);
	if (tTurbineSide == 2)
	{
		LeftHeadBody = BRepAlgoAPI_Cut(LeftHeadBody, OutLetBody2);
	}

	if (tTurbineSide == 2)
	{
		LeftHeadBody = BRepAlgoAPI_Fuse(LeftHeadBody, DevideBody);
	}
	aBuilder.Add(aRes, LeftHeadBody);

	return aRes;
}

TopoDS_Shape GenerateRightHead(int tTurbineSide, TSHEData tTSHEData)
{
	gp_Pnt RHPnt0(tTSHEData.mLs / 2.0, 0, 0);
		   
	gp_Pnt RHPnt1(tTSHEData.mLs / 2.0, 0, tTSHEData.mRs);
	gp_Pnt RHPnt2(tTSHEData.mLs / 2.0 + tTSHEData.mRs, 0, 0);
	gp_Pnt RHPnt3(tTSHEData.mLs / 2.0 + tTSHEData.mRs * cos(tTSHEData.mAtout * M_PI / 180), 0, tTSHEData.mRs * sin(tTSHEData.mAtout * M_PI / 180));
		   
	gp_Pnt RHPnt4(tTSHEData.mLs / 2.0, 0, tTSHEData.mRs + tTSHEData.mTs);
	gp_Pnt RHPnt5(tTSHEData.mLs / 2.0 + tTSHEData.mRs + tTSHEData.mTs, 0, 0);
	gp_Pnt RHPnt6(tTSHEData.mLs / 2.0 + (tTSHEData.mRs + tTSHEData.mTs) * 
		cos(tTSHEData.mAtout * M_PI / 180), 0, (tTSHEData.mRs + tTSHEData.mTs) * sin(tTSHEData.mAtout * M_PI / 180));

	Handle(Geom_TrimmedCurve) anArc1 = GC_MakeArcOfCircle(RHPnt1, RHPnt3, RHPnt2);
	TopoDS_Edge Arc1 = BRepBuilderAPI_MakeEdge(anArc1);
	Handle(Geom_TrimmedCurve) anArc2 = GC_MakeArcOfCircle(RHPnt4, RHPnt6, RHPnt5);
	TopoDS_Edge Arc2 = BRepBuilderAPI_MakeEdge(anArc2);

	TopoDS_Edge Edge1 = BRepBuilderAPI_MakeEdge(RHPnt1, RHPnt4);
	TopoDS_Edge Edge2 = BRepBuilderAPI_MakeEdge(RHPnt2, RHPnt5);

	TopoDS_Wire Wire1 = BRepBuilderAPI_MakeWire(Arc1, Edge1, Arc2, Edge2);

	TopoDS_Face Face1 = BRepBuilderAPI_MakeFace(Wire1);
	gp_Ax1 xAxis = gp::OX();
	TopoDS_Shape RightHeadBody = BRepPrimAPI_MakeRevol(Face1, xAxis);


	TopoDS_Shape OutLetBody1;
	TopoDS_Shape OutLetBody2;
	if (tTurbineSide == 1)
	{
		gp_Dir OutLetDir(cos(tTSHEData.mAtout * M_PI / 180), 0, sin(tTSHEData.mAtout * M_PI / 180));
		gp_Vec TmpVec = OutLetDir;
		Standard_Real Len = sqrt(pow(tTSHEData.mRs + tTSHEData.mTs, 2) - pow(tTSHEData.mDtout / 2.0 + tTSHEData.mTtout, 2)) + tTSHEData.mHtout;
		gp_Pnt OutLetPnt = gp_Pnt(tTSHEData.mLs / 2.0, 0, 0).Translated(TmpVec * Len);
		gp_Circ OutLetCir1(gp_Ax2(OutLetPnt, OutLetDir), tTSHEData.mDtout / 2.0 + tTSHEData.mTtout);
		gp_Circ OutLetCir2(gp_Ax2(OutLetPnt, OutLetDir), tTSHEData.mDtout / 2.0);

		TopoDS_Edge OutLetEdge1 = BRepBuilderAPI_MakeEdge(OutLetCir1);
		TopoDS_Edge OutLetEdge2 = BRepBuilderAPI_MakeEdge(OutLetCir2);

		TopoDS_Wire OutLetWire1 = BRepBuilderAPI_MakeWire(OutLetEdge1);
		TopoDS_Wire OutLetWire2 = BRepBuilderAPI_MakeWire(OutLetEdge2);

		TopoDS_Face OutLetFace1 = BRepBuilderAPI_MakeFace(OutLetWire1);
		TopoDS_Face OutLetFace2 = BRepBuilderAPI_MakeFace(OutLetWire2);

		OutLetBody1 = BRepPrimAPI_MakePrism(OutLetFace1, -TmpVec * Len);
		OutLetBody2 = BRepPrimAPI_MakePrism(OutLetFace2, -TmpVec * Len);
	}



	TopoDS_Shape ToolSphere = BRepPrimAPI_MakeSphere(RHPnt0, tTSHEData.mRs).Shape();

	if (tTurbineSide == 1)
	{
		RightHeadBody = BRepAlgoAPI_Fuse(RightHeadBody, OutLetBody1);
		RightHeadBody = BRepAlgoAPI_Cut(RightHeadBody, ToolSphere);;
		RightHeadBody = BRepAlgoAPI_Cut(RightHeadBody, OutLetBody2);

	}

	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);
	aBuilder.Add(aRes, RightHeadBody);

	return aRes;
}

TopoDS_Shape GenerateShell(TSHEData tTSHEData)
{

	TopoDS_Edge ShellEdge1 = BRepBuilderAPI_MakeEdge(gp_Pnt(-tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs), 
		gp_Pnt(tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs));
	TopoDS_Edge ShellEdge2 = BRepBuilderAPI_MakeEdge(gp_Pnt(-tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs), 
		gp_Pnt(-tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0));
	TopoDS_Edge ShellEdge3 = BRepBuilderAPI_MakeEdge(gp_Pnt(-tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0), 
		gp_Pnt(tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0));
	TopoDS_Edge ShellEdge4 = BRepBuilderAPI_MakeEdge(gp_Pnt(tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs), 
		gp_Pnt(tTSHEData.mLs / 2.0, 0, tTSHEData.mDs / 2.0));

	TopoDS_Wire ShellWire = BRepBuilderAPI_MakeWire(ShellEdge1, ShellEdge2, ShellEdge3, ShellEdge4);
	TopoDS_Face ShellFace = BRepBuilderAPI_MakeFace(ShellWire);

	gp_Ax1 xAxis = gp::OX();
	TopoDS_Shape ShellBody = BRepPrimAPI_MakeRevol(ShellFace, xAxis);


	gp_Pnt InLetPnt = gp_Pnt(-tTSHEData.mLs / 2.0 + tTSHEData.mBsin, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs + tTSHEData.mHsin);
	gp_Circ InLetCir1(gp_Ax2(InLetPnt, gp_Dir(0, 0, 1)), tTSHEData.mDsin / 2.0 + tTSHEData.mTsin);
	gp_Circ InLetCir2(gp_Ax2(InLetPnt, gp_Dir(0, 0, 1)), tTSHEData.mDsin / 2.0);

	TopoDS_Edge InLetEdge1 = BRepBuilderAPI_MakeEdge(InLetCir1);
	TopoDS_Edge InLetEdge2 = BRepBuilderAPI_MakeEdge(InLetCir2);

	TopoDS_Wire InLetWire1 = BRepBuilderAPI_MakeWire(InLetEdge1);
	TopoDS_Wire InLetWire2 = BRepBuilderAPI_MakeWire(InLetEdge2);

	TopoDS_Face InLetFace1 = BRepBuilderAPI_MakeFace(InLetWire1);
	TopoDS_Face InLetFace2 = BRepBuilderAPI_MakeFace(InLetWire2);

	TopoDS_Shape InLetBody1 = BRepPrimAPI_MakePrism(InLetFace1, gp_Vec(0, 0, -1) * (tTSHEData.mHsin + tTSHEData.mTs + tTSHEData.mDs / 2.0));
	TopoDS_Shape InLetBody2 = BRepPrimAPI_MakePrism(InLetFace2, gp_Vec(0, 0, -1) * (tTSHEData.mHsin + tTSHEData.mTs + tTSHEData.mDs / 2.0));

	gp_Vec OutletVec;
	gp_Pnt OutletPnt;
	if (tTSHEData.mNzlb % 2 == 0){
		OutletVec = gp_Vec(0, 0, 1);
		OutletPnt = gp_Pnt(tTSHEData.mLs / 2.0 - tTSHEData.mBsout, 0, -tTSHEData.mDs / 2.0 - tTSHEData.mTs - tTSHEData.mHsout);
	}
	else{
		OutletVec = gp_Vec(0, 0, -1);
		OutletPnt = gp_Pnt(tTSHEData.mLs / 2.0 - tTSHEData.mBsout, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs + tTSHEData.mHsout);
	}

	gp_Circ OutletCir1(gp_Ax2(OutletPnt, gp_Dir(0, 0, 1)), tTSHEData.mDsout / 2.0 + tTSHEData.mTsout);
	gp_Circ OutletCir2(gp_Ax2(OutletPnt, gp_Dir(0, 0, 1)), tTSHEData.mDsout / 2.0);

	TopoDS_Edge OutletEdge1 = BRepBuilderAPI_MakeEdge(OutletCir1);
	TopoDS_Edge OutletEdge2 = BRepBuilderAPI_MakeEdge(OutletCir2);

	TopoDS_Wire OutletWire1 = BRepBuilderAPI_MakeWire(OutletEdge1);
	TopoDS_Wire OutletWire2 = BRepBuilderAPI_MakeWire(OutletEdge2);

	TopoDS_Face OutletFace1 = BRepBuilderAPI_MakeFace(OutletWire1);
	TopoDS_Face OutletFace2 = BRepBuilderAPI_MakeFace(OutletWire2);

	TopoDS_Shape OutletBody1 = BRepPrimAPI_MakePrism(OutletFace1, OutletVec * (tTSHEData.mHsout + tTSHEData.mTs + tTSHEData.mDs / 2.0));
	TopoDS_Shape OutletBody2 = BRepPrimAPI_MakePrism(OutletFace2, OutletVec * (tTSHEData.mHsout + tTSHEData.mTs + tTSHEData.mDs / 2.0));

	gp_Circ AnToolCirc(gp_Ax2(gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0), gp_Dir(1, 0, 0)), tTSHEData.mDs / 2.0);
	TopoDS_Edge ToolCircEdge = BRepBuilderAPI_MakeEdge(AnToolCirc);
	TopoDS_Wire ToolCircWire = BRepBuilderAPI_MakeWire(ToolCircEdge);
	TopoDS_Face ToolCircFace = BRepBuilderAPI_MakeFace(ToolCircWire);
	TopoDS_Shape ToolCylinder = BRepPrimAPI_MakePrism(ToolCircFace, gp_Vec(tTSHEData.mLs, 0, 0));

	ShellBody = BRepAlgoAPI_Fuse(ShellBody, InLetBody1);
	ShellBody = BRepAlgoAPI_Fuse(ShellBody, OutletBody1);

	ShellBody = BRepAlgoAPI_Cut(ShellBody, InLetBody2);
	ShellBody = BRepAlgoAPI_Cut(ShellBody, OutletBody2);
	ShellBody = BRepAlgoAPI_Cut(ShellBody, ToolCylinder);


	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);
	aBuilder.Add(aRes, ShellBody);


	return aRes;


}

TopoDS_Shape GenerateTubes(int tTurbineSide, std::vector<gp_Pnt> TubePosition, std::vector<gp_Pnt> LGPosition, TSHEData tTSHEData)
{
	
	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);

	if (tTurbineSide == 1)
	{
		for (int i = 0; i < TubePosition.size(); ++i)
		{
			Standard_Boolean IsSame = Standard_False;
			for (int j = 0; j < LGPosition.size(); ++j)
			{
				if (LGPosition[j].Distance(TubePosition[i]) < 0.01)
				{
					IsSame = Standard_True;
					break;
				}
				
			}

			TopoDS_Shape TubeBody;
			if (IsSame == Standard_True)
			{
				gp_Pnt TmpPnt = TubePosition[i];
				TmpPnt.SetX(-tTSHEData.mLs / 2.0);

				gp_Circ TmpCir(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mDkk / 2.0);
				TopoDS_Edge TmpEdge = BRepBuilderAPI_MakeEdge(TmpCir);
				TopoDS_Wire TmpWire = BRepBuilderAPI_MakeWire(TmpEdge);
				TopoDS_Face TmpFace = BRepBuilderAPI_MakeFace(TmpWire);
				TubeBody = BRepPrimAPI_MakePrism(TmpFace, gp_Vec(tTSHEData.mLs, 0, 0));

			}
			else
			{
				gp_Pnt TmpPnt = TubePosition[i];
				TmpPnt.SetX(-tTSHEData.mLs / 2.0 - tTSHEData.mLtin);

				gp_Circ TmpCir1(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mdt / 2.0);
				gp_Circ TmpCir2(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mdt / 2.0 + tTSHEData.mTt);
				TopoDS_Edge TmpEdge1 = BRepBuilderAPI_MakeEdge(TmpCir1);
				TopoDS_Edge TmpEdge2 = BRepBuilderAPI_MakeEdge(TmpCir2);
				TopoDS_Wire TmpWire1 = BRepBuilderAPI_MakeWire(TmpEdge1);
				TopoDS_Wire TmpWire2 = BRepBuilderAPI_MakeWire(TmpEdge2);

				TopoDS_Face TmpFace1 = BRepBuilderAPI_MakeFace(TmpWire1);
				TopoDS_Face TmpFace2 = BRepBuilderAPI_MakeFace(TmpWire2);
				TopoDS_Shape TmpFace = BRepAlgoAPI_Cut(TmpFace2, TmpFace1);

				TubeBody = BRepPrimAPI_MakePrism(TmpFace, gp_Vec(tTSHEData.mLs + tTSHEData.mLtin + tTSHEData.mLtout, 0, 0));
			}
			aBuilder.Add(aRes, TubeBody);
		}
	}

	if (tTurbineSide == 2)
	{

		double Len = sqrt(pow(tTSHEData.mDs / 2.0, 2) - pow((tTSHEData.mNlayer + 1) * tTSHEData.mLt + 0.5 * tTSHEData.mdt + tTSHEData.mTt + 0.5 * tTSHEData.mlE, 2));

		double deltaLs = tTSHEData.mLs + Len - (tTSHEData.mRt + (tTSHEData.mNlayer + 1) * tTSHEData.mLt * sqrt(3.0) / 2.0 + 0.5 * tTSHEData.mdt + tTSHEData.mTt);


		for (int i = 0; i < TubePosition.size(); ++i)
		{

			Standard_Boolean IsSame = Standard_False;
			for (int j = 0; j < LGPosition.size(); ++j)
			{
				if (LGPosition[j].Distance(TubePosition[i]) < 0.01)
				{
					IsSame = Standard_True;
					break;
				}

			}
			if (IsSame == Standard_True)
			{
				gp_Pnt TmpPnt = TubePosition[i];
				TmpPnt.SetX(-tTSHEData.mLs / 2.0);
				gp_Circ TmpCir(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mDkk / 2.0);
				TopoDS_Edge TmpEdge = BRepBuilderAPI_MakeEdge(TmpCir);
				TopoDS_Wire TmpWire = BRepBuilderAPI_MakeWire(TmpEdge);
				TopoDS_Face TmpFace = BRepBuilderAPI_MakeFace(TmpWire);

				Standard_Real Len = (tTSHEData.mNzlb - 1) * (tTSHEData.mLzlb + tTSHEData.mTzlb);
				TopoDS_Shape TubeBody = BRepPrimAPI_MakePrism(TmpFace,
					gp_Vec(tTSHEData.mLs / 2.0 + Len / 2.0 + tTSHEData.mTzlb / 2.0, 0, 0));
				TopoDS_Shape CopyTubeBody = BRepBuilderAPI_Copy(TubeBody);
				gp_Trsf Trsf_Translate;
				Trsf_Translate.SetTranslation(gp_Vec(0, 0, -2 * TmpPnt.Z()));
				CopyTubeBody = BRepBuilderAPI_Transform(CopyTubeBody, Trsf_Translate);

				aBuilder.Add(aRes, TubeBody);
				aBuilder.Add(aRes, CopyTubeBody);
			}
			else
			{
				gp_Pnt PathStartPnt = TubePosition[i];
				PathStartPnt.SetX(-tTSHEData.mLs / 2.0 - tTSHEData.mLtin);
				gp_Pnt PathMidPnt = TubePosition[i].Rotated(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), M_PI_2).Translated(
					gp_Vec(-tTSHEData.mlE / 2.0 - tTSHEData.mLs / 2.0 + deltaLs + tTSHEData.mRt, 0, 0));
				gp_Pnt PathEndPnt = PathStartPnt;
				PathEndPnt.SetZ(-PathStartPnt.Z());


				gp_Circ TmpCir1(gp_Ax2(PathStartPnt, gp_Dir(1, 0, 0)), tTSHEData.mdt / 2.0);
				gp_Circ TmpCir2(gp_Ax2(PathStartPnt, gp_Dir(1, 0, 0)), tTSHEData.mdt / 2.0 + tTSHEData.mTt);
				TopoDS_Edge TmpEdge1 = BRepBuilderAPI_MakeEdge(TmpCir1);
				TopoDS_Edge TmpEdge2 = BRepBuilderAPI_MakeEdge(TmpCir2);
				TopoDS_Wire TmpWire1 = BRepBuilderAPI_MakeWire(TmpEdge1);
				TopoDS_Wire TmpWire2 = BRepBuilderAPI_MakeWire(TmpEdge2);

				TopoDS_Face TmpFace1 = BRepBuilderAPI_MakeFace(TmpWire1);
				TopoDS_Face TmpFace2 = BRepBuilderAPI_MakeFace(TmpWire2);

				TopoDS_Shape TmpFace = BRepAlgoAPI_Cut(TmpFace2, TmpFace1);

				//path
				TopoDS_Wire HalfPath;
				TopTools_ListOfShape PathEdges;

				gp_Trsf Trans_Mirror;
				Trans_Mirror.SetMirror(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)));
				if (TubePosition[i].Z() < tTSHEData.mRt)
				{
					gp_Pnt PathCirStartPnt(PathMidPnt.X() - TubePosition[i].Z(), TubePosition[i].Y(), TubePosition[i].Z());
					gp_Pnt PathCirEndPnt(PathMidPnt.X() - TubePosition[i].Z(), TubePosition[i].Y(), -TubePosition[i].Z());

					Handle(Geom_TrimmedCurve) AnArc = GC_MakeArcOfCircle(PathCirStartPnt, PathMidPnt, PathCirEndPnt);
					TopoDS_Edge PathArc = BRepBuilderAPI_MakeEdge(AnArc);
					TopoDS_Edge PathStartLine = BRepBuilderAPI_MakeEdge(PathStartPnt, PathCirStartPnt);
					TopoDS_Edge PathEndLine = BRepBuilderAPI_MakeEdge(PathCirEndPnt, PathEndPnt);

					PathEdges.Append(PathStartLine);
					PathEdges.Append(PathArc);
					PathEdges.Append(PathEndLine);


				}
				else
				{
					gp_Pnt PathCir1StartPnt(PathMidPnt.X() - tTSHEData.mRt, TubePosition[i].Y(), TubePosition[i].Z());
					gp_Pnt PathCir1EndPnt(PathMidPnt.X(), TubePosition[i].Y(), TubePosition[i].Z() - tTSHEData.mRt);

					gp_Pnt PathCir2StartPnt(PathMidPnt.X(), TubePosition[i].Y(), -TubePosition[i].Z() + tTSHEData.mRt);
					gp_Pnt PathCir2EndPnt(PathMidPnt.X() - tTSHEData.mRt, TubePosition[i].Y(), -TubePosition[i].Z());

					Handle(Geom_TrimmedCurve) AnArc1 = GC_MakeArcOfCircle(PathCir1StartPnt, gp_Pnt(PathMidPnt.X() + (1 / sqrt(2) - 1) * tTSHEData.mRt,
						TubePosition[i].Y(), TubePosition[i].Z() + (1 / sqrt(2) - 1) * tTSHEData.mRt), PathCir1EndPnt);
					TopoDS_Edge PathArc1 = BRepBuilderAPI_MakeEdge(AnArc1);
					Handle(Geom_TrimmedCurve) AnArc2 = GC_MakeArcOfCircle(PathCir2StartPnt, gp_Pnt(PathMidPnt.X() + (1 / sqrt(2) - 1) * tTSHEData.mRt,
						TubePosition[i].Y(), -TubePosition[i].Z() - (1 / sqrt(2) - 1) * tTSHEData.mRt), PathCir2EndPnt);
					TopoDS_Edge PathArc2 = BRepBuilderAPI_MakeEdge(AnArc2);

					TopoDS_Edge PathStartLine = BRepBuilderAPI_MakeEdge(PathStartPnt, PathCir1StartPnt);
					TopoDS_Edge PathMidLine = BRepBuilderAPI_MakeEdge(PathCir1EndPnt, PathCir2StartPnt);
					TopoDS_Edge PathEndLine = BRepBuilderAPI_MakeEdge(PathCir2EndPnt, PathEndPnt);

					PathEdges.Append(PathStartLine);
					PathEdges.Append(PathArc1);
					PathEdges.Append(PathMidLine);
					PathEdges.Append(PathArc2);
					PathEdges.Append(PathEndLine);

				}

				BRepBuilderAPI_MakeWire aTubePath;
				aTubePath.Add(PathEdges);
				TopoDS_Wire TubePath = TopoDS::Wire(aTubePath.Shape());
				TopoDS_Shape TmpTube = BRepOffsetAPI_MakePipe(TubePath, TmpFace);

				aBuilder.Add(aRes, TmpTube);
			}
		}
	}


	return aRes;

}

TopoDS_Shape GenerateBulkhead(int tTurbineSide, std::vector<gp_Pnt> TubePosition, TSHEData tTSHEData)
{
	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);



	gp_Circ BHCir(gp_Ax2(gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0), gp_Dir(1, 0, 0)), tTSHEData.mDs / 2.0);

	TopoDS_Edge BHEdge = BRepBuilderAPI_MakeEdge(BHCir);

	TopoDS_Wire BHWire = BRepBuilderAPI_MakeWire(BHEdge);

	TopoDS_Face BHFace = BRepBuilderAPI_MakeFace(BHWire);

	TopoDS_Shape BHBody = BRepPrimAPI_MakePrism(BHFace, gp_Vec(tTSHEData.mTgb, 0, 0));

	if (tTurbineSide == 1)
	{

		for (int i = 0; i < TubePosition.size(); ++i)
		{
			gp_Pnt TmpPnt = TubePosition[i];
			TmpPnt.SetX(-tTSHEData.mLs / 2.0 - 5);
			gp_Circ TmpCir(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mdt / 2.0 + tTSHEData.mTt);
			TopoDS_Edge TmpEdge = BRepBuilderAPI_MakeEdge(TmpCir);
			TopoDS_Wire TmpWire = BRepBuilderAPI_MakeWire(TmpEdge);
			TopoDS_Face TmpFace = BRepBuilderAPI_MakeFace(TmpWire);
			TopoDS_Shape TmpBody = BRepPrimAPI_MakePrism(TmpFace, gp_Vec(5 + tTSHEData.mTgb * 2, 0, 0));

			BHBody = BRepAlgoAPI_Cut(BHBody, TmpBody);
		}

		gp_Trsf Trans_Translate;
		Trans_Translate.SetTranslation(gp_Vec(tTSHEData.mLs - tTSHEData.mTgb, 0, 0));
		TopoDS_Shape BHBody2 = BRepBuilderAPI_Transform(BHBody, Trans_Translate);

		aBuilder.Add(aRes, BHBody);
		aBuilder.Add(aRes, BHBody2);
	}

	if (tTurbineSide == 2)
	{
		for (int i = 0; i < TubePosition.size(); ++i)
		{
			gp_Pnt TmpPnt = TubePosition[i];
			TmpPnt.SetX(-tTSHEData.mLs / 2.0 - 5);
			gp_Circ TmpCir(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mdt / 2.0 + tTSHEData.mTt);
			TopoDS_Edge TmpEdge = BRepBuilderAPI_MakeEdge(TmpCir);
			TopoDS_Wire TmpWire = BRepBuilderAPI_MakeWire(TmpEdge);
			TopoDS_Face TmpFace = BRepBuilderAPI_MakeFace(TmpWire);
			TopoDS_Shape TmpBody = BRepPrimAPI_MakePrism(TmpFace, gp_Vec(5 + tTSHEData.mTgb * 2, 0, 0));

			BHBody = BRepAlgoAPI_Cut(BHBody, TmpBody);

			gp_Trsf Transf_Mirror;
			Transf_Mirror.SetMirror(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)));
			TopoDS_Shape TmpMirrorBody = BRepBuilderAPI_Transform(TmpBody, Transf_Mirror);
			BHBody = BRepAlgoAPI_Cut(BHBody, TmpMirrorBody);


		}
		aBuilder.Add(aRes, BHBody);


	}

	return aRes;

}

TopoDS_Shape GenerateBaffles(int tTurbineSide, std::vector<gp_Pnt> TubePosition, TSHEData tTSHEData)
{
	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);

	Standard_Real Len = (tTSHEData.mNzlb - 1) * (tTSHEData.mLzlb + tTSHEData.mTzlb);

	gp_Circ BaffCir(gp_Ax2(gp_Pnt(-Len / 2.0 - tTSHEData.mTzlb / 2.0, 0, 0), gp_Dir(1, 0, 0)), tTSHEData.mDzlb / 2.0);

	TopoDS_Edge BaffEdge = BRepBuilderAPI_MakeEdge(BaffCir);

	TopoDS_Wire BaffWire = BRepBuilderAPI_MakeWire(BaffEdge);

	TopoDS_Face BaffFace = BRepBuilderAPI_MakeFace(BaffWire);

	TopoDS_Shape BaffBody = BRepPrimAPI_MakePrism(BaffFace, gp_Vec(tTSHEData.mTzlb, 0, 0));

	gp_Circ ToolCir(gp_Ax2(gp_Pnt(-Len / 2.0, 0, -tTSHEData.mDzlb / 2.0 + tTSHEData.mhqk), gp_Dir(0, 0, 1)), tTSHEData.mDzlb / 2.0);

	TopoDS_Edge ToolEdge = BRepBuilderAPI_MakeEdge(ToolCir);

	TopoDS_Wire ToolWire = BRepBuilderAPI_MakeWire(ToolEdge);

	TopoDS_Face ToolFace = BRepBuilderAPI_MakeFace(ToolWire);

	TopoDS_Shape ToolBody = BRepPrimAPI_MakePrism(ToolFace, gp_Vec(0, 0, -tTSHEData.mhqk * 2.0));

	BaffBody = BRepAlgoAPI_Cut(BaffBody, ToolBody);
	aBuilder.Add(aRes, BaffBody);

	//std::vector<TopoDS_Shape> BaffList;
	//TopTools_ListOfShape Baffs;
	//Baffs.Append(BaffBody);
	//BaffList.push_back(BaffBody);

	for (int i = 1; i < tTSHEData.mNzlb; ++i)
	{
		TopoDS_Shape TmpBody = BRepBuilderAPI_Copy(BaffBody);

		gp_Trsf Trans_Rotate;
		Trans_Rotate.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), M_PI * i);
		gp_Trsf Trans_Translate;
		Trans_Translate.SetTranslation(gp_Vec((tTSHEData.mLzlb + tTSHEData.mTzlb) * i, 0, 0));

		TmpBody = BRepBuilderAPI_Transform(TmpBody, Trans_Rotate * Trans_Translate);
		
		aBuilder.Add(aRes, TmpBody);
		//aBuilder.Add(aRes, TmpBody);
		//Baffs.Append(TmpBody);
		//BaffList.push_back(TmpBody);
	}



	if (tTurbineSide == 1)
	{
		for (int i = 0; i < TubePosition.size(); ++i)
		{
			gp_Pnt TmpPnt = TubePosition[i];
			TmpPnt.SetX(-tTSHEData.mLs / 2.0);
			gp_Circ TmpCir(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mDkk / 2.0);
			TopoDS_Edge TmpEdge = BRepBuilderAPI_MakeEdge(TmpCir);
			TopoDS_Wire TmpWire = BRepBuilderAPI_MakeWire(TmpEdge);
			TopoDS_Face TmpFace = BRepBuilderAPI_MakeFace(TmpWire);
			TopoDS_Shape TmpBody = BRepPrimAPI_MakePrism(TmpFace, gp_Vec(tTSHEData.mLs, 0, 0));

			aRes = TopoDS::Compound(BRepAlgoAPI_Cut(aRes, TmpBody));

		}
	}

	if (tTurbineSide == 2)
	{
		for (int i = 0; i < TubePosition.size(); ++i)
		{
			gp_Pnt TmpPnt = TubePosition[i];
			TmpPnt.SetX(-tTSHEData.mLs / 2.0);
			gp_Circ TmpCir(gp_Ax2(TmpPnt, gp_Dir(1, 0, 0)), tTSHEData.mDkk / 2.0);
			TopoDS_Edge TmpEdge = BRepBuilderAPI_MakeEdge(TmpCir);
			TopoDS_Wire TmpWire = BRepBuilderAPI_MakeWire(TmpEdge);
			TopoDS_Face TmpFace = BRepBuilderAPI_MakeFace(TmpWire);
			TopoDS_Shape TmpBody = BRepPrimAPI_MakePrism(TmpFace, gp_Vec(tTSHEData.mLs, 0, 0));

			aRes = TopoDS::Compound(BRepAlgoAPI_Cut(aRes, TmpBody));

			gp_Trsf Transf_Mirror;
			Transf_Mirror.SetMirror(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)));
			TopoDS_Shape TmpMirrorBody = BRepBuilderAPI_Transform(TmpBody, Transf_Mirror);
			aRes = TopoDS::Compound(BRepAlgoAPI_Cut(aRes, TmpMirrorBody));

		}
	}


	return aRes;
}

TopoDS_Shape DivideHETS(int tTurbineSide, TSHEData tTSHEData, TopoDS_Shape LeftHead, TopoDS_Shape RightHead, TopoDS_Shape Shell,
	TopoDS_Shape Tubes, TopoDS_Shape Bulkheads, TopoDS_Shape Baffles, TopoDS_Shape& TubeFaces, TopoDS_Shape& ShellFaces, TopoDS_Shape& AllFaces)
{
	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);

	TopoDS_Compound aTubeFacesComp;
	aBuilder.MakeCompound(aTubeFacesComp);
	TopoDS_Compound aShellFacesComp;
	aBuilder.MakeCompound(aShellFacesComp);
	TopoDS_Compound aAllFacesComp;
	aBuilder.MakeCompound(aAllFacesComp);


	TopoDS_Shape copy_LeftHead = BRepBuilderAPI_Copy(LeftHead);
	TopoDS_Shape copy_RightHead = BRepBuilderAPI_Copy(RightHead);
	TopoDS_Shape copy_Bulkheads = BRepBuilderAPI_Copy(Bulkheads);
	TopoDS_Shape copy_Tubes = BRepBuilderAPI_Copy(Tubes);
	TopoDS_Shape copy_Shell = BRepBuilderAPI_Copy(Shell);
	TopoDS_Shape copy_Baffles = BRepBuilderAPI_Copy(Baffles);





	TopoDS_Shape HeatExchangerBody = BRepAlgoAPI_Fuse(copy_LeftHead, copy_RightHead);
	HeatExchangerBody = BRepAlgoAPI_Fuse(HeatExchangerBody, copy_Bulkheads);
	HeatExchangerBody = BRepAlgoAPI_Fuse(HeatExchangerBody, copy_Tubes);
	HeatExchangerBody = BRepAlgoAPI_Fuse(HeatExchangerBody, copy_Shell);
	HeatExchangerBody = BRepAlgoAPI_Fuse(HeatExchangerBody, copy_Baffles);

	for (TopExp_Explorer faceExp(HeatExchangerBody, TopAbs_FACE); faceExp.More(); faceExp.Next())
	{
		TopoDS_Face anFace = TopoDS::Face(faceExp.Current());
		aBuilder.Add(aAllFacesComp, anFace);

		Handle(Geom_Surface) aSurf = BRep_Tool::Surface(anFace);

		BRepAdaptor_Surface AdaFace;
		AdaFace.Initialize(anFace, Standard_False);

		Standard_Real umin, umax, vmin, vmax;
		BRepTools::UVBounds(anFace, umin, umax, vmin, vmax);
		GeomLProp_SLProps props(aSurf, (umin + umax) / 2.0, (vmin + vmax) / 2.0, 1, 0.01);
		gp_Dir normal = props.Normal();

		if (AdaFace.GetType() == GeomAbs_Sphere)
		{
			gp_Sphere tmpSph = AdaFace.Sphere();


			if (tTurbineSide == 2 && abs(tmpSph.Radius() - tTSHEData.mRs) < 0.01 && props.Value().X() > 0)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}


			if (tTurbineSide == 1 && abs(tmpSph.Radius() - tTSHEData.mRs) < 0.01 && props.Value().X() > 0)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

			if (abs(tmpSph.Radius() - tTSHEData.mRs) < 0.01&& props.Value().X() < 0)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}
		}

		if (AdaFace.GetType() == GeomAbs_Cylinder)
		{
			gp_Cylinder tmpCylin = AdaFace.Cylinder();


			//ShellFaces
			if (tmpCylin.Radius() == tTSHEData.mRs)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}

			if ((abs(tmpCylin.Radius() - tTSHEData.mDsin / 2.0) < 0.01 || abs(tmpCylin.Radius() - tTSHEData.mDsout / 2.0) < 0.01 ) 
				&& abs(props.Value().X()) < tTSHEData.mLs / 2.0)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}

			if (tTurbineSide == 2 && tmpCylin.Radius() == tTSHEData.mdt / 2.0 + tTSHEData.mTt
				&& props.Value().X() > -tTSHEData.mLs / 2.0 + tTSHEData.mTgb / 2.0)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}
			if (tTurbineSide == 1 && tmpCylin.Radius() == tTSHEData.mdt / 2.0 + tTSHEData.mTt
				&& abs(props.Value().X()) < tTSHEData.mLs / 2.0 - tTSHEData.mTgb / 2.0)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}

			if (tmpCylin.Radius() == tTSHEData.mDkk / 2.0)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}

			//TubeFaces
			if ((abs(tmpCylin.Radius() - tTSHEData.mDtin / 2.0) < 0.01 || abs(tmpCylin.Radius() - tTSHEData.mDtout / 2.0) < 0.01)
				&& abs(props.Value().X()) > tTSHEData.mLs / 2.0)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

			if (tmpCylin.Radius() == tTSHEData.mdt / 2.0 )
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

			if (tmpCylin.Radius() == tTSHEData.mdt / 2.0 + tTSHEData.mTt
				&& props.Value().X() < -tTSHEData.mLs / 2.0 + tTSHEData.mTgb / 2.0)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

			if (tTurbineSide == 1 && tmpCylin.Radius() == tTSHEData.mdt / 2.0 + tTSHEData.mTt
				&& props.Value().X() > tTSHEData.mLs / 2.0 - tTSHEData.mTgb / 2.0)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

		}


		if (AdaFace.GetType() == GeomAbs_Torus)
		{
			gp_Torus tmpTorus = AdaFace.Torus();

			if (tmpTorus.MajorRadius() > tTSHEData.mdt / 2.0 + tTSHEData.mTt / 2.0)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}

			if (tmpTorus.MinorRadius() < tTSHEData.mdt / 2.0 + tTSHEData.mTt / 2.0)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}
		}

		if (AdaFace.GetType() == GeomAbs_Plane)
		{
			gp_Pln tmpPln = AdaFace.Plane();

			if (tmpPln.Distance(gp_Pnt(-tTSHEData.mLs / 2.0 + tTSHEData.mTgb, 0, 0)) < 0.01
				|| tmpPln.Distance(gp_Pnt(tTSHEData.mLs / 2.0 - tTSHEData.mTgb, 0, 0)) < 0.01)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}

			if (tmpPln.Distance(gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0)) < 0.01
				|| tmpPln.Distance(gp_Pnt(tTSHEData.mLs / 2.0, 0, 0)) < 0.01)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

			if (tmpPln.Distance(gp_Pnt(0, 0, tTSHEData.mTE / 2.0)) < 0.01
				|| tmpPln.Distance(gp_Pnt(0, 0, -tTSHEData.mTE / 2.0)) < 0.01)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

			if (tmpPln.Distance(gp_Pnt(-tTSHEData.mLs / 2.0 - tTSHEData.mLtin, 0, 0)) < 0.01
				|| tmpPln.Distance(gp_Pnt(tTSHEData.mLs / 2.0 + tTSHEData.mLtout, 0, 0)) < 0.01)
			{
				aBuilder.Add(aTubeFacesComp, anFace);
			}

			if (abs(props.Value().X()) < tTSHEData.mLs / 2.0 - tTSHEData.mTgb / 2.0 && abs(props.Value().Z()) < tTSHEData.mDs / 2.0)
			{
				aBuilder.Add(aShellFacesComp, anFace);
			}

		}

	}

	//ShellClosedFace
	gp_Circ ShellInletCir(gp_Ax2(gp_Pnt(-tTSHEData.mLs / 2.0 + tTSHEData.mBsin, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs + tTSHEData.mHsin), 
		gp_Dir(0, 0, 1)), tTSHEData.mDsin / 2.0);
	TopoDS_Edge ShellInletEdge = BRepBuilderAPI_MakeEdge(ShellInletCir);
	TopoDS_Wire ShellInletWire = BRepBuilderAPI_MakeWire(ShellInletEdge);
	TopoDS_Face ShellInletFace = BRepBuilderAPI_MakeFace(ShellInletWire);

	gp_Pnt OutletPnt;
	if (tTSHEData.mNzlb % 2 == 0){
		OutletPnt = gp_Pnt(tTSHEData.mLs / 2.0 - tTSHEData.mBsout, 0, -tTSHEData.mDs / 2.0 - tTSHEData.mTs - tTSHEData.mHsout);
	}
	else{
		OutletPnt = gp_Pnt(tTSHEData.mLs / 2.0 - tTSHEData.mBsout, 0, tTSHEData.mDs / 2.0 + tTSHEData.mTs + tTSHEData.mHsout);
	}
	gp_Circ ShellOutletCir(gp_Ax2(OutletPnt, gp_Dir(0, 0, 1)), tTSHEData.mDsout / 2.0);
	TopoDS_Edge ShellOutletEdge = BRepBuilderAPI_MakeEdge(ShellOutletCir);
	TopoDS_Wire ShellOutletWire = BRepBuilderAPI_MakeWire(ShellOutletEdge);
	TopoDS_Face ShellOutletFace = BRepBuilderAPI_MakeFace(ShellOutletWire);

	aBuilder.Add(aShellFacesComp, ShellInletFace);
	aBuilder.Add(aShellFacesComp, ShellOutletFace);

	//TubeClosedFace
	gp_Dir TubeInLetDir(-cos(tTSHEData.mAtin * M_PI / 180), 0, sin(tTSHEData.mAtin * M_PI / 180));
	Standard_Real Len = sqrt(pow(tTSHEData.mRs + tTSHEData.mTs, 2) - pow(tTSHEData.mDtin / 2.0 + tTSHEData.mTtin, 2)) + tTSHEData.mHtin;
	gp_Pnt TubeInLetPnt = gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0).Translated(gp_Vec(TubeInLetDir) * Len);
	gp_Circ TubeInLetCir(gp_Ax2(TubeInLetPnt, TubeInLetDir), tTSHEData.mDtin / 2.0);
	TopoDS_Edge TubeInLetEdge = BRepBuilderAPI_MakeEdge(TubeInLetCir);
	TopoDS_Wire TubeInLetWire = BRepBuilderAPI_MakeWire(TubeInLetEdge);
	TopoDS_Face TubeInLetFace = BRepBuilderAPI_MakeFace(TubeInLetWire);
	
	TopoDS_Face TubeOutLetFace;
	if (tTurbineSide == 1)
	{
		gp_Dir TubeOutLetDir(cos(tTSHEData.mAtout * M_PI / 180), 0, sin(tTSHEData.mAtout * M_PI / 180));
		Standard_Real Len = sqrt(pow(tTSHEData.mRs + tTSHEData.mTs, 2) - pow(tTSHEData.mDtout / 2.0 + tTSHEData.mTtout, 2)) + tTSHEData.mHtout;
		gp_Pnt TubeOutLetPnt = gp_Pnt(tTSHEData.mLs / 2.0, 0, 0).Translated(gp_Vec(TubeOutLetDir) * Len);
		gp_Circ TubeOutLetCir(gp_Ax2(TubeOutLetPnt, TubeOutLetDir), tTSHEData.mDtout / 2.0);
		TopoDS_Edge TubeOutLetEdge = BRepBuilderAPI_MakeEdge(TubeOutLetCir);
		TopoDS_Wire TubeOutLetWire = BRepBuilderAPI_MakeWire(TubeOutLetEdge);
		TubeOutLetFace = BRepBuilderAPI_MakeFace(TubeOutLetWire);
	}
	else
	{
		gp_Dir TubeOutLetDir(-cos(tTSHEData.mAtout * M_PI / 180), 0, -sin(tTSHEData.mAtout * M_PI / 180));
		Standard_Real Len = sqrt(pow(tTSHEData.mRs + tTSHEData.mTs, 2) - pow(tTSHEData.mDtout / 2.0 + tTSHEData.mTtout, 2)) + tTSHEData.mHtout;
		gp_Pnt TubeOutLetPnt = gp_Pnt(-tTSHEData.mLs / 2.0, 0, 0).Translated(gp_Vec(TubeOutLetDir) * Len);
		gp_Circ TubeOutLetCir(gp_Ax2(TubeOutLetPnt, TubeOutLetDir), tTSHEData.mDtout / 2.0);
		TopoDS_Edge TubeOutLetEdge = BRepBuilderAPI_MakeEdge(TubeOutLetCir);
		TopoDS_Wire TubeOutLetWire = BRepBuilderAPI_MakeWire(TubeOutLetEdge);
		TubeOutLetFace = BRepBuilderAPI_MakeFace(TubeOutLetWire);
	}
	aBuilder.Add(aTubeFacesComp, TubeInLetFace);
	aBuilder.Add(aTubeFacesComp, TubeOutLetFace);

	///
	aBuilder.Add(aRes, HeatExchangerBody);
	aBuilder.Add(aRes, Baffles);

	ShellFaces = aShellFacesComp;
	TubeFaces = aTubeFacesComp;
	AllFaces = aAllFacesComp;
	return aRes;
}

TopoDS_Shape ClassTubularHE::GenerateHeatExchanger()
{

	TopoDS_Shape LeftHead = GenerateLeftHead(mTurbineSide, mTSHEData);
	
	TopoDS_Shape RightHead = GenerateRightHead(mTurbineSide, mTSHEData);

	TopoDS_Shape Shell = GenerateShell(mTSHEData);

	std::vector<gp_Pnt> TubePosition;
	std::vector<gp_Pnt> LGPosition;
	DesignTubineConfiguration(mTurbineSide, mTSHEData, TubePosition, LGPosition);

	TopoDS_Shape Tubes = GenerateTubes(mTurbineSide, TubePosition, LGPosition, mTSHEData);

	TopoDS_Shape Bulkheads = GenerateBulkhead(mTurbineSide, TubePosition, mTSHEData);

	TopoDS_Shape Baffles = GenerateBaffles(mTurbineSide, TubePosition, mTSHEData);

	TopoDS_Shape TubeFaces;
	TopoDS_Shape ShellFaces;
	TopoDS_Shape AllFaces;
	TopoDS_Shape HeatExchanger = DivideHETS(mTurbineSide, mTSHEData, LeftHead, RightHead, Shell, Tubes, Bulkheads, Baffles,
		TubeFaces, ShellFaces, AllFaces);

	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);
	//aBuilder.Add(aRes, HeatExchanger);
	//aBuilder.Add(aRes, TubeFaces);
	aBuilder.Add(aRes, ShellFaces);

	//aBuilder.Add(aRes, LeftHead);
	//aBuilder.Add(aRes, RightHead);
	//aBuilder.Add(aRes, Shell);
	//aBuilder.Add(aRes, Tubes);
	//aBuilder.Add(aRes, Bulkheads);
	//aBuilder.Add(aRes, Baffles);


	TopTools_ListOfShape HELeftHead;
	HELeftHead.Append(LeftHead);
	//SaveIges(HELeftHead, mExportPath + "HELeftHead.iges");
	OCCBasicTools::SaveStep(LeftHead, mExportPath + "HELeftHead.stp");

	TopTools_ListOfShape HERightHead;
	HERightHead.Append(RightHead);
	//SaveIges(HERightHead, mExportPath + "HERightHead.iges");
	OCCBasicTools::SaveStep(RightHead, mExportPath + "HERightHead.stp");

	TopTools_ListOfShape HEShell;
	HEShell.Append(Shell);
	//SaveIges(HEShell, mExportPath + "HEShell.iges");
	OCCBasicTools::SaveStep(Shell, mExportPath + "HEShell.stp");

	TopTools_ListOfShape HETubes;
	HETubes.Append(Tubes);
	//SaveIges(HETubes, mExportPath + "HETubes.iges");
	OCCBasicTools::SaveStep(Tubes, mExportPath + "HETubes.stp");

	TopTools_ListOfShape HEBulkheads;
	HEBulkheads.Append(Bulkheads);
	//SaveIges(HEBulkheads, mExportPath + "HEBulkheads.iges");
	OCCBasicTools::SaveStep(Bulkheads, mExportPath + "HEBulkheads.stp");

	TopTools_ListOfShape HEBaffles;
	HEBaffles.Append(Baffles);
	//SaveIges(HEBaffles, mExportPath + "HEBaffles.iges");
	OCCBasicTools::SaveStep(Baffles, mExportPath + "HEBaffles.stp");
	
	OCCBasicTools::SaveStl(TubeFaces, mExportPath + "HETubeFaces.stl");

	OCCBasicTools::SaveStl(ShellFaces, mExportPath + "HEShellFaces.stl");

	OCCBasicTools::SaveStl(AllFaces, mExportPath + "HESolidFaces.stl");

	return aRes;
}
