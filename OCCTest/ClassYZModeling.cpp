#include "ClassYZModeling.h"

#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Circ.hxx>

#include <GC_MakeCircle.hxx>
#include <GC_MakeArcOfCircle.hxx>

#include <Geom_Circle.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_TrimmedCurve.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Section.hxx>

#include <BRepExtrema_DistShapeShape.hxx>
#include <vector>

#include <OCCBasicTools.h>

ClassYZModeling::ClassYZModeling()
{
	mYZData.m_n = 0;

	mYZData.m_L = 0;
	mYZData.m_L0 = 0;
	mYZData.m_D = 0;
	mYZData.m_e = 0;
	mYZData.m_l = 0;
	mYZData.m_r = 0;
	mYZData.m_a = 0;
	mYZData.m_r1 = 0;
	mYZData.m_b = 0;
	mYZData.m_lmd = 0;

	mYZData.m_InletColor = mYZData.m_OutletColor = { 1, 0, 0 };
}

ClassYZModeling::ClassYZModeling(YZData tYZData)
{
	mYZData.m_n = tYZData.m_n > 1 ? tYZData.m_n : 2;
	mYZData.m_b = 360.0 / 2.0 / mYZData.m_n;
	//mYZData.m_b = tYZData.m_b;
	//mYZData.m_n = (int)(360.0 / 2.0 / mYZData.m_b + 0.5);
	
	mYZData.m_L = tYZData.m_L;
	mYZData.m_L0 = tYZData.m_L0;
	mYZData.m_D = tYZData.m_D;
	mYZData.m_e = mYZData.m_D / 2. - mYZData.m_l - mYZData.m_r;
	mYZData.m_l = tYZData.m_l;
	mYZData.m_r = tYZData.m_r;
	mYZData.m_a = tYZData.m_a;
	mYZData.m_r1 = tYZData.m_r1;
	mYZData.m_lmd = tYZData.m_lmd;
}


bool ClassYZModeling::GenerateModeling()
{
	TopoDS_Compound aRes;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aRes);

	int t_n = mYZData.m_n;
	double t_b = mYZData.m_b;
	//double t_b = 360.0 / 2.0 / mYZData.m_n;
	
	double t_L = mYZData.m_L;
	double t_L0 = mYZData.m_L0;
	double t_D = mYZData.m_D;
	double t_e = mYZData.m_e;
	double t_l = mYZData.m_l;
	double t_r = mYZData.m_r;
	double t_a = mYZData.m_a;
	double t_r1 = mYZData.m_r1;

	double t_lmd = mYZData.m_lmd;


	gp_Pnt Center(t_L0, 0, 0);

	TopoDS_Shape Cylinder = BRepPrimAPI_MakeCylinder(gp_Ax2(Center, gp::DX()), t_D / 2., t_L);


	gp_Pnt Pnt1(t_L0, 0, t_l + t_r);
	gp_Pnt Pnt2 = gp_Pnt(t_L0, 0, t_l).Rotated(gp_Ax1(Center, gp::DX()), (t_b - t_lmd) * M_PI / 180.);
	gp_Pnt Pnt3 = Pnt2.Translated(gp_Vec(Center, Pnt2).Normalized() * t_r);
	gp_Pnt Pnt4 = Pnt3.Rotated(gp_Ax1(Pnt2, gp::DX()), (90 - t_a + t_lmd) * M_PI / 180.);
	gp_Pnt Pnt5 = Pnt4.Translated(gp_Vec(gp::DX()).Crossed(gp_Vec(Pnt2, Pnt4)).Normalized() * t_l);
	gp_Pnt Pnt6 = Pnt1.Rotated(gp_Ax1(Center, gp::DX()), t_b * M_PI / 180.);

	Handle(Geom_TrimmedCurve) anArc1 = GC_MakeArcOfCircle(gp_Circ(gp_Ax2(Center, gp::DX()), t_l + t_r), Pnt1, Pnt3, true);
	TopoDS_Edge Arc1 = BRepBuilderAPI_MakeEdge(anArc1);
	Handle(Geom_TrimmedCurve) anArc2 = GC_MakeArcOfCircle(gp_Circ(gp_Ax2(Pnt2, gp::DX()), t_r), Pnt3, Pnt4, true);
	TopoDS_Edge Arc2 = BRepBuilderAPI_MakeEdge(anArc2);

	TopoDS_Edge Edge1 = BRepBuilderAPI_MakeEdge(Pnt4, Pnt5);
	TopoDS_Edge Edge2 = BRepBuilderAPI_MakeEdge(Center, Pnt6);

	
	BRepExtrema_DistShapeShape tDistShapeShapeBuilder(Edge1, Edge2);
	if (tDistShapeShapeBuilder.IsDone())
	{
		Pnt5 = tDistShapeShapeBuilder.PointOnShape1(1);
	}

	Pnt6 = Pnt5.Translated(gp_Vec(Center, Pnt5).Normalized() * t_r1 / sin(t_a * M_PI / 180.));
	gp_Pnt Pnt7 = Pnt5.Translated(gp_Vec(Pnt5, Pnt4).Normalized() * t_r1 / tan(t_a * M_PI / 180.));
	Edge1 = BRepBuilderAPI_MakeEdge(Pnt4, Pnt7);
	//Edge2 = BRepBuilderAPI_MakeEdge(Center, Pnt6);
	Handle(Geom_TrimmedCurve) anArc3 = GC_MakeArcOfCircle(gp_Circ(gp_Ax2(Pnt6, gp::DX()), t_r1), Pnt5, Pnt7, true);
	TopoDS_Edge Arc3 = BRepBuilderAPI_MakeEdge(anArc3);

	gp_Trsf tMirror;
	tMirror.SetMirror(gp_Ax2(Center, gp::DY()));
	std::vector<TopoDS_Edge> anEdges, allEdges;
	anEdges.push_back(Arc1);
	anEdges.push_back(Arc2);
	anEdges.push_back(Arc3);
	anEdges.push_back(Edge1);
	anEdges.push_back(TopoDS::Edge(BRepBuilderAPI_Transform(Arc1, tMirror, true)));
	anEdges.push_back(TopoDS::Edge(BRepBuilderAPI_Transform(Arc2, tMirror, true)));
	anEdges.push_back(TopoDS::Edge(BRepBuilderAPI_Transform(Arc3, tMirror, true)));
	anEdges.push_back(TopoDS::Edge(BRepBuilderAPI_Transform(Edge1, tMirror, true)));

	gp_Trsf tRotate;
	for (int i = 0; i < anEdges.size(); ++i)
	{
		for (int j = 0; j < t_n; ++j)
		{
			tRotate.SetRotation(gp_Ax1(Center, gp::DX()), j * 2 * M_PI / t_n);

			TopoDS_Edge RotateEdge = TopoDS::Edge(BRepBuilderAPI_Transform(anEdges[i], tRotate, true));
			allEdges.push_back(RotateEdge);
		}

	}

	TopoDS_Wire StarWire;
	OCCBasicTools::GetOrderWireFromEdges(allEdges, StarWire);

	TopoDS_Face StarFace = BRepBuilderAPI_MakeFace(StarWire);

	TopoDS_Shape StarBody = BRepPrimAPI_MakePrism(StarFace, gp_Vec(t_L, 0, 0));

	StarBody = BRepAlgoAPI_Cut(Cylinder, StarBody);

	aBuilder.Add(aRes, StarBody);
	m_Shape = aRes;

	return true;
}
