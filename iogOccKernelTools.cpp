//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/14
//
//=============================================================================
#include "core.h"
#include "comobj.h"
#include "appinterface.h"
#include "config.h"

#include "iogOccKernelTools.h"

//////////////////////////////////////////////////////////////////
//

struct struShellFaces
{
	TopTools_ListOfShape faceList;
	std::string name;
};

iogOccKernelTools::iogOccKernelTools()
{
}


iogOccKernelTools::~iogOccKernelTools()
{
}

//////////////////////////////////////////////////////////////////
//

bool iogOccKernelTools::shapeFilter(const TopoDS_Shape& origShape, TopAbs_ShapeEnum targetType,
	TopoDS_Shape& filterShape)
{
	if (origShape.NbChildren() == 1 && origShape.ShapeType() < targetType)
	{
		TopoDS_Iterator iter(origShape);
		return shapeFilter(iter.Value(), targetType, filterShape);
	}
	else if (origShape.ShapeType() == targetType)
	{
		filterShape = origShape;
		return true;
	}
	else
	{
		return false;
	}
}

bool iogOccKernelTools::AIsContainedInB(TopoDS_Shape aShape, TopoDS_Shape bShape)
{
	TopAbs_ShapeEnum judgetype = aShape.ShapeType();
	TopExp_Explorer ex(bShape, judgetype);
	for (; ex.More(); ex.Next())
	{
		if (aShape.IsEqual(ex.Current()))
		{
			return true;
		}
	}
	return false;
}

bool iogOccKernelTools::IsAIndipendentInTheList(TopoDS_Shape aShape, TopTools_ListOfShape shapeList)
{
	for (auto iter : shapeList)
	{
		bool overlap = false;
		if (iter.IsSame(aShape))
		{
			continue;
		}
		if (IsShapeGeomSame(iter, aShape, aShape.ShapeType()))
		{
			continue;
		}
		TopoDS_Shape commonShape = BRepAlgoAPI_Common(aShape, iter);
		TopoDS_Iterator topoiter(commonShape);
		for (; topoiter.More(); topoiter.Next())
		{
			overlap = true;
			break;
		}

		bool connect = false;
		TopExp_Explorer edgeex;
		for (edgeex.Init(aShape, TopAbs_EDGE); edgeex.More(); edgeex.Next())
		{
			TopoDS_Edge edge1 = TopoDS::Edge(edgeex.Current());
			TopExp_Explorer edgeex2;
			for (edgeex2.Init(iter, TopAbs_EDGE); edgeex2.More(); edgeex2.Next())
			{
				TopoDS_Edge edge2 = TopoDS::Edge(edgeex2.Current());
				if (IsShapeGeomSame(edge1, edge2, TopAbs_EDGE))
				{
					connect = true;
					break;
				}
			}

			if (connect)
			{
				break;
			}
		}
		if (connect || overlap)
		{
			return false;
		}
	}
	return true;
}


bool iogOccKernelTools::IsShapeGeomSame(const TopoDS_Shape shape1, const TopoDS_Shape shape2,
	const TopAbs_ShapeEnum type)
{
	try
	{
		if (shape1.ShapeType() > type || shape2.ShapeType() > type)
			return false;
		TopoDS_Shape filterShape1, filterShape2;
		shapeFilter(shape1, type, filterShape1);
		shapeFilter(shape2, type, filterShape2);

		if (type == TopAbs_VERTEX)
		{
			if (BRep_Tool::Pnt(TopoDS::Vertex(filterShape1)).IsEqual(BRep_Tool::Pnt(TopoDS::Vertex(filterShape2)), Precision::Confusion()))
				return true;
		}
		else if (type == TopAbs_EDGE)
		{
			double start1, end1, start2, end2;
			Handle(Geom_Curve) curve1 = BRep_Tool::Curve(TopoDS::Edge(filterShape1), start1, end1);
			Handle(Geom_Curve) curve2 = BRep_Tool::Curve(TopoDS::Edge(filterShape2), start2, end2);
			if (!curve1 || !curve2)
				return false;

			if (curve1->Value(start1).IsEqual(curve2->Value(start2), Precision::Confusion()) &&
				curve1->Value(end1).IsEqual(curve2->Value(end2), Precision::Confusion()) &&
				curve1->Value(0.5 * (start1 + end1)).IsEqual(curve2->Value(0.5 * (start2 + end2)), Precision::Confusion()))
				return true;
			else if (curve1->Value(start1).IsEqual(curve2->Value(end2), Precision::Confusion()) &&
				curve1->Value(end1).IsEqual(curve2->Value(start2), Precision::Confusion()) &&
				curve1->Value(0.5 * (start1 + end1)).IsEqual(curve2->Value(0.5 * (start2 + end2)), Precision::Confusion()))
				return true;
		}
		else if (type == TopAbs_FACE)
		{
			double ustart1, uend1, vstart1, vend1;
			double ustart2, uend2, vstart2, vend2;
			Handle(Geom_Surface) surface1 = BRep_Tool::Surface(TopoDS::Face(filterShape1));
			Handle(Geom_Surface) surface2 = BRep_Tool::Surface(TopoDS::Face(filterShape2));

			BRepAdaptor_Surface adp_sur1(TopoDS::Face(filterShape1));
			BRepAdaptor_Surface adp_sur2(TopoDS::Face(filterShape2));
			ustart1 = adp_sur1.FirstUParameter();
			uend1 = adp_sur1.LastUParameter();
			vstart1 = adp_sur1.FirstVParameter();
			vend1 = adp_sur1.LastVParameter();

			ustart2 = adp_sur2.FirstUParameter();
			uend2 = adp_sur2.LastUParameter();
			vstart2 = adp_sur2.FirstVParameter();
			vend2 = adp_sur2.LastVParameter();

			if (surface1->Value(ustart1, vstart1).IsEqual(surface2->Value(ustart2, vstart2), Precision::Confusion()) &&
				surface1->Value(uend1, vstart1).IsEqual(surface2->Value(uend2, vstart2), Precision::Confusion()) &&
				surface1->Value(ustart1, vend1).IsEqual(surface2->Value(ustart2, vend2), Precision::Confusion()) &&
				surface1->Value(uend1, vend1).IsEqual(surface2->Value(uend2, vend2), Precision::Confusion()) &&
				surface1->Value(0.5 * (ustart1 + uend1), 0.5 * (vstart1 + vend1)).IsEqual(surface2->Value(0.5 * (ustart2 + uend2), 0.5 * (vstart2 + vend2)), Precision::Confusion()))
				return true;
		}
		return false;
	}
	catch (...)
	{
		return false;
	}
}

//bool iogOccKernelTools::IsFaceSameRole(const TopoDS_Shape shape1, const TopoDS_Shape shape2)
//{
//	//Handle(BRepAdaptor_Surface) brepSurf1 = new BRepAdaptor_Surface(TopoDS::Face(shape1));
//	//Handle(BRepAdaptor_Surface) brepSurf2 = new BRepAdaptor_Surface(TopoDS::Face(shape1));
//
//	//GeomAbs_SurfaceType surfType1 = brepSurf1->GetType();
//	//GeomAbs_SurfaceType surfType2 = brepSurf1->GetType();
//
//	///*Handle(Geom_BSplineSurface) bsSurf1 = brepSurf1->BSpline();
//	//Handle(Geom_BSplineSurface) bsSurf2 = brepSurf2->BSpline();*/
//
//	//int nUDegree1 = brepSurf1->UDegree();
//	//int nUDegree2 = brepSurf2->UDegree();
//
//	//int nVDegree1 = brepSurf1->VDegree();
//	//int nVDegree2 = brepSurf2->VDegree();
//
//	//int nUKnots1 = brepSurf1->NbUKnots();
//	//int nUKnots2 = brepSurf2->NbUKnots();
//
//	//int nVKnots1 = brepSurf1->NbVKnots();
//	//int nVKnots2 = brepSurf2->NbVKnots();
//
//	//if (nUDegree1 == nUDegree2 && nVDegree1 == nVDegree2 &&
//	//	nUKnots1 == nUKnots2 && nVKnots1 == nVKnots2)
//	//{
//	//	return true;
//	//}
//}


bool iogOccKernelTools::IsFaceSameRole(const TopoDS_Shape shape1, const TopoDS_Shape shape2)
{
	if (shape1.ShapeType() != TopAbs_FACE || shape2.ShapeType() != TopAbs_FACE)
	{
		return false;
	}
	BRepAdaptor_Surface adp_sur1(TopoDS::Face(shape1));
	BRepAdaptor_Surface adp_sur2(TopoDS::Face(shape2));
	double ustart1 = adp_sur1.FirstUParameter();
	double uend1 = adp_sur1.LastUParameter();
	double vstart1 = adp_sur1.FirstVParameter();
	double vend1 = adp_sur1.LastVParameter();

	double ustart2 = adp_sur2.FirstUParameter();
	double uend2 = adp_sur2.LastUParameter();
	double vstart2 = adp_sur2.FirstVParameter();
	double vend2 = adp_sur2.LastVParameter();

	gp_Pnt pntUsVsSurf1 = adp_sur1.Value(ustart1, vstart1);
	gp_Pnt pntUsVsSurf2 = adp_sur2.Value(ustart2, vstart2);

	double dXPntUsVsSurf1 = pntUsVsSurf1.X(); 
	double dYPntUsVsSurf1 = pntUsVsSurf1.Y(); 
	double dZPntUsVsSurf1 = pntUsVsSurf1.Z();

	double dXPntUsVsSurf2 = pntUsVsSurf2.X();
	double dYPntUsVsSurf2 = pntUsVsSurf2.Y();
	double dZPntUsVsSurf2 = pntUsVsSurf2.Z();

	QString qstr1 = QString::number(dXPntUsVsSurf1, 'f', 5) + ","
		+ QString::number(dYPntUsVsSurf1, 'f', 5) + ","
		+ QString::number(dZPntUsVsSurf1, 'f', 5);

	QString qstr2 = QString::number(dXPntUsVsSurf2, 'f', 5) + ","
		+ QString::number(dYPntUsVsSurf2, 'f', 5) + ","
		+ QString::number(dZPntUsVsSurf2, 'f', 5);

	/*if (qstr1 != qstr2)
		return false;*/

	gp_Pnt pntUeVsSurf1 = adp_sur1.Value(uend1, vstart1);
	gp_Pnt pntUeVsSurf2 = adp_sur2.Value(uend2, vstart2);

	double dXPntUeVsSurf1 = pntUeVsSurf1.X();
	double dYPntUeVsSurf1 = pntUeVsSurf1.Y();
	double dZPntUeVsSurf1 = pntUeVsSurf1.Z();

	double dXPntUeVsSurf2 = pntUeVsSurf2.X();
	double dYPntUeVsSurf2 = pntUeVsSurf2.Y();
	double dZPntUeVsSurf2 = pntUeVsSurf2.Z();

	QString qstr3 = QString::number(dXPntUeVsSurf1, 'f', 5) + ","
		+ QString::number(dYPntUeVsSurf1, 'f', 5) + ","
		+ QString::number(dZPntUeVsSurf1, 'f', 5);

	QString qstr4 = QString::number(dXPntUeVsSurf2, 'f', 5) + ","
		+ QString::number(dYPntUeVsSurf2, 'f', 5) + ","
		+ QString::number(dZPntUeVsSurf2, 'f', 5);

	/*if (qstr3 != qstr4)
		return false;*/

	gp_Pnt pntUsVeSurf1 = adp_sur1.Value(ustart1, vend1);
	gp_Pnt pntUsVeSurf2 = adp_sur2.Value(ustart2, vend2);

	double dXPntUsVeSurf1 = pntUsVeSurf1.X();
	double dYPntUsVeSurf1 = pntUsVeSurf1.Y();
	double dZPntUsVeSurf1 = pntUsVeSurf1.Z();

	double dXPntUsVeSurf2 = pntUsVeSurf2.X();
	double dYPntUsVeSurf2 = pntUsVeSurf2.Y();
	double dZPntUsVeSurf2 = pntUsVeSurf2.Z();

	QString qstr5 = QString::number(dXPntUsVeSurf1, 'f', 5) + ","
		+ QString::number(dYPntUsVeSurf1, 'f', 5) + ","
		+ QString::number(dZPntUsVeSurf1, 'f', 5);

	QString qstr6 = QString::number(dXPntUsVeSurf2, 'f', 5) + ","
		+ QString::number(dYPntUsVeSurf2, 'f', 5) + ","
		+ QString::number(dZPntUsVeSurf2, 'f', 5);

	/*if (qstr5 != qstr6)
		return false;*/

	gp_Pnt pntUeVeSurf1 = adp_sur1.Value(vend1, vend1);
	gp_Pnt pntUeVeSurf2 = adp_sur2.Value(vend2, vend2);

	double dXPntUeVeSurf1 = pntUeVeSurf1.X();
	double dYPntUeVeSurf1 = pntUeVeSurf1.Y();
	double dZPntUeVeSurf1 = pntUeVeSurf1.Z();

	double dXPntUeVeSurf2 = pntUeVeSurf2.X();
	double dYPntUeVeSurf2 = pntUeVeSurf2.Y();
	double dZPntUeVeSurf2 = pntUeVeSurf2.Z();

	QString qstr7 = QString::number(dXPntUeVeSurf1, 'f', 5) + ","
		+ QString::number(dYPntUeVeSurf1, 'f', 5) + ","
		+ QString::number(dZPntUeVeSurf1, 'f', 5);

	QString qstr8 = QString::number(dXPntUeVeSurf2, 'f', 5) + ","
		+ QString::number(dYPntUeVeSurf2, 'f', 5) + ","
		+ QString::number(dZPntUeVeSurf2, 'f', 5);

	/*if (qstr7 != qstr8)
		return false;*/

	gp_Pnt pntUfVfSurf1 = adp_sur1.Value(0.5 * (ustart1 + uend1), 0.5 * (ustart1 + uend1));
	gp_Pnt pntUfVfSurf2 = adp_sur2.Value(0.5 * (ustart2 + uend2), 0.5 * (ustart2 + uend2));

	double dXPntUfVfSurf1 = pntUfVfSurf1.X();
	double dYPntUfVfSurf1 = pntUfVfSurf1.Y();
	double dZPntUfVfSurf1 = pntUfVfSurf1.Z();

	double dXPntUfVfSurf2 = pntUfVfSurf2.X();
	double dYPntUfVfSurf2 = pntUfVfSurf2.Y();
	double dZPntUfVfSurf2 = pntUfVfSurf2.Z();

	QString qstr9 = QString::number(dXPntUfVfSurf1, 'f', 5) + ","
		+ QString::number(dYPntUfVfSurf1, 'f', 5) + ","
		+ QString::number(dZPntUfVfSurf1, 'f', 5);

	QString qstr10 = QString::number(dXPntUfVfSurf2, 'f', 5) + ","
		+ QString::number(dYPntUfVfSurf2, 'f', 5) + ","
		+ QString::number(dZPntUfVfSurf2, 'f', 5);

	/*if (qstr9 != qstr10)
		return false;*/

	return false;
}

TopoDS_Shape iogOccKernelTools::makeSpiralHelix(double radiusbottom, double radiustop,
	double height, double nbturns,
	double breakperiod, bool leftHanded)
{
	// 1000 periods is an OCCT limit. The 3D curve gets truncated
	// if the 2D curve spans beyond this limit.
	if ((breakperiod < 0) || (breakperiod > 1000))
		Standard_Failure::Raise("Break period must be in [0, 1000]");
	if (breakperiod == 0)
		breakperiod = 1000;
	if (nbturns <= 0)
		Standard_Failure::Raise("Number of turns must be greater than 0");

	Standard_Real nbPeriods = nbturns / breakperiod;
	Standard_Real nbFullPeriods = floor(nbPeriods);
	Standard_Real partPeriod = nbPeriods - nbFullPeriods;

	// A Bezier curve is used below, to get a periodic surface also for spirals.
	TColgp_Array1OfPnt poles(1, 2);
	poles(1) = gp_Pnt(radiusbottom, 0, 0);
	poles(2) = gp_Pnt(radiustop, 0, height);
	Handle(Geom_BezierCurve) meridian = new Geom_BezierCurve(poles);

	gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
	Handle(Geom_Surface) surf = new Geom_SurfaceOfRevolution(meridian, axis);

	gp_Pnt2d beg(0, 0);
	gp_Pnt2d end(0, 0);
	gp_Vec2d dir(breakperiod * 2.0 * M_PI, 1 / nbPeriods);
	if (leftHanded == Standard_True)
		dir = gp_Vec2d(-breakperiod * 2.0 * M_PI, 1 / nbPeriods);
	Handle(Geom2d_TrimmedCurve) segm;
	TopoDS_Edge edgeOnSurf;
	BRepBuilderAPI_MakeWire mkWire;
	for (unsigned long i = 0; i < nbFullPeriods; i++) {
		end = beg.Translated(dir);
		segm = GCE2d_MakeSegment(beg, end);
		edgeOnSurf = BRepBuilderAPI_MakeEdge(segm, surf);
		mkWire.Add(edgeOnSurf);
		beg = end;
	}
	if (partPeriod > Precision::Confusion()) {
		dir.Scale(partPeriod);
		end = beg.Translated(dir);
		segm = GCE2d_MakeSegment(beg, end);
		edgeOnSurf = BRepBuilderAPI_MakeEdge(segm, surf);
		mkWire.Add(edgeOnSurf);
	}

	TopoDS_Wire wire = mkWire.Wire();
	BRepLib::BuildCurves3d(wire, Precision::Confusion(), GeomAbs_Shape::GeomAbs_C1, 14, 10000);
	return TopoDS_Shape(std::move(wire));
}

bool iogOccKernelTools::GetPlanarFaceApexs(TopoDS_Face face, TopTools_ListOfShape& vtxs,
	TopTools_ListOfShape& uselessEdges)
{
	std::vector<TopoDS_Edge> vecEdges;
	TopExp_Explorer edgex(face, TopAbs_EDGE);
	for (; edgex.More(); edgex.Next())
	{
		vecEdges.push_back(TopoDS::Edge(edgex.Current()));
	}
	for (int i = 0; i < vecEdges.size(); i++)
	{
		TopoDS_Edge edge1 = vecEdges[i];
		ShapeAnalysis_Edge sae;
		TopoDS_Vertex vtx11 = sae.FirstVertex(edge1);
		gp_Pnt pt11 = BRep_Tool::Pnt(vtx11);
		TopoDS_Vertex vtx12 = sae.LastVertex(edge1);
		gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);

		for (int j = 0; j < vecEdges.size(); j++)
		{
			TopoDS_Edge edge2 = vecEdges[j];
			if (edge1.IsSame(edge2))
			{
				continue;
			}
			TopoDS_Vertex vtx21 = sae.FirstVertex(edge2);
			gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);
			TopoDS_Vertex vtx22 = sae.LastVertex(edge2);
			gp_Pnt pt22 = BRep_Tool::Pnt(vtx22);
			bool ifcontinue = true;
			if (pt12.X() == pt21.X() &&
				pt12.Y() == pt21.Y() &&
				pt12.Z() == pt21.Z())
			{
				ifcontinue = false;
			}
			if (ifcontinue)
			{
				continue;
			}
			gp_Dir dir1(pt12.X() - pt11.X(), pt12.Y() - pt11.Y(), pt12.Z() - pt11.Z());
			gp_Dir dir2(pt22.X() - pt21.X(), pt22.Y() - pt21.Y(), pt22.Z() - pt21.Z());
			if (!dir1.IsParallel(dir2, 0.1))
			{
				if (!vtxs.Contains(vtx21))
				{
					vtxs.Append(vtx21);
				}
			}
			else
			{
				if (!uselessEdges.Contains(edge2))
				{
					uselessEdges.Append(edge2);
				}
			}
		}
	}
	return true;
}

bool iogOccKernelTools::ReOrgnizeEdgeOrderWire(TopTools_ListOfShape& edges, TopoDS_Vertex vtx,
	std::vector<TopoDS_Shape>& newOrderEdges)
{
	std::vector<TopoDS_Shape> vecOrder;
	for (auto iter : edges)
	{
		vecOrder.push_back(iter);
	}
	int count = -1;
	for (int i = 0; i < vecOrder.size(); i++)
	{
		count++;
		TopoDS_Edge edge1 = TopoDS::Edge(vecOrder[i]);
		ShapeAnalysis_Edge sae;
		TopoDS_Vertex vtx11 = sae.FirstVertex(edge1);
		if (IsShapeGeomSame(vtx11, vtx, TopAbs_VERTEX))
		{
			break;
		}
	}
	for (int i = count; i < vecOrder.size(); i++)
	{
		newOrderEdges.push_back(vecOrder[i]);
	}
	for (int i = 0; i < count; i++)
	{
		newOrderEdges.push_back(vecOrder[i]);
	}
	return true;
}

bool iogOccKernelTools::GroupEdgesInWire(std::vector<TopoDS_Shape>& edges, 
	std::vector<std::vector<TopoDS_Shape>>& groups)
{
	if (edges.size() <= 0)
		return false;
	std::vector<TopoDS_Shape> group;
	group.push_back(edges[0]);
	int count = -1;
	for (int i = 0; i < edges.size() - 1; i++)
	{
		TopoDS_Edge edge1 = TopoDS::Edge(edges[i]);
		ShapeAnalysis_Edge sae1;
		TopoDS_Vertex vtx11 = sae1.FirstVertex(edge1);
		gp_Pnt pt11 = BRep_Tool::Pnt(vtx11);
		TopoDS_Vertex vtx12 = sae1.LastVertex(edge1);
		gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);

		TopoDS_Edge edge2 = TopoDS::Edge(edges[i+1]);
		ShapeAnalysis_Edge sae2;
		TopoDS_Vertex vtx21 = sae2.FirstVertex(edge2);
		gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);
		TopoDS_Vertex vtx22 = sae2.LastVertex(edge2);
		gp_Pnt pt22 = BRep_Tool::Pnt(vtx22);

		gp_Dir dir1(pt12.X() - pt11.X(), pt12.Y() - pt11.Y(), pt12.Z() - pt11.Z());
		gp_Dir dir2(pt22.X() - pt21.X(), pt22.Y() - pt21.Y(), pt22.Z() - pt21.Z());
		if (dir1.IsParallel(dir2, 0.1))
		{
			group.push_back(edges[i + 1]);
		}
		else
		{
			count = i + 1;
			break;
		}
	}

	std::vector<TopoDS_Shape> newEdges;
	for (int i = count; i < edges.size(); i++)
	{
		newEdges.push_back(edges[i]);
	}
	groups.push_back(group);
	GroupEdgesInWire(newEdges, groups);
	return true;
}

bool iogOccKernelTools::ReOrderEdgesInWire(std::vector<std::vector<TopoDS_Shape>>& groups, TopoDS_Wire &wire)
{
	BRepBuilderAPI_MakeWire mkwire;
	for (int i = 0; i < groups.size(); i++)
	{
		std::vector<TopoDS_Shape> group = groups[i];
		if (group.size() == 1)
		{
			mkwire.Add(TopoDS::Edge(group[0]));
		}
		else if (group.size() > 1)
		{
			GeomConvert_CompCurveToBSplineCurve gcc;
			for (int j = 0; j < group.size(); j++)
			{
				TopoDS_Edge xedge = TopoDS::Edge(group[j]);
				BRepAdaptor_Curve bac(xedge);
				double start, end;
				start = bac.FirstParameter();
				end = bac.LastParameter();
				Handle(Geom_Curve) gc = bac.Curve().Curve();
				Handle(Geom_TrimmedCurve) gtc = new Geom_TrimmedCurve(gc, start, end);
				gcc.Add(gtc, Precision::Confusion());
			}
			TopoDS_Edge xedge = BRepBuilderAPI_MakeEdge(gcc.BSplineCurve()).Edge();
			mkwire.Add(xedge);
		}
	}
	if (mkwire.IsDone())
	{
		wire = mkwire.Wire();
	}
	else
	{
		return false;
	}
	return true;
}

//分离step文件各特征方法 start
bool iogOccKernelTools::STPSeperateBodies(TopoDS_Shape input, TopoDS_Compound& structuredRegroupShape)
{
	//存一下所有面
	TopTools_ListOfShape allfaces;
	TopExp_Explorer faceex;
	for (faceex.Init(input, TopAbs_FACE); faceex.More(); faceex.Next())
	{
		allfaces.Append(faceex.Current());
	}
	//存放各个独立体的面
	std::vector<std::vector<TopoDS_Face>> independentbodyfaces;
	GetIndependentFeatureFacesAndStore(input, allfaces, independentbodyfaces);
	if (independentbodyfaces.size() < 1)
	{
		return false;
	}
	BRep_Builder bMain;
	bMain.MakeCompound(structuredRegroupShape);
	for (int i = 0; i < independentbodyfaces.size(); i++)
	{
		BRep_Builder b;
		TopoDS_Compound currentComp;
		b.MakeCompound(currentComp);
		std::vector<TopoDS_Face> currentFaceList = independentbodyfaces[i];
		if (currentFaceList.size() < 1)
		{
			continue;
		}
		for (int j = 0; j < currentFaceList.size(); j++)
		{
			TopoDS_Face cuurentFace = currentFaceList[j];
			b.Add(currentComp, cuurentFace);
		}
		bMain.Add(structuredRegroupShape, currentComp);
	}
	return true;
}

bool iogOccKernelTools::GetIndependentFeatureFacesAndStore(TopoDS_Shape shape, TopTools_ListOfShape& facepackage,
	std::vector<std::vector<TopoDS_Face>>& independentbodyfaces)
{
	while (facepackage.Size() > 0)
	{
		std::vector<TopoDS_Face> featurefaces;
		if (!LoopOperate(shape, featurefaces, facepackage))
		{
			continue;
		}
		independentbodyfaces.push_back(featurefaces);
	}
	return true;
}

bool iogOccKernelTools::LoopOperate(TopoDS_Shape shape, std::vector<TopoDS_Face>& featurefaces,
	TopTools_ListOfShape& facepackage)
{
	if (facepackage.Size() == 0)
		return false;
	if (facepackage.Size() == 1)
	{
		featurefaces.push_back(TopoDS::Face(*facepackage.begin()));
		return true;
	}
	TopTools_ListOfShape usedfeatures, reffeaturefaces;
	for (auto iter : facepackage)
	{
		if (usedfeatures.Contains(iter))
			continue;
		BRepAdaptor_Surface brs(TopoDS::Face(iter));
		double uf = brs.FirstUParameter();
		double vf = brs.FirstVParameter();
		double ue = brs.LastUParameter();
		double ve = brs.LastVParameter();
		
		gp_Pnt midFace1 = brs.Value((ue + uf) / 2, (ve + vf) / 2);
		/*gp_Dir dir;
		if (!FaceNormal(TopoDS::Face(iter), (ue + uf) / 2, (ve + vf) / 2, dir))
			continue;*/
		for (auto iter2 : facepackage)
		{
			if (iter2.IsSame(iter))
				continue;
			if (usedfeatures.Contains(iter))
				continue;
			TopoDS_Shape common = BRepAlgoAPI_Section(iter, iter2).Shape();
			TopoDS_Edge commonedge = TopoDS::Edge(common);
			BRepAdaptor_Curve bac(commonedge);
			gp_Dir edgedir = bac.DN(bac.FirstParameter(), 1);
			gp_Pnt midEdge = bac.Value((bac.FirstParameter() + bac.LastParameter()) / 2);
			if (!common.IsNull())
			{
				BRepAdaptor_Surface xbrs(TopoDS::Face(iter2));
				double xuf = xbrs.FirstUParameter();
				double xvf = xbrs.FirstVParameter();
				double xue = xbrs.LastUParameter();
				double xve = xbrs.LastVParameter();
				gp_Pnt midFace2 = xbrs.Value((xue + xuf) / 2, (xve + xvf) / 2);
				/*gp_Dir xdir;
				if (!FaceNormal(TopoDS::Face(iter2), (xue + xuf) / 2, (xve + xvf) / 2, xdir))
					continue;
				gp_Dir newdir = xdir.Crossed(dir);*/
				gp_Dir edgeFace1(midFace1.X() - midEdge.X(),
					midFace1.Y() - midEdge.Y(),
					midFace1.Z() - midEdge.Z());
				gp_Dir edgeFace2(midFace2.X() - midEdge.X(),
					midFace2.Y() - midEdge.Y(),
					midFace2.Z() - midEdge.Z());

				if (edgeFace2.Angle(edgeFace1)>M_PI)
				{
					if (!usedfeatures.Contains(iter))
					{
						usedfeatures.Append(iter);
						featurefaces.push_back(TopoDS::Face(iter));
					}
					if (!usedfeatures.Contains(iter2))
					{
						usedfeatures.Append(iter2);
						featurefaces.push_back(TopoDS::Face(iter2));
					}
				}
			}
		}
	}

	for (auto iter : facepackage)
	{
		if (!usedfeatures.Contains(iter))
			reffeaturefaces.Append(iter);
	}

	facepackage = reffeaturefaces;
	return true;
}
//分离step文件各特征方法 end

bool iogOccKernelTools::FaceNormal(const TopoDS_Face face, const double& u, 
	const double& v, gp_Dir& dir)
{
	BRepGProp_Face anaface(face);
	double umin, umax, vmin, vmax;
	anaface.Bounds(umin, umax, vmin, vmax);
	gp_Pnt pnt;
	gp_Vec normal;
	anaface.Normal(u, v, pnt, normal);
	if (normal.Magnitude() == 0)
	{
		Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(face));
		GeomAbs_SurfaceType type;
		Handle(Geom_Surface) basicSurf = GetBasicSurface(surface, type);
		if (basicSurf->IsKind(STANDARD_TYPE(Geom_SurfaceOfRevolution)))
		{
			Handle(Geom_SurfaceOfRevolution) finalSurf = Handle(Geom_SurfaceOfRevolution)::DownCast(basicSurf);
			normal = finalSurf->Axis().Direction();
		}
		else if (basicSurf->IsKind(STANDARD_TYPE(Geom_ConicalSurface)))
		{
			Handle(Geom_ConicalSurface) finalSurf = Handle(Geom_ConicalSurface)::DownCast(basicSurf);
			normal = finalSurf->Axis().Direction();
		}
		else if (basicSurf->IsKind(STANDARD_TYPE(Geom_SphericalSurface)))
		{
			Handle(Geom_SphericalSurface) finalSurf = Handle(Geom_SphericalSurface)::DownCast(basicSurf);
			normal = finalSurf->Axis().Direction();
		}
		else
			return false;
	}
	dir = gp_Dir(normal);

	return true;
}

Handle(Geom_Surface) iogOccKernelTools::GetBasicSurface(Handle(Geom_Surface) surf, GeomAbs_SurfaceType& type)
{
	const Handle(Standard_Type)& thetype = surf->DynamicType();
	if (thetype == STANDARD_TYPE(Geom_RectangularTrimmedSurface))
	{
		return GetBasicSurface(Handle(Geom_RectangularTrimmedSurface)::DownCast(surf)->BasisSurface(), type);
	}
	else if (thetype == STANDARD_TYPE(Geom_Plane))
		type = GeomAbs_Plane;
	else if (thetype == STANDARD_TYPE(Geom_CylindricalSurface))
		type = GeomAbs_Cylinder;
	else if (thetype == STANDARD_TYPE(Geom_ConicalSurface))
		type = GeomAbs_Cone;
	else if (thetype == STANDARD_TYPE(Geom_SphericalSurface))
		type = GeomAbs_Sphere;
	else if (thetype == STANDARD_TYPE(Geom_ToroidalSurface))
		type = GeomAbs_Torus;
	else if (thetype == STANDARD_TYPE(Geom_SurfaceOfRevolution))
		type = GeomAbs_SurfaceOfRevolution;
	else if (thetype == STANDARD_TYPE(Geom_SurfaceOfLinearExtrusion))
		type = GeomAbs_SurfaceOfExtrusion;
	else if (thetype == STANDARD_TYPE(Geom_BezierSurface))
		type = GeomAbs_BezierSurface;
	else if (thetype == STANDARD_TYPE(Geom_BSplineSurface))
		type = GeomAbs_BSplineSurface;
	else if (thetype == STANDARD_TYPE(Geom_OffsetSurface))
		type = GeomAbs_OffsetSurface;
	else
		type = GeomAbs_OtherSurface;
	return surf;
}

bool iogOccKernelTools::GroupEdgesInTrianglestest(std::vector<TopoDS_Edge>& vecEdges,
	int& stop)
{
	if (stop < 3)
	{
		return false;
	}

	for (int j = 1; j < vecEdges.size(); j++)
	{
		for (int k = 1; k < vecEdges.size(); k++)
		{
			int a = 0;
		}
	}
	stop--;
	return GroupEdgesInTrianglestest(vecEdges, stop);

}

bool iogOccKernelTools::GroupEdgesInTriangles(double dPointEqual, std::vector<TopoDS_Edge>& vecEdges,
	std::vector<std::vector<TopoDS_Edge>>& vecvecGroups)
{
	if (vecEdges.size() == 0)
	{
		return false;
	}
	if (vecEdges.size() < 3)
	{
		vecvecGroups.push_back(vecEdges);
		vecEdges.clear();
		return false;
	}

	std::vector<int>* vecUsedIdx = new std::vector<int>;

	std::vector<TopoDS_Edge>* vecTopoEdges = new std::vector<TopoDS_Edge>;
	ShapeAnalysis_Edge sae;
	TopoDS_Edge eg1 = vecEdges[0];
	TopoDS_Vertex v11 = sae.FirstVertex(eg1);
	gp_Pnt pt11 = BRep_Tool::Pnt(v11);
	TopoDS_Vertex v12 = sae.LastVertex(eg1);
	gp_Pnt pt12 = BRep_Tool::Pnt(v12);
	for (int j = 1; j < vecEdges.size(); j++)
	{
		bool ifbreak = false;

		TopoDS_Edge eg2 = vecEdges[j];
		TopoDS_Vertex v21 = sae.FirstVertex(eg2);
		gp_Pnt pt21 = BRep_Tool::Pnt(v21);
		TopoDS_Vertex v22 = sae.LastVertex(eg2);
		gp_Pnt pt22 = BRep_Tool::Pnt(v22);
		if (pt12.IsEqual(pt21, dPointEqual))
		{
			for (int k = 1; k < vecEdges.size(); k++)
			{
				if (k == j)
				{
					continue;
				}
				TopoDS_Edge eg3 = vecEdges[k];
				TopoDS_Vertex v31 = sae.FirstVertex(eg3);
				gp_Pnt pt31 = BRep_Tool::Pnt(v31);
				TopoDS_Vertex v32 = sae.LastVertex(eg3);
				gp_Pnt pt32 = BRep_Tool::Pnt(v32);
				if ((pt31.IsEqual(pt11, dPointEqual) && pt32.IsEqual(pt22, dPointEqual)) ||
					(pt31.IsEqual(pt22, dPointEqual) && pt32.IsEqual(pt11, dPointEqual)))
				{
					vecTopoEdges->push_back(eg1);
					vecTopoEdges->push_back(eg2);
					vecTopoEdges->push_back(eg3);

					vecUsedIdx->push_back(0);
					vecUsedIdx->push_back(j);
					vecUsedIdx->push_back(k);

					ifbreak = true;
					break;
				}
			}
		}
		else if (pt11.IsEqual(pt22, dPointEqual))
		{
			for (int k = 1; k < vecEdges.size(); k++)
			{
				if (k == j)
				{
					continue;
				}
				TopoDS_Edge eg3 = vecEdges[k];
				TopoDS_Vertex v31 = sae.FirstVertex(eg3);
				gp_Pnt pt31 = BRep_Tool::Pnt(v31);
				TopoDS_Vertex v32 = sae.LastVertex(eg3);
				gp_Pnt pt32 = BRep_Tool::Pnt(v32);
				if ((pt31.IsEqual(pt12, dPointEqual) && pt32.IsEqual(pt21, dPointEqual)) ||
					(pt31.IsEqual(pt21, dPointEqual) && pt32.IsEqual(pt12, dPointEqual)))
				{
					vecTopoEdges->push_back(eg1);
					vecTopoEdges->push_back(eg2);
					vecTopoEdges->push_back(eg3);

					vecUsedIdx->push_back(0);
					vecUsedIdx->push_back(j);
					vecUsedIdx->push_back(k);

					ifbreak = true;
					break;
				}
			}
		}
		else if (pt11.IsEqual(pt21, dPointEqual))
		{
			for (int k = 1; k < vecEdges.size(); k++)
			{
				if (k == j)
				{
					continue;
				}
				TopoDS_Edge eg3 = vecEdges[k];
				TopoDS_Vertex v31 = sae.FirstVertex(eg3);
				gp_Pnt pt31 = BRep_Tool::Pnt(v31);
				TopoDS_Vertex v32 = sae.LastVertex(eg3);
				gp_Pnt pt32 = BRep_Tool::Pnt(v32);
				if ((pt31.IsEqual(pt12, dPointEqual) && pt32.IsEqual(pt22, dPointEqual)) ||
					(pt31.IsEqual(pt22, dPointEqual) && pt32.IsEqual(pt12, dPointEqual)))
				{
					vecTopoEdges->push_back(eg1);
					vecTopoEdges->push_back(eg2);
					vecTopoEdges->push_back(eg3);

					vecUsedIdx->push_back(0);
					vecUsedIdx->push_back(j);
					vecUsedIdx->push_back(k);

					ifbreak = true;
					break;
				}
			}
		}
		else if (pt12.IsEqual(pt22, dPointEqual))
		{
			for (int k = 1; k < vecEdges.size(); k++)
			{
				if (k == j)
				{
					continue;
				}
				TopoDS_Edge eg3 = vecEdges[k];
				TopoDS_Vertex v31 = sae.FirstVertex(eg3);
				gp_Pnt pt31 = BRep_Tool::Pnt(v31);
				TopoDS_Vertex v32 = sae.LastVertex(eg3);
				gp_Pnt pt32 = BRep_Tool::Pnt(v32);
				if ((pt31.IsEqual(pt11, dPointEqual) && pt32.IsEqual(pt21, dPointEqual)) ||
					(pt31.IsEqual(pt21, dPointEqual) && pt32.IsEqual(pt11, dPointEqual)))
				{
					vecTopoEdges->push_back(eg1);
					vecTopoEdges->push_back(eg2);
					vecTopoEdges->push_back(eg3);

					vecUsedIdx->push_back(0);
					vecUsedIdx->push_back(j);
					vecUsedIdx->push_back(k);

					ifbreak = true;
					break;
				}
			}
		}

		if (ifbreak)
			break;
	}
	if (vecTopoEdges->size() == 0)
	{
		vecTopoEdges->push_back(eg1);
		vecvecGroups.push_back(*vecTopoEdges);

		std::vector<TopoDS_Edge>* nextEdges = new std::vector<TopoDS_Edge>;
		for (int i = 1; i < vecEdges.size(); i++)
		{
			nextEdges->push_back(vecEdges[i]);
		}
		vecEdges = *nextEdges;

		delete nextEdges;
		nextEdges = NULL;

		delete vecUsedIdx;
		vecUsedIdx = NULL;

		delete vecTopoEdges;
		vecTopoEdges = NULL;

		return GroupEdgesInTriangles(dPointEqual, vecEdges, vecvecGroups);
	}
	else if (vecTopoEdges->size() == 3)
	{
		vecvecGroups.push_back(*vecTopoEdges);
		std::vector<TopoDS_Edge>* nextEdges = new std::vector<TopoDS_Edge>;
		for (int i = 0; i < vecEdges.size(); i++)
		{
			bool ifcontinue = false;
			for (int j = 0; j < vecUsedIdx->size(); j++)
			{
				if (i == (*vecUsedIdx)[j])
				{
					ifcontinue = true;
					break;
				}
			}
			if (ifcontinue)
				continue;
			nextEdges->push_back(vecEdges[i]);
		}
		vecEdges = *nextEdges;

		delete nextEdges;
		nextEdges = NULL;

		delete vecUsedIdx;
		vecUsedIdx = NULL;

		delete vecTopoEdges;
		vecTopoEdges = NULL;

		return GroupEdgesInTriangles(dPointEqual, vecEdges, vecvecGroups);
	}

}

bool iogOccKernelTools::GroupEdgesInCompLoop(TopoDS_Edge edge, std::vector<int>& vecUsedIndex,
	std::vector<TopoDS_Edge>& vecEdges, std::vector<TopoDS_Edge>& vecXGroup)
{
	ShapeAnalysis_Edge sae;
	TopoDS_Vertex vtx12 = sae.LastVertex(edge);
	gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);
	TopoDS_Vertex vtx11 = sae.FirstVertex(edge);
	gp_Pnt pt11 = BRep_Tool::Pnt(vtx11);

	for (int i = 0; i < vecEdges.size(); i++)
	{
		bool ifcontinue = false;
		for (int j = 0; j < vecUsedIndex.size(); j++)
		{
			if (i == vecUsedIndex[j])
			{
				ifcontinue = true;
				break;
			}
		}
		if (ifcontinue)
		{
			continue;
		}
		TopoDS_Edge edge2 = TopoDS::Edge(vecEdges[i]);
		TopoDS_Vertex vtx22 = sae.LastVertex(edge2);
		gp_Pnt pt22 = BRep_Tool::Pnt(vtx22);
		TopoDS_Vertex vtx21 = sae.FirstVertex(edge2);
		gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);

		if (pt12.IsEqual(pt21, 0.01) || pt11.IsEqual(pt22, 0.01) ||
			(pt11.IsEqual(pt21, 0.01) || pt12.IsEqual(pt22, 0.01)))
		{
			vecUsedIndex.push_back(i);
			vecXGroup.push_back(edge2);
			GroupEdgesInCompLoop(edge2, vecUsedIndex, vecEdges, vecXGroup);
		}
	}
	return true;
}

bool iogOccKernelTools::GroupEdgesInComp(std::vector<TopoDS_Edge>& vecEdges,
	std::vector<std::vector<TopoDS_Edge>>& vecvecGroups)
{
	if (vecEdges.size() < 1)
	{
		return false;
	}
	if (vecEdges.size() == 1)
	{
		vecvecGroups.push_back(vecEdges);
	}

	std::vector<int> vecUsedIndex;

	for (int i = 0; i < vecEdges.size(); i++)
	{
		bool ifcontinue = false;
		for (int m = 0; m < vecUsedIndex.size(); m++)
		{
			if (i == vecUsedIndex[m])
			{
				ifcontinue = true;
				break;
			}
		}
		if (ifcontinue)
		{
			continue;
		}

		vecUsedIndex.push_back(i);
		std::vector<TopoDS_Edge> vecXGroup;
		ShapeAnalysis_Edge sae;
		TopoDS_Edge edge1 = TopoDS::Edge(vecEdges[i]);
		vecXGroup.push_back(edge1);
		TopoDS_Vertex vtx11 = sae.FirstVertex(edge1);
		gp_Pnt pt11 = BRep_Tool::Pnt(vtx11);
		TopoDS_Vertex vtx12 = sae.LastVertex(edge1);
		gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);
		for (int j = 0; j < vecEdges.size(); j++)
		{
			if (i == j)
			{
				continue;
			}

			bool ifcontinue = false;
			for (int k = 0; k < vecUsedIndex.size(); k++)
			{
				if (j == vecUsedIndex[k])
				{
					ifcontinue = true;
					break;
				}
			}
			if (ifcontinue)
			{
				continue;
			}

			TopoDS_Edge edge2 = TopoDS::Edge(vecEdges[j]);
			TopoDS_Vertex vtx22 = sae.LastVertex(edge2);
			gp_Pnt pt22 = BRep_Tool::Pnt(vtx22);
			TopoDS_Vertex vtx21 = sae.FirstVertex(edge2);
			gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);
			if (pt12.IsEqual(pt21, 0.01) || pt11.IsEqual(pt22, 0.01) ||
				(pt11.IsEqual(pt21, 0.01) || pt12.IsEqual(pt22, 0.01)))
			{
				vecUsedIndex.push_back(j);
				vecXGroup.push_back(edge2);
				GroupEdgesInCompLoop(edge2, vecUsedIndex, vecEdges, vecXGroup);
			}
		}
		vecvecGroups.push_back(vecXGroup);
	}
	return true;
}

bool iogOccKernelTools::GetShapeNameMap(Handle(XCAFDoc_ShapeTool)& assembly, 
	TDF_Label& freeshape, std::vector<std::pair<TopoDS_Shape, std::string>>& vecnamemap)
{
	Handle(TDataStd_Name) name;
	TCollection_ExtendedString nametext;
	TDF_LabelSequence components;
	assembly->GetComponents(freeshape, components, true);

	for (int j = 1; j <= components.Length(); j++)
	{
		TDF_Label complabel = components.Value(j);
		if (complabel.FindAttribute(TDataStd_Name::GetID(), name))
		{
			nametext = name->Get();
			std::string s1 = TCollection_AsciiString(nametext).ToCString();

			TopoDS_Shape compareShape = assembly->GetShape(freeshape);
			vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
		}
		GetShapeNameMap(assembly, complabel, vecnamemap);
	}

	return true;
}

bool iogOccKernelTools::GetShapeNameMap(TopoDS_Shape& inputShape, TopTools_ListOfShape& shapeList)
{
	shapeList.Append(inputShape);
	TopoDS_Iterator topoIter(inputShape);
	for (; topoIter.More(); topoIter.Next())
	{
		TopoDS_Shape subShape = topoIter.Value();
		shapeList.Append(subShape);
		GetShapeNameMap(subShape, shapeList);
	}
	return true;
}

bool iogOccKernelTools::GetDocShape(Handle(TDocStd_Document) doc, TopoDS_Shape& resultshape)
{
	if (!doc)
	{
		return false;
	}

	TDF_Label mainLabel = doc->Main();
	TDF_LabelSequence freeshapes;
	Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(mainLabel);
	shapeTool->GetFreeShapes(freeshapes);
	int nFreeShapeNum = freeshapes.Length();
	TDF_Label startLabel = freeshapes.Value(1);
	resultshape = XCAFDoc_ShapeTool::GetShape(startLabel);
	//LoopBuildDocShape(startLabel, resultShape);

	return true;
}

bool iogOccKernelTools::LoopBuildDocShape(TDF_Label mainLabel, TopoDS_Shape& resultShape)
{
	BRep_Builder b;

	resultShape = XCAFDoc_ShapeTool::GetShape(mainLabel);
	if (resultShape.IsNull())
	{
		return false;
	}
	TopAbs_ShapeEnum shapeType = resultShape.ShapeType();
	if (shapeType == TopAbs_COMPOUND)
	{
		b.MakeCompound(TopoDS::Compound(resultShape));
	}
	else if (shapeType == TopAbs_SOLID)
	{
		b.MakeSolid(TopoDS::Solid(resultShape));
	}
	else if (shapeType == TopAbs_SHELL)
	{
		b.MakeShell(TopoDS::Shell(resultShape));
	}
	else
	{
		return true;
	}

	TDF_ChildIterator it;
	for (it.Initialize(mainLabel); it.More(); it.Next())
	{
		TDF_Label childLabel = it.Value();
		TopoDS_Shape childShape = XCAFDoc_ShapeTool::GetShape(childLabel);
		b.Add(resultShape, childShape);
		LoopBuildDocShape(childLabel, childShape);
	}
	return true;
}

TopoDS_Shape iogOccKernelTools::ReBuildShape(int nAllFaces, int& nCount, TopoDS_Shape entryShape, std::vector<std::string> vecUniqName,
	std::vector<std::pair<TopoDS_Shape, std::string>> vecCSNameMap,
	std::vector<std::pair<TopoDS_Shape, std::string>> vecFaceNameMap,
	QHash<QString, std::string> hashFace,
	std::vector<std::pair<TopoDS_Shape, std::string>>& vecReCSNameMap,
	std::vector<std::pair<TopoDS_Shape, std::string>>& vecReFaceNameMap,
	QHash<QString, int>& hashUsedFace)
{
	TopoDS_Shape result;
	if (entryShape.ShapeType() == TopAbs_COMPOUND)
	{
		BRep_Builder b;
		TopoDS_Compound compound;
		b.MakeCompound(compound);
		TopoDS_Iterator subiter(entryShape);
		std::vector<TopoDS_Shape> vecSubFaces;
		for (; subiter.More(); subiter.Next())
		{
			TopoDS_Shape currentShape = subiter.Value();
			if (currentShape.ShapeType() == TopAbs_COMPOUND || currentShape.ShapeType() == TopAbs_SOLID)
			{
				currentShape = ReBuildShape(nAllFaces, nCount, currentShape,
					vecUniqName, vecCSNameMap, vecFaceNameMap, hashFace, vecReCSNameMap, vecReFaceNameMap, hashUsedFace);
				b.Add(compound, currentShape);
			}
			else if (currentShape.ShapeType() == TopAbs_SHELL || currentShape.ShapeType() == TopAbs_FACE)
			{
				TopExp_Explorer faceEx(currentShape, TopAbs_FACE);
				for (; faceEx.More(); faceEx.Next())
				{
					vecSubFaces.push_back(faceEx.Current());
				}
			}
		}
		if (vecSubFaces.size() > 0)
		{
			std::vector<struShellFaces> vecShellFaces;
			for (int i = 0; i < vecUniqName.size(); i++)
			{
				struShellFaces currentStruc;
				std::string strShellName = vecUniqName[i];
				TopTools_ListOfShape faceList;
				currentStruc.faceList = faceList;
				currentStruc.name = strShellName;
				vecShellFaces.push_back(currentStruc);
			}

			//can not avoid
			for (int i = 0; i < vecSubFaces.size(); i++)
			{
				/*DWORD dStart = GetTickCount();*/
				nCount++;
				double dProgress = nCount * 100 / nAllFaces;
				TheExe.ProgressSetValue(dProgress);
				TopoDS_Shape currentFace = vecSubFaces[i];
				std::string strSurfaceName;

				//
				QString qstrCompare1;
				{
					BRepAdaptor_Surface adp_sur1(TopoDS::Face(currentFace));
					double ustart1 = adp_sur1.FirstUParameter();
					double uend1 = adp_sur1.LastUParameter();
					double vstart1 = adp_sur1.FirstVParameter();
					double vend1 = adp_sur1.LastVParameter();
					gp_Pnt pntUsVsSurf1 = adp_sur1.Value(ustart1, vstart1);
					double dXPntUsVsSurf1 = pntUsVsSurf1.X();
					double dYPntUsVsSurf1 = pntUsVsSurf1.Y();
					double dZPntUsVsSurf1 = pntUsVsSurf1.Z();
					QString qstr1 = QString::number(dXPntUsVsSurf1, 'f', 5) + ","
						+ QString::number(dYPntUsVsSurf1, 'f', 5) + ","
						+ QString::number(dZPntUsVsSurf1, 'f', 5) + ",";
					gp_Pnt pntUeVsSurf1 = adp_sur1.Value(uend1, vstart1);
					double dXPntUeVsSurf1 = pntUeVsSurf1.X();
					double dYPntUeVsSurf1 = pntUeVsSurf1.Y();
					double dZPntUeVsSurf1 = pntUeVsSurf1.Z();
					QString qstr3 = QString::number(dXPntUeVsSurf1, 'f', 5) + ","
						+ QString::number(dYPntUeVsSurf1, 'f', 5) + ","
						+ QString::number(dZPntUeVsSurf1, 'f', 5) + ",";
					gp_Pnt pntUsVeSurf1 = adp_sur1.Value(ustart1, vend1);
					double dXPntUsVeSurf1 = pntUsVeSurf1.X();
					double dYPntUsVeSurf1 = pntUsVeSurf1.Y();
					double dZPntUsVeSurf1 = pntUsVeSurf1.Z();
					QString qstr5 = QString::number(dXPntUsVeSurf1, 'f', 5) + ","
						+ QString::number(dYPntUsVeSurf1, 'f', 5) + ","
						+ QString::number(dZPntUsVeSurf1, 'f', 5) + ",";
					gp_Pnt pntUeVeSurf1 = adp_sur1.Value(vend1, vend1);
					double dXPntUeVeSurf1 = pntUeVeSurf1.X();
					double dYPntUeVeSurf1 = pntUeVeSurf1.Y();
					double dZPntUeVeSurf1 = pntUeVeSurf1.Z();
					QString qstr7 = QString::number(dXPntUeVeSurf1, 'f', 5) + ","
						+ QString::number(dYPntUeVeSurf1, 'f', 5) + ","
						+ QString::number(dZPntUeVeSurf1, 'f', 5) + ",";
					gp_Pnt pntUfVfSurf1 = adp_sur1.Value(0.5 * (ustart1 + uend1), 0.5 * (ustart1 + uend1));
					double dXPntUfVfSurf1 = pntUfVfSurf1.X();
					double dYPntUfVfSurf1 = pntUfVfSurf1.Y();
					double dZPntUfVfSurf1 = pntUfVfSurf1.Z();
					QString qstr9 = QString::number(dXPntUfVfSurf1, 'f', 5) + ","
						+ QString::number(dYPntUfVfSurf1, 'f', 5) + ","
						+ QString::number(dZPntUfVfSurf1, 'f', 5) + ",";
					qstrCompare1 = qstr1 + qstr3 + qstr5 + qstr7 + qstr9; //qstr1 + qstr3 + qstr5 + qstr7 + 
				}

				//take current face name
				/*for (int j = 0; j < vecFaceNameMap.size(); j++)
				{
					std::pair<TopoDS_Shape, std::string> currentpair = vecFaceNameMap[j];
					TopoDS_Shape currentpairFace = currentpair.first;
					TopAbs_ShapeEnum curFaceType = currentpairFace.ShapeType();
					if (curFaceType != TopAbs_FACE)
					{
						continue;
					}

					QString qstrCompare2;
					{
						BRepAdaptor_Surface adp_sur2(TopoDS::Face(currentpairFace));
						double ustart2 = adp_sur2.FirstUParameter();
						double uend2 = adp_sur2.LastUParameter();
						double vstart2 = adp_sur2.FirstVParameter();
						double vend2 = adp_sur2.LastVParameter();
						gp_Pnt pntUsVsSurf2 = adp_sur2.Value(ustart2, vstart2);
						double dXPntUsVsSurf2 = pntUsVsSurf2.X();
						double dYPntUsVsSurf2 = pntUsVsSurf2.Y();
						double dZPntUsVsSurf2 = pntUsVsSurf2.Z();
						QString qstr2 = QString::number(dXPntUsVsSurf2, 'f', 5) + ","
							+ QString::number(dYPntUsVsSurf2, 'f', 5) + ","
							+ QString::number(dZPntUsVsSurf2, 'f', 5);
						gp_Pnt pntUeVsSurf2 = adp_sur2.Value(uend2, vstart2);
						double dXPntUeVsSurf2 = pntUeVsSurf2.X();
						double dYPntUeVsSurf2 = pntUeVsSurf2.Y();
						double dZPntUeVsSurf2 = pntUeVsSurf2.Z();
						QString qstr4 = QString::number(dXPntUeVsSurf2, 'f', 5) + ","
							+ QString::number(dYPntUeVsSurf2, 'f', 5) + ","
							+ QString::number(dZPntUeVsSurf2, 'f', 5);
						gp_Pnt pntUsVeSurf2 = adp_sur2.Value(ustart2, vend2);
						double dXPntUsVeSurf2 = pntUsVeSurf2.X();
						double dYPntUsVeSurf2 = pntUsVeSurf2.Y();
						double dZPntUsVeSurf2 = pntUsVeSurf2.Z();
						QString qstr6 = QString::number(dXPntUsVeSurf2, 'f', 5) + ","
							+ QString::number(dYPntUsVeSurf2, 'f', 5) + ","
							+ QString::number(dZPntUsVeSurf2, 'f', 5);
						gp_Pnt pntUeVeSurf2 = adp_sur2.Value(vend2, vend2);
						double dXPntUeVeSurf2 = pntUeVeSurf2.X();
						double dYPntUeVeSurf2 = pntUeVeSurf2.Y();
						double dZPntUeVeSurf2 = pntUeVeSurf2.Z();
						QString qstr8 = QString::number(dXPntUeVeSurf2, 'f', 5) + ","
							+ QString::number(dYPntUeVeSurf2, 'f', 5) + ","
							+ QString::number(dZPntUeVeSurf2, 'f', 5);
						gp_Pnt pntUfVfSurf2 = adp_sur2.Value(0.5 * (ustart2 + uend2), 0.5 * (ustart2 + uend2));
						double dXPntUfVfSurf2 = pntUfVfSurf2.X();
						double dYPntUfVfSurf2 = pntUfVfSurf2.Y();
						double dZPntUfVfSurf2 = pntUfVfSurf2.Z();
						QString qstr10 = QString::number(dXPntUfVfSurf2, 'f', 5) + ","
							+ QString::number(dYPntUfVfSurf2, 'f', 5) + ","
							+ QString::number(dZPntUfVfSurf2, 'f', 5);

						qstrCompare2 = qstr2 + qstr4 + qstr6 + qstr8 + qstr10;
					}

					if (qstrCompare1 == qstrCompare2)
					{
						strSurfaceName = currentpair.second;
						break;
					}
				}*/

				if (hashUsedFace.contains(qstrCompare1))
					continue;

				hashUsedFace.insert(qstrCompare1, 0);

				if (hashFace.contains(qstrCompare1))
				{
					strSurfaceName = hashFace.value(qstrCompare1);
				}

				if (strSurfaceName == "")
				{
					strSurfaceName = "Sewed Surface";
				}
				//compare name to insert shell
				for (int k = 0; k < vecUniqName.size(); k++)
				{
					if (strSurfaceName == vecUniqName[k])
					{
						vecShellFaces[k].faceList.Append(currentFace);
						break;
					}
				}
				/*DWORD dEnd = GetTickCount();
				double dWaittime = (double)(dEnd - dStart) / 1000;
				int a = 0;*/
			}

			for (int i = 0; i < vecShellFaces.size(); i++)
			{
				struShellFaces currentStruc = vecShellFaces[i];
				if (currentStruc.faceList.Size() < 1)
				{
					continue;
				}
				else
				{
					std::string strShellName = currentStruc.name;
					TopoDS_Shell currentShell;
					BRep_Builder bShell;
					bShell.MakeShell(currentShell);
					for (auto iter : currentStruc.faceList)
					{
						bShell.Add(currentShell, iter);
					}
					vecReFaceNameMap.push_back(std::pair<TopoDS_Shape, std::string>(currentShell, strShellName));
					b.Add(compound, currentShell);
				}
			}
		}
		result = compound;

		std::string strAssName;
		for (int i = 0; i < vecCSNameMap.size(); i++)
		{
			std::pair<TopoDS_Shape, std::string> currentpair = vecCSNameMap[i];
			TopoDS_Shape currentpairAss = currentpair.first;
			if (currentpairAss.ShapeType() != TopAbs_COMPOUND)
			{
				continue;
			}
			if (entryShape.IsSame(currentpair.first))
			{
				strAssName = currentpair.second;
			}
		}
		if (strAssName != "")
		{
			vecReCSNameMap.push_back(std::pair<TopoDS_Shape, std::string>(compound, strAssName));
		}
	}
	else if (entryShape.ShapeType() == TopAbs_SOLID)
	{
		BRep_Builder b;
		TopoDS_Solid solid;
		b.MakeSolid(solid);
		TopoDS_Iterator subiter(entryShape);
		std::vector<TopoDS_Shape> vecSubFaces;
		for (; subiter.More(); subiter.Next())
		{
			TopoDS_Shape currentShape = subiter.Value();
			if (currentShape.ShapeType() == TopAbs_SHELL || currentShape.ShapeType() == TopAbs_FACE)
			{
				TopExp_Explorer faceEx(currentShape, TopAbs_FACE);
				for (; faceEx.More(); faceEx.Next())
				{
					vecSubFaces.push_back(faceEx.Current());
				}
			}
		}
		if (vecSubFaces.size() > 0)
		{
			std::vector<struShellFaces> vecShellFaces;
			for (int i = 0; i < vecUniqName.size(); i++)
			{
				struShellFaces currentStruc;
				std::string strShellName = vecUniqName[i];
				TopTools_ListOfShape faceList;
				currentStruc.faceList = faceList;
				currentStruc.name = strShellName;
				vecShellFaces.push_back(currentStruc);
			}

			for (int i = 0; i < vecSubFaces.size(); i++)
			{
				nCount++;
				double dProgress = nCount * 100 / nAllFaces;
				TheExe.ProgressSetValue(dProgress);
				TopoDS_Shape currentFace = vecSubFaces[i];
				std::string strSurfaceName;

				//
				QString qstrCompare1;
				{
					BRepAdaptor_Surface adp_sur1(TopoDS::Face(currentFace));
					double ustart1 = adp_sur1.FirstUParameter();
					double uend1 = adp_sur1.LastUParameter();
					double vstart1 = adp_sur1.FirstVParameter();
					double vend1 = adp_sur1.LastVParameter();
					gp_Pnt pntUsVsSurf1 = adp_sur1.Value(ustart1, vstart1);
					double dXPntUsVsSurf1 = pntUsVsSurf1.X();
					double dYPntUsVsSurf1 = pntUsVsSurf1.Y();
					double dZPntUsVsSurf1 = pntUsVsSurf1.Z();
					QString qstr1 = QString::number(dXPntUsVsSurf1, 'f', 5) + ","
						+ QString::number(dYPntUsVsSurf1, 'f', 5) + ","
						+ QString::number(dZPntUsVsSurf1, 'f', 5);
					gp_Pnt pntUeVsSurf1 = adp_sur1.Value(uend1, vstart1);
					double dXPntUeVsSurf1 = pntUeVsSurf1.X();
					double dYPntUeVsSurf1 = pntUeVsSurf1.Y();
					double dZPntUeVsSurf1 = pntUeVsSurf1.Z();
					QString qstr3 = QString::number(dXPntUeVsSurf1, 'f', 5) + ","
						+ QString::number(dYPntUeVsSurf1, 'f', 5) + ","
						+ QString::number(dZPntUeVsSurf1, 'f', 5);
					gp_Pnt pntUsVeSurf1 = adp_sur1.Value(ustart1, vend1);
					double dXPntUsVeSurf1 = pntUsVeSurf1.X();
					double dYPntUsVeSurf1 = pntUsVeSurf1.Y();
					double dZPntUsVeSurf1 = pntUsVeSurf1.Z();
					QString qstr5 = QString::number(dXPntUsVeSurf1, 'f', 5) + ","
						+ QString::number(dYPntUsVeSurf1, 'f', 5) + ","
						+ QString::number(dZPntUsVeSurf1, 'f', 5);
					gp_Pnt pntUeVeSurf1 = adp_sur1.Value(vend1, vend1);
					double dXPntUeVeSurf1 = pntUeVeSurf1.X();
					double dYPntUeVeSurf1 = pntUeVeSurf1.Y();
					double dZPntUeVeSurf1 = pntUeVeSurf1.Z();
					QString qstr7 = QString::number(dXPntUeVeSurf1, 'f', 5) + ","
						+ QString::number(dYPntUeVeSurf1, 'f', 5) + ","
						+ QString::number(dZPntUeVeSurf1, 'f', 5);
					gp_Pnt pntUfVfSurf1 = adp_sur1.Value(0.5 * (ustart1 + uend1), 0.5 * (ustart1 + uend1));
					double dXPntUfVfSurf1 = pntUfVfSurf1.X();
					double dYPntUfVfSurf1 = pntUfVfSurf1.Y();
					double dZPntUfVfSurf1 = pntUfVfSurf1.Z();
					QString qstr9 = QString::number(dXPntUfVfSurf1, 'f', 5) + ","
						+ QString::number(dYPntUfVfSurf1, 'f', 5) + ","
						+ QString::number(dZPntUfVfSurf1, 'f', 5);
					qstrCompare1 = qstr1 + qstr3 + qstr5 + qstr7 + qstr9; // = qstr1 + qstr3 + qstr5 + qstr7 + qstr9
				}

				//take current face name
				/*for (int j = 0; j < vecFaceNameMap.size(); j++)
				{
					std::pair<TopoDS_Shape, std::string> currentpair = vecFaceNameMap[j];
					TopoDS_Shape currentpairFace = currentpair.first;
					if (currentpairFace.ShapeType() != TopAbs_FACE)
					{
						continue;
					}

					QString qstrCompare2;
					{
						BRepAdaptor_Surface adp_sur2(TopoDS::Face(currentpairFace));
						double ustart2 = adp_sur2.FirstUParameter();
						double uend2 = adp_sur2.LastUParameter();
						double vstart2 = adp_sur2.FirstVParameter();
						double vend2 = adp_sur2.LastVParameter();
						gp_Pnt pntUsVsSurf2 = adp_sur2.Value(ustart2, vstart2);
						double dXPntUsVsSurf2 = pntUsVsSurf2.X();
						double dYPntUsVsSurf2 = pntUsVsSurf2.Y();
						double dZPntUsVsSurf2 = pntUsVsSurf2.Z();
						QString qstr2 = QString::number(dXPntUsVsSurf2, 'f', 5) + ","
							+ QString::number(dYPntUsVsSurf2, 'f', 5) + ","
							+ QString::number(dZPntUsVsSurf2, 'f', 5);
						gp_Pnt pntUeVsSurf2 = adp_sur2.Value(uend2, vstart2);
						double dXPntUeVsSurf2 = pntUeVsSurf2.X();
						double dYPntUeVsSurf2 = pntUeVsSurf2.Y();
						double dZPntUeVsSurf2 = pntUeVsSurf2.Z();
						QString qstr4 = QString::number(dXPntUeVsSurf2, 'f', 5) + ","
							+ QString::number(dYPntUeVsSurf2, 'f', 5) + ","
							+ QString::number(dZPntUeVsSurf2, 'f', 5);
						gp_Pnt pntUsVeSurf2 = adp_sur2.Value(ustart2, vend2);
						double dXPntUsVeSurf2 = pntUsVeSurf2.X();
						double dYPntUsVeSurf2 = pntUsVeSurf2.Y();
						double dZPntUsVeSurf2 = pntUsVeSurf2.Z();
						QString qstr6 = QString::number(dXPntUsVeSurf2, 'f', 5) + ","
							+ QString::number(dYPntUsVeSurf2, 'f', 5) + ","
							+ QString::number(dZPntUsVeSurf2, 'f', 5);
						gp_Pnt pntUeVeSurf2 = adp_sur2.Value(vend2, vend2);
						double dXPntUeVeSurf2 = pntUeVeSurf2.X();
						double dYPntUeVeSurf2 = pntUeVeSurf2.Y();
						double dZPntUeVeSurf2 = pntUeVeSurf2.Z();
						QString qstr8 = QString::number(dXPntUeVeSurf2, 'f', 5) + ","
							+ QString::number(dYPntUeVeSurf2, 'f', 5) + ","
							+ QString::number(dZPntUeVeSurf2, 'f', 5);
						gp_Pnt pntUfVfSurf2 = adp_sur2.Value(0.5 * (ustart2 + uend2), 0.5 * (ustart2 + uend2));
						double dXPntUfVfSurf2 = pntUfVfSurf2.X();
						double dYPntUfVfSurf2 = pntUfVfSurf2.Y();
						double dZPntUfVfSurf2 = pntUfVfSurf2.Z();
						QString qstr10 = QString::number(dXPntUfVfSurf2, 'f', 5) + ","
							+ QString::number(dYPntUfVfSurf2, 'f', 5) + ","
							+ QString::number(dZPntUfVfSurf2, 'f', 5);

						qstrCompare2 = qstr2 + qstr4 + qstr6 + qstr8 + qstr10;
					}

					if (qstrCompare1 == qstrCompare2)
					{
						strSurfaceName = currentpair.second;
						break;
					}
				}*/

				if (hashUsedFace.contains(qstrCompare1))
					continue;

				hashUsedFace.insert(qstrCompare1, 0);

				if (hashFace.contains(qstrCompare1))
				{
					strSurfaceName = hashFace.value(qstrCompare1);
				}

				if (strSurfaceName == "")
				{
					strSurfaceName = "Sewed Surface";
				}
				for (int k = 0; k < vecUniqName.size(); k++)
				{
					if (strSurfaceName == vecUniqName[k])
					{
						vecShellFaces[k].faceList.Append(currentFace);
					}
				}
			}

			for (int i = 0; i < vecShellFaces.size(); i++)
			{
				struShellFaces currentStruc = vecShellFaces[i];
				if (currentStruc.faceList.Size() < 1)
				{
					continue;
				}
				else
				{
					std::string strShellName = currentStruc.name;
					TopoDS_Shell currentShell;
					BRep_Builder bShell;
					bShell.MakeShell(currentShell);
					for (auto iter : currentStruc.faceList)
					{
						bShell.Add(currentShell, iter);
					}
					vecReFaceNameMap.push_back(std::pair<TopoDS_Shape, std::string>(currentShell, strShellName));
					b.Add(solid, currentShell);
				}
			}
		}
		result = solid;

		std::string strPrtName;
		for (int i = 0; i < vecCSNameMap.size(); i++)
		{
			std::pair<TopoDS_Shape, std::string> currentpair = vecCSNameMap[i];
			TopoDS_Shape currentpairSolid = currentpair.first;
			if (currentpairSolid.ShapeType() != TopAbs_SOLID)
			{
				continue;
			}
			if (entryShape.IsPartner(currentpair.first))
			{
				strPrtName = currentpair.second;
			}
		}
		if (strPrtName != "")
		{
			vecReCSNameMap.push_back(std::pair<TopoDS_Shape, std::string>(solid, strPrtName));
		}
	}
	return result;
}

//TopoDS_Shape iogOccKernelTools::ReBuildShape(TopoDS_Shape entryShape, std::vector<std::string> vecUniqName,
//	std::vector<std::pair<TopoDS_Shape, std::string>>& vecnamemap)
//{
//	TopoDS_Shape result;
//	if (entryShape.ShapeType() == TopAbs_COMPOUND)
//	{
//		BRep_Builder b;
//		TopoDS_Compound compound;
//		b.MakeCompound(compound);
//		TopoDS_Iterator subiter(entryShape);
//		std::vector<TopoDS_Shape> vecSubFaces;
//		for (; subiter.More(); subiter.Next())
//		{
//			TopoDS_Shape currentShape = subiter.Value();
//			if (currentShape.ShapeType() == TopAbs_COMPOUND || currentShape.ShapeType() == TopAbs_SOLID)
//			{
//				currentShape = ReBuildShape(currentShape, vecUniqName, vecnamemap);
//				b.Add(compound, currentShape);
//			}
//			else if (currentShape.ShapeType() == TopAbs_SHELL || currentShape.ShapeType() == TopAbs_FACE)
//			{
//				TopExp_Explorer faceEx(currentShape, TopAbs_FACE);
//				for (; faceEx.More(); faceEx.Next())
//				{
//					vecSubFaces.push_back(faceEx.Current());
//				}
//			}
//		}
//		if (vecSubFaces.size() > 0)
//		{
//			for (int i = 0; i < vecUniqName.size(); i++)
//			{
//				std::string strShellName = vecUniqName[i];
//				TopoDS_Shell currentShell;
//				BRep_Builder bShell;
//				bShell.MakeShell(currentShell);
//				for (int j = 0; j < vecSubFaces.size(); j++)
//				{
//					TopoDS_Shape currentFace = vecSubFaces[j];
//					std::string strSurfaceName;
//					//take current face name
//					for (int k = 0; k < vecnamemap.size(); k++)
//					{
//						std::pair<TopoDS_Shape, std::string> currentpair = vecnamemap[k];
//						TopoDS_Shape currentpairFace = currentpair.first;
//						if (currentpairFace.ShapeType() != TopAbs_FACE)
//						{
//							continue;
//						}
//
//						if (iogOccKernelTools::IsFaceSameRole(currentFace, currentpairFace))
//						{
//							strSurfaceName = currentpair.second;
//							break;
//						}
//					}
//					if (strSurfaceName == "")
//					{
//						strSurfaceName = "Sewed Surface";
//					}
//					if (strSurfaceName == strShellName)
//					{
//						bShell.Add(currentShell, currentFace);
//					}
//				}
//				vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(currentShell, strShellName));
//				b.Add(compound, currentShell);
//			}
//		}
//		result = compound;
//
//		std::string strAssName;
//		for (int i = 0; i < vecnamemap.size(); i++)
//		{
//			std::pair<TopoDS_Shape, std::string> currentpair = vecnamemap[i];
//			TopoDS_Shape currentpairAss = currentpair.first;
//			if (currentpairAss.ShapeType() != TopAbs_COMPOUND)
//			{
//				continue;
//			}
//			if (entryShape.IsSame(currentpair.first))
//			{
//				strAssName = currentpair.second;
//			}
//		}
//		if (strAssName != "")
//		{
//			vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(compound, strAssName));
//		}
//	}
//	else if (entryShape.ShapeType() == TopAbs_SOLID)
//	{
//		BRep_Builder b;
//		TopoDS_Solid solid;
//		b.MakeSolid(solid);
//		TopoDS_Iterator subiter(entryShape);
//		std::vector<TopoDS_Shape> vecSubFaces;
//		for (; subiter.More(); subiter.Next())
//		{
//			TopoDS_Shape currentShape = subiter.Value();
//			if (currentShape.ShapeType() == TopAbs_SHELL || currentShape.ShapeType() == TopAbs_FACE)
//			{
//				TopExp_Explorer faceEx(currentShape, TopAbs_FACE);
//				for (; faceEx.More(); faceEx.Next())
//				{
//					vecSubFaces.push_back(faceEx.Current());
//				}
//			}
//		}
//		if (vecSubFaces.size() > 0)
//		{
//			for (int i = 0; i < vecUniqName.size(); i++)
//			{
//				std::string strShellName = vecUniqName[i];
//				TopoDS_Shell currentShell;
//				BRep_Builder bShell;
//				bShell.MakeShell(currentShell);
//				for (int j = 0; j < vecSubFaces.size(); j++)
//				{
//					TopoDS_Shape currentFace = vecSubFaces[j];
//					std::string strSurfaceName;
//					//take current face name
//					for (int k = 0; k < vecnamemap.size(); k++)
//					{
//						std::pair<TopoDS_Shape, std::string> currentpair = vecnamemap[k];
//						TopoDS_Shape currentpairFace = currentpair.first;
//						if (currentpairFace.ShapeType() != TopAbs_FACE)
//						{
//							continue;
//						}
//
//						if (iogOccKernelTools::IsFaceSameRole(currentFace, currentpairFace))
//						{
//							strSurfaceName = currentpair.second;
//							break;
//						}
//					}
//					if (strSurfaceName == "")
//					{
//						strSurfaceName = "Sewed Surface";
//					}
//					if (strSurfaceName == strShellName)
//					{
//						bShell.Add(currentShell, currentFace);
//					}
//				}
//				vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(currentShell, strShellName));
//				b.Add(solid, currentShell);
//			}
//		}
//		result = solid;
//
//		std::string strPrtName;
//		for (int i = 0; i < vecnamemap.size(); i++)
//		{
//			std::pair<TopoDS_Shape, std::string> currentpair = vecnamemap[i];
//			TopoDS_Shape currentpairSolid = currentpair.first;
//			if (currentpairSolid.ShapeType() != TopAbs_SOLID)
//			{
//				continue;
//			}
//			if (entryShape.IsPartner(currentpair.first))
//			{
//				strPrtName = currentpair.second;
//			}
//		}
//		if (strPrtName != "")
//		{
//			vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(solid, strPrtName));
//		}
//	}
//	return result;
//}

bool iogOccKernelTools::IfModelRealSolid(TopoDS_Shape igesshape, bool& bIfSolid)
{
	TopAbs_ShapeEnum mainType = igesshape.ShapeType();
	if (mainType != TopAbs_COMPOUND)
		return true;
	TopoDS_Iterator iterator(igesshape);
	for (; iterator.More(); iterator.Next())
	{
		TopoDS_Shape subChild = iterator.Value();
		TopAbs_ShapeEnum subType = subChild.ShapeType();
		if (subType == TopAbs_COMPOUND)
		{
			IfModelRealSolid(subChild, bIfSolid);
		}
		else if (subChild.ShapeType() > TopAbs_SOLID)
		{
			bIfSolid = false;
			return true;
		}
	}
	return true;
}
