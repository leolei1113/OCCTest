#include "OCCBasicTools.h"
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <ShapeAnalysis_WireOrder.hxx>

#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>
#include <STEPControl_Controller.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI_Writer.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>

#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <TopExp_Explorer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <GeomConvert_CompCurveToBSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>

OCCBasicTools::OCCBasicTools()
{
}


OCCBasicTools::~OCCBasicTools()
{
}




void OCCBasicTools::SaveIges(const TopTools_ListOfShape& shapes, const std::string& igesFile)
{
	IGESControl_Controller::Init();
	IGESControl_Writer igesWriter;

	for (TopTools_ListIteratorOfListOfShape si(shapes); si.More(); si.Next())
	{
		igesWriter.AddShape(si.Value());
	}

	igesWriter.ComputeModel();
	igesWriter.Write(igesFile.c_str());
}

void OCCBasicTools::SaveStep(TopoDS_Shape& shapes, const std::string& stepFile)
{
	STEPControl_Writer aWriter;
	IFSelect_ReturnStatus aStat = aWriter.Transfer(shapes, STEPControl_AsIs);
	aStat = aWriter.Write(stepFile.c_str());
}

void OCCBasicTools::SaveStl(TopoDS_Shape& shapes, const std::string& stlFile)
{
	BRepMesh_IncrementalMesh(shapes, 0.1, Standard_True);
	StlAPI_Writer STLwriter;
	STLwriter.Write(shapes, stlFile.c_str());
}


TopoDS_Face OCCBasicTools::GetFaceFromPntList(std::vector<gp_Pnt> PntsList)
{
	TopTools_ListOfShape tmpEdgeList;

	for (int i = 0; i < PntsList.size(); ++i)
	{
		gp_Pnt NextPnt;
		if (i == PntsList.size() - 1)
		{
			NextPnt = PntsList[0];
		}
		else
		{
			NextPnt = PntsList[i + 1];
		}
		tmpEdgeList.Append(BRepBuilderAPI_MakeEdge(PntsList[i], NextPnt));
	}
	BRepBuilderAPI_MakeWire anWire;
	anWire.Add(tmpEdgeList);

	TopoDS_Wire Wire = anWire.Wire();
	return BRepBuilderAPI_MakeFace(Wire);


}

bool OCCBasicTools::IsShapeGeomSame(const TopoDS_Shape shape1, const TopoDS_Shape shape2, const TopAbs_ShapeEnum type)
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

Standard_Boolean OCCBasicTools::GetOrderWireFromEdges(std::vector<TopoDS_Edge> anEdges, TopoDS_Wire& OrderWire) 
{
	ShapeAnalysis_Edge anEdgeAnalyser;
	ShapeAnalysis_WireOrder aWireOrder;
	for (std::vector<TopoDS_Edge>::const_iterator i = anEdges.begin(); i != anEdges.end(); ++i)
	{
		TopoDS_Vertex aVf = anEdgeAnalyser.FirstVertex(*i);
		TopoDS_Vertex aVl = anEdgeAnalyser.LastVertex(*i);
		gp_Pnt aPf = BRep_Tool::Pnt(aVf);
		gp_Pnt aPl = BRep_Tool::Pnt(aVl);
		aWireOrder.Add(aPf.XYZ(), aPl.XYZ());
	}
	// 
	TopTools_ListOfShape aOrderedEdges;
	for (Standard_Integer e = 1; e <= aWireOrder.NbEdges(); ++e)
	{
		const TopoDS_Edge& anEdge = anEdges.at(e - 1);
		aOrderedEdges.Append(anEdge);
	}
	BRepBuilderAPI_MakeWire aWireMaker;
	aWireMaker.Add(aOrderedEdges);

	if (!aWireMaker.IsDone())
	{
		return Standard_False;
	}
	OrderWire = aWireMaker.Wire();
	return Standard_True;
}

//大小面重叠测试
bool OCCBasicTools::GetPlanarFaceApexs(TopoDS_Face face, TopTools_ListOfShape& vtxs,
	TopTools_ListOfShape& uselessEdges)
{
	std::vector<TopoDS_Edge> vecEdges;
	TopExp_Explorer edgex(face,TopAbs_EDGE);
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
			if (!dir1.IsParallel(dir2,0.0001))
			{
				if (!vtxs.Contains(vtx21))
				{
					vtxs.Append(vtx21);
				}
			}
			else
			{
				uselessEdges.Append(edge2);
			}
		}
	}
}

bool OCCBasicTools::ReOrgnizeEdgeOrderWire(TopTools_ListOfShape& edges, TopoDS_Vertex vtx,
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
		if (IsShapeGeomSame(vtx11, vtx, TopAbs_VERTEX)
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
}

bool  OCCBasicTools::GroupEdgesInTriangles(double dPointEqual, std::vector<TopoDS_Edge>& vecEdges,
	std::vector<std::vector<TopoDS_Edge>>& vecvecGroups)
{
	if (vecEdges.size() == 0)
		return false;
	if (vecEdges.size() < 3)
	{
		vecvecGroups.push_back(vecEdges);
		vecEdges.clear();
		return false;
	}

	std::vector<int> vecUsedIdx;
	std::vector<TopoDS_Edge> vecTopoEdges;
	ShapeAnalysis_Edge sae;
	TopoDS_Edge edge1 = vecEdges[0];
	TopoDS_Vertex vtx11 = sae.FirstVertex(edge1);
	gp_Pnt pt11 = BRep_Tool::Pnt(vtx11);
	TopoDS_Vertex vtx12 = sae.LastVertex(edge1);
	gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);

	for (int j = 1; j < vecEdges.size(); j++)
	{
		bool ifbreak = false;
		TopoDS_Edge edge2 = vecEdges[j];
		TopoDS_Vertex vtx21 = sae.FirstVertex(edge2);
		gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);
		TopoDS_Vertex vtx22 = sae.LastVertex(edge2);
		gp_Pnt pt22 = BRep_Tool::Pnt(vtx22);

		if (pt12.IsEqual(pt21, dPointEqual))
		{
			for (int k = 1; k < vecEdges.size(); k++)
			{
				if (k == j)
					continue;
				TopoDS_Edge edge3 = vecEdges[k];
				TopoDS_Vertex vtx31 = sae.FirstVertex(edge3);
				gp_Pnt pt31 = BRep_Tool::Pnt(vtx31);
				TopoDS_Vertex vtx32 = sae.LastVertex(edge3);
				gp_Pnt pt32 = BRep_Tool::Pnt(vtx32);
				if ((pt31.IsEqual(pt11, dPointEqual) && pt32.IsEqual(pt22, dPointEqual)) ||
					(pt31.IsEqual(pt22, dPointEqual) && pt32.IsEqual(pt11, dPointEqual)))
				{
					vecTopoEdges.push_back(edge1);
					vecTopoEdges.push_back(edge2);
					vecTopoEdges.push_back(edge3);

					vecUsedIdx.push_back(0);
					vecUsedIdx.push_back(j);
					vecUsedIdx.push_back(k);

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
					continue;
				TopoDS_Edge edge3 = vecEdges[k];
				TopoDS_Vertex vtx31 = sae.FirstVertex(edge3);
				gp_Pnt pt31 = BRep_Tool::Pnt(vtx31);
				TopoDS_Vertex vtx32 = sae.LastVertex(edge3);
				gp_Pnt pt32 = BRep_Tool::Pnt(vtx32);
				if ((pt31.IsEqual(pt12, dPointEqual) && pt32.IsEqual(pt21, dPointEqual)) ||
					(pt31.IsEqual(pt21, dPointEqual) && pt32.IsEqual(pt12, dPointEqual)))
				{
					vecTopoEdges.push_back(edge1);
					vecTopoEdges.push_back(edge2);
					vecTopoEdges.push_back(edge3);

					vecUsedIdx.push_back(0);
					vecUsedIdx.push_back(j);
					vecUsedIdx.push_back(k);

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
					continue;
				TopoDS_Edge edge3 = vecEdges[k];
				TopoDS_Vertex vtx31 = sae.FirstVertex(edge3);
				gp_Pnt pt31 = BRep_Tool::Pnt(vtx31);
				TopoDS_Vertex vtx32 = sae.LastVertex(edge3);
				gp_Pnt pt32 = BRep_Tool::Pnt(vtx32);
				if ((pt31.IsEqual(pt12, dPointEqual) && pt32.IsEqual(pt22, dPointEqual)) ||
					(pt31.IsEqual(pt22, dPointEqual) && pt32.IsEqual(pt12, dPointEqual)))
				{
					vecTopoEdges.push_back(edge1);
					vecTopoEdges.push_back(edge2);
					vecTopoEdges.push_back(edge3);

					vecUsedIdx.push_back(0);
					vecUsedIdx.push_back(j);
					vecUsedIdx.push_back(k);

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
					continue;
				TopoDS_Edge edge3 = vecEdges[k];
				TopoDS_Vertex vtx31 = sae.FirstVertex(edge3);
				gp_Pnt pt31 = BRep_Tool::Pnt(vtx31);
				TopoDS_Vertex vtx32 = sae.LastVertex(edge3);
				gp_Pnt pt32 = BRep_Tool::Pnt(vtx32);
				if ((pt31.IsEqual(pt11, dPointEqual) && pt32.IsEqual(pt21, dPointEqual)) ||
					(pt31.IsEqual(pt21, dPointEqual) && pt32.IsEqual(pt11, dPointEqual)))
				{
					vecTopoEdges.push_back(edge1);
					vecTopoEdges.push_back(edge2);
					vecTopoEdges.push_back(edge3);

					vecUsedIdx.push_back(0);
					vecUsedIdx.push_back(j);
					vecUsedIdx.push_back(k);

					ifbreak = true;
					break;
				}
			}
		}

		if (ifbreak)
			break;
	}
	if (vecTopoEdges.size() == 0)
	{
		vecTopoEdges.push_back(edge1);
		vecvecGroups.push_back(vecTopoEdges);

		std::vector<TopoDS_Edge> vecNextEdges;
		for (int i = 1; i < vecEdges.size(); i++)
		{
			vecNextEdges.push_back(vecEdges[i]);
		}

		return GroupEdgesInTriangles(dPointEqual, vecNextEdges, vecvecGroups);
	}
	else if (vecTopoEdges.size() == 3)
	{
		vecvecGroups.push_back(vecTopoEdges);
		std::vector<TopoDS_Edge> vecNextEdges;
		for (int i = 0; i < vecEdges.size(); i++)
		{
			bool ifcontinue = false;
			for (int j = 0; j < vecUsedIdx.size(); j++)
			{
				if (i == vecUsedIdx[j])
				{
					ifcontionue = true;
					break;
				}
			}
			if (ifcontinue)
				continue;
			vecNextEdges.push_back(vecEdges[i]);
			return GroupEdgesInTriangles(dPointEqual, vecNextEdges, vecvecGroups);
		}
	}
}

bool OCCBasicTools::GroupEdgesInCompLoop(TopoDS_Edge edge, std::vector<int> vecUsedIndex,
	std::vector<TopoDS_Edge>& vecEdges, std::vector<TopoDS_Edge> vecXGroup)
{
	ShapeAnalysis_Edge sae;
	TopoDS_Edge edge1 = TopoDS::Edge(vecEdges[i]);
	TopoDS_Vertex vtx12 = sae.LastVertex(edge1);
	gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);

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
		TopoDS_Edge edge2 = TopoDS::Edge(vecEdges[j]);
		TopoDS_Vertex vtx21 = sae.FirstVertex(edge2);
		gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);

		if (pt12.IsEqual(pt21, 0.01))
		{
			vecUsedIndex.push_back(i);
			vecXGroup.push_back(edge2);
			GroupEdgesInComp(edge2, vecUsedIndex, vecEdges, vecXGroup);
		}
	}
}

bool OCCBasicTools::GroupEdgesInComp(std::vector<TopoDS_Edge>& vecEdges, 
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
		vecUsedIndex.push_back(i);
		std::vector<TopoDS_Edge> vecXGroup;
		ShapeAnalysis_Edge sae;
		TopoDS_Edge edge1 = TopoDS::Edge(vecEdges[i]);
		vecXGroup.push_back(edge1);
		TopoDS_Vertex vtx12 = sae.LastVertex(edge1);
		gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);
		for (int j = 0; j < vecEdges.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			TopoDS_Edge edge2 = TopoDS::Edge(vecEdges[j]);
			TopoDS_Vertex vtx21 = sae.FirstVertex(edge2);
			gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);
			if (pt12.IsEqual(pt21, 0.01))
			{
				vecUsedIndex.push_back(j);
				vecXGroup.push_back(edge2);
				GroupEdgesInComp(edge2, vecUsedIndex, vecEdges, vecXGroup);
			}
		}
	}
}

bool OCCBasicTools::GroupEdgesInWire(std::vector<TopoDS_Shape>& edges, std::vector<std::vector<TopoDS_Shape>>& groups)
{
	if (edges.size() < 0)
		return false;
	std::vector<TopoDS_Shape> group;
	group.push_back(edges[0]);
	int count = -1;
	for (int i=0;i< edges.size()-1;i++)
	{
		TopoDS_Edge edge1 = edges[i];
		ShapeAnalysis_Edge sae1;
		TopoDS_Vertex vtx11 = sae1.FirstVertex(edge1);
		gp_Pnt pt11 = BRep_Tool::Pnt(vtx11);
		TopoDS_Vertex vtx12 = sae1.LastVertex(edge1);
		gp_Pnt pt12 = BRep_Tool::Pnt(vtx12);

		TopoDS_Edge edge2 = edges[i+1];
		ShapeAnalysis_Edge sae2;
		TopoDS_Vertex vtx21 = sae2.FirstVertex(edge2);
		gp_Pnt pt21 = BRep_Tool::Pnt(vtx21);
		TopoDS_Vertex vtx22 = sae2.LastVertex(edge2);
		gp_Pnt pt22 = BRep_Tool::Pnt(vtx22);

		gp_Dir dir1(pt12.X() - pt11.X(), pt12.Y() - pt11.Y(), pt12.Z() - pt11.Z());
		gp_Dir dir2(pt22.X() - pt21.X(), pt22.Y() - pt21.Y(), pt22.Z() - pt21.Z());
		if (dir1.IsParallel(dir2, 0.0001))
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

bool OCCBasicTools::ReOrderEdgesInWire(std::vector<std::vector<TopoDS_Shape>>& groups, TopoDS_Wire &wire)
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
	wire = mkwire.Wire();
}

//分离step文件各特征方法 start
bool OCCBasicTools::STPSeperateBodies(TopoDS_Shape input, TopTools_ListOfShape featurelists)
{
	//存一下所有面
	TopTools_ListOfShape allfaces;
	TopExp_Explorer faceex;
	for (faceex.Init(input, TopAbs_FACE); faceex.More(); faceex.Next())
	{
		allfaces.Append(faceex.Current())
	}
	//存放各个独立体的面
	std::vector<std::vector<TopoDS_Face>> independentbodyfaces;
	GetIndependentFeatureFacesAndStore(input, allfaces, independentbodyfaces);
}

bool OCCBasicTools::GetIndependentFeatureFacesAndStore(TopoDS_Shape shape, TopTools_ListOfShape& facepackage,
	std::vector<std::vector<TopoDS_Face>>& independentbodyfaces)
{

	while (facepackage.Size > 0)
	{
		std::vector<TopoDS_Face> featurefaces;
		if (!LoopOperate(shape, featurefaces, facepackage))
		{
			continue;
		}
		independentbodyfaces.push_back(featurefaces);
	}
}

bool OCCBasicTools::LoopOperate(TopoDS_Shape shape, std::vector<TopoDS_Face>& featurefaces, 
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
		gp_Dir dir;
		gp_Pnt mid = brs.Value((ue + uf) / 2, (ve + vf) / 2);     //面1中间点
		if (!ShapeUtilities::FaceNormal(TopoDS::Face(iter), (ue + uf) / 2, (ve + vf) / 2, dir))
			continue;
		for (auto iter2 : facepackage)
		{
			if (iter2.IsSame(iter))
				continue;
			if (usedfeatures.Contains(iter))
				continue;
			TopoDS_Shape common = BRepAlgoAPI_Section(iter, iter2).Shape();
			TopoDS_Edge commonedge;
			TopoDS_Iterator edgeit(common);
			for(; edgeit.More(); edgeit.Next())
			{
				TopoDS_Shape subShape = edgeit.Value();
				if (subShape.ShapeType() != TopAbs_EDGE)
				{
					continue;
				}
				commonedge = TopoDS::Edge(subShape);
			}
			BRepAdaptor_Curve bac(commonedge);
			gp_Pnt curMid = bac.Value((bac.FirstParameter() + bac.LastParameter()) / 2);
			gp_Dir edgedir = bac.DN(bac.FirstParameter(), 1);
			if (!common.IsNull())
			{
				BRepAdaptor_Surface xbrs(TopoDS::Face(iter2));
				double xuf = xbrs.FirstUParameter();
				double xvf = xbrs.FirstVParameter();
				double xue = xbrs.LastUParameter();
				double xve = xbrs.LastVParameter();
				gp_Pnt mid2 = xbrs.Value((xue + xuf) / 2, (xve + xvf) / 2);     //面2中间点
				gp_Dir xdir;
				if (!ShapeUtilities::FaceNormal(TopoDS::Face(iter2), (xue + xuf) / 2, (xve + xvf) / 2, xdir))
					continue;
				gp_Dir newdir = xdir.Crossed(dir);

				gp_Dir edgeFace1(mid.X() - curMid.X(), mid.Y() - curMid.Y(), mid.Z() - curMid.Z());
				gp_Dir edgeFace2(mid2.X() - curMid.X(), mid2.Y() - curMid.Y(), mid2.Z() - curMid.Z());

				if (edgeFace1.Angle(edgeFace2)>M_PI)  //newdir.IsEqual(edgedir, 0.01)
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
}
//分离step文件各特征方法 end