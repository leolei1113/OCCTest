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
bool OCCBasicTools::GetPlanarFaceApexs(TopoDS_Face face, TopTools_ListOfShape& vtxs)
{
	std::vector<TopoDS_Edge> vecEdges;
	TopExp_Explorer edgex(face);
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

		for (int j = 0; j < vecEdges.size(); i++)
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
			if (!dir1.IsParallel(dir2))
			{
				if (!vtxs.Contains(vtx21))
				{
					vtxs.Append(vtx21);
				}
			}
		}
	}
}