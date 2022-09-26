#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Pnt.hxx>
#include <vector>

class OCCBasicTools
{
public:
	OCCBasicTools();
	~OCCBasicTools();

	static void SaveIges(const TopTools_ListOfShape& shapes, const std::string& igesFile);
	static void SaveStep(TopoDS_Shape& shapes, const std::string& stepFile);
	static void SaveStl(TopoDS_Shape& shapes, const std::string& stlFile);
	static TopoDS_Face GetFaceFromPntList(std::vector<gp_Pnt> PntsList);
	static Standard_Boolean GetOrderWireFromEdges(std::vector<TopoDS_Edge> anEdges, TopoDS_Wire& OrderWire);
	static bool GetPlanarFaceApexs(TopoDS_Face face, TopTools_ListOfShape& vtxs);
};


