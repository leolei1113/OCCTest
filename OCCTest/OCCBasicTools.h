#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Pnt.hxx>
#include <vector>
#include <STEPCAFControl_Reader.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>
#include <Transfer_TransientProcess.hxx>
#include <Transfer_Binder.hxx>
#include <TransferBRep.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <StepRepr_NextAssemblyUsageOccurrence.hxx>

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
	static bool GetPlanarFaceApexs(TopoDS_Face face, TopTools_ListOfShape& vtxs,
		TopTools_ListOfShape& uselessEdges);
	static bool IsShapeGeomSame(const TopoDS_Shape shape1, const TopoDS_Shape shape2, const TopAbs_ShapeEnum type);
	static bool ReOrgnizeEdgeOrderWire(TopTools_ListOfShape& edges, TopoDS_Vertex vtx,
		std::vector<TopoDS_Shape>& newOrderEdges);
	static bool GroupEdgesInWire(std::vector<TopoDS_Shape>& edges, std::vector<std::vector<TopoDS_Shape>>& groups);
	static bool ReOrderEdgesInWire(std::vector<std::vector<TopoDS_Shape>>& groups, TopoDS_Wire& wire);
	static bool STPSeperateBodies(TopoDS_Shape input, TopTools_ListOfShape featurelists);
	static bool GetIndependentFeatureFacesAndStore(TopoDS_Shape shape, TopTools_ListOfShape& facepackage,
		std::vector<std::vector<TopoDS_Face>>& independentbodyfaces);
	static bool LoopOperate(TopoDS_Shape shape, std::vector<TopoDS_Face>& featurefaces,
		TopTools_ListOfShape& facepackage);

	static bool GroupEdgesInTriangles(double dPointEqual, std::vector<TopoDS_Edge>& vecEdges,
		std::vector<std::vector<TopoDS_Edge>>& vecvecGroups);

	static bool ReadStepFile(Standard_CString strPath);
};


