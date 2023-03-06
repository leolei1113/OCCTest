//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/09
//
//=============================================================================

#pragma once
#include "iogOccInclude.h"

class iogOccKernelTools
{
public:
	iogOccKernelTools();
	~iogOccKernelTools();

	static bool IsShapeGeomSame(const TopoDS_Shape shape1, const TopoDS_Shape shape2,
		const TopAbs_ShapeEnum type);
	static bool shapeFilter(const TopoDS_Shape& origShape, TopAbs_ShapeEnum targetType,
		TopoDS_Shape& filterShape);
	static TopoDS_Shape makeSpiralHelix(double radiusbottom, double radiustop,
		double height, double nbturns,
		double breakperiod, bool leftHanded);
	static bool AIsContainedInB(TopoDS_Shape aShape, TopoDS_Shape bShape);
	static bool IsAIndipendentInTheList(TopoDS_Shape aShape, TopTools_ListOfShape shapeList);
	static bool GetPlanarFaceApexs(TopoDS_Face face, TopTools_ListOfShape& vtxs,
		TopTools_ListOfShape& uselessEdges);
	static bool ReOrgnizeEdgeOrderWire(TopTools_ListOfShape& edges, TopoDS_Vertex vtx,
		std::vector<TopoDS_Shape>& newOrderEdges);
	static bool GroupEdgesInWire(std::vector<TopoDS_Shape>& edges,
		std::vector<std::vector<TopoDS_Shape>>& groups);
	static bool ReOrderEdgesInWire(std::vector<std::vector<TopoDS_Shape>>& groups, TopoDS_Wire &wire);
	static bool STPSeperateBodies(TopoDS_Shape input, TopoDS_Compound& structuredRegroupShape);
	static bool GetIndependentFeatureFacesAndStore(TopoDS_Shape shape, TopTools_ListOfShape& facepackage,
		std::vector<std::vector<TopoDS_Face>>& independentbodyfaces);
	static bool LoopOperate(TopoDS_Shape shape, std::vector<TopoDS_Face>& featurefaces,
		TopTools_ListOfShape& facepackage);
	static bool FaceNormal(const TopoDS_Face face, const double& u,
		const double& v, gp_Dir& dir);
	static Handle(Geom_Surface) GetBasicSurface(Handle(Geom_Surface) surf, GeomAbs_SurfaceType& type);

	static bool GroupEdgesInTriangles(double dPointEqual, std::vector<TopoDS_Edge>& vecEdges,
		std::vector<std::vector<TopoDS_Edge>>& vecvecGroups);
	static bool GroupEdgesInTrianglestest(std::vector<TopoDS_Edge>& vecEdges,
		int& stop);

	static bool GroupEdgesInCompLoop(TopoDS_Edge edge, std::vector<int>& vecUsedIndex,
		std::vector<TopoDS_Edge>& vecEdges, std::vector<TopoDS_Edge>& vecXGroup);

	static bool GroupEdgesInComp(std::vector<TopoDS_Edge>& vecEdges,
		std::vector<std::vector<TopoDS_Edge>>& vecvecGroups);

	static bool GetShapeNameMap(Handle(XCAFDoc_ShapeTool)& assembly,
		TDF_Label& freeshape, std::vector<std::pair<TopoDS_Shape, std::string>>& vecnamemap);

	static bool GetShapeNameMap(TopoDS_Shape& inputShape, TopTools_ListOfShape& shapeList);

	static bool IsFaceSameRole(const TopoDS_Shape shape1, const TopoDS_Shape shape2);

	static bool GetDocShape(Handle(TDocStd_Document) doc, TopoDS_Shape& resultshape);

	static bool LoopBuildDocShape(TDF_Label mainLabel, TopoDS_Shape& resultShape);

	static TopoDS_Shape ReBuildShape(int nAllFaces, int& nCount, TopoDS_Shape resultShape, std::vector<std::string> vecUniqName,
		std::vector<std::pair<TopoDS_Shape, std::string>> vecCSNameMap,
		std::vector<std::pair<TopoDS_Shape, std::string>> vecFaceNameMap,
		QHash<QString, std::string> hashFace,
		std::vector<std::pair<TopoDS_Shape, std::string>>& vecReCSNameMap,
		std::vector<std::pair<TopoDS_Shape, std::string>>& vecReFaceNameMap,
		QHash<QString, int>& hashUsedFace);

	static bool IfModelRealSolid(TopoDS_Shape igesshape, bool& bIfSolid);

};

