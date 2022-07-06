#pragma once
#include <TopoDS_Shape.hxx>
#include <Geom_Ellipse.hxx>
#include <GCPnts_QuasiUniformAbscissa.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <BRepAdaptor_CompCurve.hxx>
#include <vector>
#include <string>
#include <map>
#include <list>
#include <windows.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include "OCCBasicTools.h"
#include <ShapeFix_ShapeTolerance.hxx>
#include <math.h>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>
#include <Extrema_ExtCC.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepAlgo_Section.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BOPAlgo_BOP.hxx>
#include <BOPAlgo_PaveFiller.hxx>
#include <BOPAlgo_CellsBuilder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <ShapeFix_Wireframe.hxx>
#include <GProp_PEquation.hxx>
#include <BOPAlgo_Section.hxx>
#include <BOPTools_AlgoTools.hxx>
#include <IntTools_Context.hxx>
#include <string>
#include <iostream>
#include <stdio.h>

#include <BRepExtrema_ExtCC.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_Viewer.hxx>
#include <V3d_View.hxx>
#include <WNT_Window.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_Handle.hxx>
#include <Graphic3d_StructureManager.hxx>
#include <Prs3d_Root.hxx>
#include <Graphic3d_ArrayOfPoints.hxx>
#include <Graphic3d_Text.hxx>
#include <GeomAPI_ExtremaCurveSurface.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <ShapeFix_Shape.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepLProp_SLProps.hxx>
#include <TDF_Label.hxx>
#include <TNaming_NamedShape.hxx>

#include <QWindow>
#include <QGLWidget>

#include <BRepBndLib.hxx>
#include <BRepMesh_FastDiscret.hxx>

using namespace std;

enum SLIVERTOLTYPE
{
	scaleFactor,
	edgeLength
};

enum class AssembleType { coincidence, align, orient };

struct HealOption
{
	int healingtype{ 1 };
	bool perfmTolStich;
	double stitchTolerance;
	bool stopAfterFirstErr;
	bool isSimplify{ false };
	double simplifyTolerance;
	double MaxGenRadius;
	int simplifyType;
	bool isTightenGaps;
	double maxTightenGaps;
	bool isRmvSmallEdges;
	bool isRmvSmallFaces;
	bool isRmvSliverFaces;
	double smallEdgeTol;
	double smallFaceTols;
	SLIVERTOLTYPE sliverTolType;
	bool UseScalFactor;
	bool UseEdgeWidth;
	double SliverFacTol;
};

struct Transforminfo
{
	bool istranslate;
	double tanslatedistance;
	double transformirection[3];
	bool isrotate;
	double rotateangleinrad;
	double transformbase[3];
	bool isscale;
	double scalevalue;
	bool ismirror;
};


class OCCTest
{
public:
	static bool MakeDraft(TopoDS_Shape& shape);
	static void ConvertEllipse2Polygon(gp_Elips& elips, int n, TopoDS_Wire& my_wire);
	static TopoDS_Face GetFaceFromPntList(std::vector<gp_Pnt> PntsList);
	static bool OffSet(TopoDS_Shape& shape, double value, bool if_filleted);
	static bool fillet(TopoDS_Shape& shape);
	static bool moveEdges(std::vector<gp_Pnt>& point_list, std::vector<TopoDS_Edge> edges, double distance, TopoDS_Face& outputshape, std::vector<gp_Pnt>& substitude_pt);
	static int LocateTarget2Num(std::vector<gp_Pnt>, gp_Pnt taget_point);
	static bool createPoint(double coordx, double coordy, double coordz, TopoDS_Shape& output_shape);
	static bool swapSheet(TopoDS_Shape mutter_3d, TopoDS_Shape vatter, TopoDS_Shape& sohn_shape);
	static bool removeFace(TopoDS_Shape mutter_3d, TopTools_ListOfShape remove_list, TopoDS_Shape& sohn_shape);
	static bool removeEdge(std::vector<TopoDS_Shape> remove_list, TopoDS_Shape& sohn_shape);
	static bool healing(TopoDS_Shape& shape, HealOption parameter);
	static bool imprintEdge(TopTools_ListOfShape total, TopoDS_Shape& shape);
	static bool ExportModel(TopoDS_Shape shape, const std::string& path);
	static bool DeleteEdges(const TopTools_IndexedMapOfShape& edges, const bool edgeOnly, TopoDS_Shape& shape);
	static bool DetachFaces(const TopTools_IndexedMapOfShape& faces, TopoDS_Shape& shape);
	static bool SmallEdges(TopoDS_Shape& shape, const double len, const double ang_deg);
	static bool IfTopoShapeExist(std::vector<TopoDS_Shape> list, TopoDS_Shape me);
	static bool TurnFace2Plane(TopoDS_Face face, gp_Pln& plane);
	static bool RayTest(TopoDS_Shape shape, gp_Pnt pos, gp_Dir dir, bool is_single, TopTools_ListOfShape& hits);
	static bool CheckOneLine(std::vector<TopoDS_Edge> anEdges);
	static bool MakeAnnotation(std::vector<gp_Pnt> pnts, std::string text);
	static void WriteLog(char* log, int ner);
	static bool DetachFaces(TopoDS_Shape ent, std::vector<TopoDS_Face> facelist, bool iscreated, TopoDS_Shape& output);
	static bool OffsetWire(TopoDS_Shape shape, double distance, TopoDS_Shape& offsetshape);
	static bool Connect(TopTools_ListOfShape shapelist, bool gensolid, TopoDS_Shape& connectshape);
	static bool CreatePolyLine(std::vector<std::vector<double>>& points, TopoDS_Shape& outputshape);
	static bool ShapeFindFilterSub(TopoDS_Shape& orishape, TopAbs_ShapeEnum targettype, TopoDS_Shape& filtershape);
	static bool AutoFillGapFaces(TopoDS_Shape& orishape, double allowedarea);
	static bool IsShapeGeomSame(const TopoDS_Shape shape1, const TopoDS_Shape shape2, const TopAbs_ShapeEnum type);
	static bool LoopFindAdjacentEdge(TopTools_ListOfShape usedshapes, TopoDS_Shape startedge, std::vector<TopoDS_Shape> edgeset,
		std::vector<TopoDS_Shape> neededges);
	static bool shapeFilter(const TopoDS_Shape& origSHape, TopAbs_ShapeEnum targetType, TopoDS_Shape& filterShape);
	static bool Assembly(TopoDS_Shape toolshape, TopoDS_Shape blankshape, AssembleType type,
		TopoDS_Shape& targetshape, NCollection_Mat4<double> matrix);
	static bool IsEdgePlanar(Handle(Geom_Curve) curve, double first, double last, gp_Pnt& centerpoint, gp_Dir& normaldir);
	static bool AssemblyPlaneFaceFace(TopoDS_Shape toolshape, BRepAdaptor_Surface bas,
		Handle(Geom_Surface) blankgeomsurface, TopoDS_Shape& targetshape, 
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner);
	static bool AssemblyPlaneFaceEdge(TopoDS_Shape toolshape, BRepAdaptor_Surface bas, Handle(Geom_Surface) blankgeomsurface,
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape);
	static bool AssemblyPlaneFaceVertex(BRepAdaptor_Surface bas, TopoDS_Shape toolshape, NCollection_Mat4<double> matrix,
		TopoDS_Shape& targetshape, TopoDS_Shape toolshapeowner, Handle(Geom_Surface) blankgeomsurface, AssembleType type);
	static bool AssemblyCylinderFaceFace(TopoDS_Shape toolshape, BRepAdaptor_Surface bas
		, TopoDS_Shape& targetshape, AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner);
	static bool AssemblyCylinderFaceEdge(TopoDS_Shape toolshape, BRepAdaptor_Surface bas,
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape);
	static bool AssemblyCylinderFaceVertex(BRepAdaptor_Surface bas, TopoDS_Shape toolshape, NCollection_Mat4<double> matrix,
		TopoDS_Shape& targetshape, TopoDS_Shape toolshapeowner, AssembleType type);
	static bool AssemblySphereFaceFace(TopoDS_Shape toolshape, BRepAdaptor_Surface bas
		, TopoDS_Shape& targetshape, AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner);
	static bool AssemblySphereFaceEdge(TopoDS_Shape toolshape, BRepAdaptor_Surface bas,
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape);
	static bool AssemblyLineFace(TopoDS_Shape toolshape, BRepAdaptor_Curve bas
		, TopoDS_Shape& targetshape, AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner);
	static bool AssemblyLineEdge(TopoDS_Shape toolshape, BRepAdaptor_Curve bas,
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape);
	static bool AssemblyPlanarLineFace(TopoDS_Shape toolshape, TopoDS_Edge blankshape, TopoDS_Shape& targetshape,
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner);
	static bool AssemblyPlanarLineEdge(TopoDS_Shape toolshape, TopoDS_Edge blankedge,
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape);
	static bool AssemblyVtxEdge(TopoDS_Shape toolshape, TopoDS_Shape blankedge,
		AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape);
	static bool AssemblyVtxFace(TopoDS_Shape toolshape, TopoDS_Shape blankshape, TopoDS_Shape& targetshape, AssembleType type
		, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner);

	static bool FaceSplitSolidShell(TopoDS_Shape inputbody, std::vector<TopoDS_Face> faces, TopoDS_Shape& outputshape);
	static bool FaceSplitBody(TopoDS_Shape inputbody, std::vector<TopoDS_Face> faces, TopoDS_Compound& outputshape);

	static bool Transform(Transforminfo transinfo, NCollection_Mat4<double> inputmatrix, NCollection_Mat4<double>& outputmatrix);
	static bool Transform(Transforminfo transinfo, TopoDS_Shape& operateshape);

	static bool CalculateSpecificPoints(TopoDS_Shape inputbody, std::vector<gp_Pnt> points);

	static bool CompondSolidTest(TopoDS_Shape shape1, TopoDS_Shape shape2, TopoDS_Shape& res);

	static bool MakeSolidTest(TopoDS_Solid shape1, TopoDS_Shell shape2, TopoDS_Shape& res);

	static bool FindTypeLevelFatherLabel(TDF_Label& inputlabel, TopAbs_ShapeEnum type, TopoDS_Shape& currentshape);
	
	static TopoDS_Shape getLabelShape(TDF_Label label);
private:
	
};

