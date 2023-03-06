//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/09
//
//=============================================================================

#pragma once
#include "iogModule.h"
#include "string.h"
#include "qui.h"
#include <QtCore/qlocale>
#include <unordered_set>

#include "iogOccInclude.h"
#include "iogBinaryTree.h"
class comAssemblyMgr;
class comPartMgr;
class comSurfaceMgr;
class comActorMgr;
class comFaceMgr;
class comVerticeMgr;

class iogBasicTools
{
public:
	iogBasicTools();
	~iogBasicTools();

	static void GetPartName(char* pName, char* pFilePath);
	static bool DiscretizeShape(double dLinearDelVal, double dAngleDelVal, int nAllFaces, bool firstloop, UINT uMaxSurfIx, double dScale, std::vector<string> vecUniqName,
		std::vector<std::pair<TopoDS_Shape, std::string>> vecnamemap, std::vector<std::pair<TopoDS_Shape, std::string>> vecFaceNameMap,
		char* pFilePath, void* pEntryShape, QHash<QString, UINT>& hashVertIdx);
	static QString RePartName(QString & strName);
	static bool DiscretizeLoopProcess(int nAllFaces, int& nCurrentFace, int& nBeginVtx, 
		IMeshTools_Parameters meshParas, bool firstloop, UINT uMaxSurfIx, std::vector<string> vecUniqName,
		std::vector<std::pair<TopoDS_Shape, std::string>> vecnamemap, std::vector<std::pair<TopoDS_Shape, std::string>> vecFaceNameMap,
		UINT nVert, int nCurrentAssIndex, int nShadingType,
		 TopoDS_Shape mainshape, QHash<QString, UINT>& hashVertIdx);
	static bool Heal(TopoDS_Shape &entryShape, double dtolerance, double dholeRad, double dfilletRad);
	static bool AutoFillInnerGaps(TopoDS_Shape &entryShape, double tolerance);
	static bool LoopFindAdjacentEdge(TopTools_ListOfShape& usedshapes, TopoDS_Shape& startedge,
		TopTools_ListOfShape& edgeset, std::vector<TopoDS_Shape>& vecneededges,
		TopTools_ListOfShape& xedgegroup);
	static bool AutoFillHoles(TopoDS_Shape &entryShape, double maxRadius);
	static bool AutoRemoveFreeEdges(TopoDS_Shape &entryShape, double tolerance);
	static bool AutoRemoveFillets(TopoDS_Shape &entryShape, double dmaxRad);
	static bool AutoMergeEdges(TopoDS_Shape &entryShape);

	static bool ClearFreeEdges(bool bIfSTP, double scale, bool firstloop, UINT uMaxFaceIx, TopoDS_Shape entryshape,
		UINT maxsurfaceidx, QHash<QString, UINT> hashVertIdx);
	static bool AddPntToVec(gp_Pnt pt1, std::vector<gp_Pnt> &vecPnt);
	static gp_Dir DirOf2Pnts(gp_Pnt pt1, gp_Pnt pt2);

	static bool SplitGroupTriangles(double dPointEqual, std::vector<TopoDS_Edge>& vecEdges,
		std::vector<std::vector<TopoDS_Edge>>& vecvecGroups);
	static bool FindMergePairs(TopoDS_Shape entryshape,
		std::vector<TopoDS_Edge> vecEdges, std::vector<std::pair<UINT, UINT>>& vecnearestpt);

	static bool FilterFreeEdgeVtxIndexes(
		double dPointEqual,
		std::vector<std::pair<UINT, gp_Pnt>> vecPairVtxIndex,
		std::vector<TopoDS_Edge> vecFreeEdge,
		std::vector<std::pair<UINT, UINT>>& vecIdxPair);

	static bool FindTriangleIndex(
		std::vector<std::pair<UINT, UINT>> vecIdxPair,
		std::vector<std::vector<UINT>>& tris);

	static bool GridScale(bool firstloop, UINT nMaxPartIx, UINT nMaxFaceIx, UINT uMaxSurfIx, double dScale);
	static QString ReNamePart(QString & strPartName);
	static QString ReNameSurf(QString & strSurfName);
	static QString ReNameAssembly(QString & strAssemblyName);
	static bool Pnt2Inx(gp_Pnt pnt, QHash<QString, UINT>& hashVertIdx, comVerticeMgr * pVertMgr,
		int& nBeginVtx, UINT nVert, UINT& uVerts);
	
	static bool GetClosedVertexIdxGroup(vector<vector<pair<basePoint, basePoint>>> inputEdges,
		QHash<QString, UINT> hashVertIdx,
		vector<vector<UINT>>& groupedIndex);

	static bool LoopFindClosedFreeEdges(vector<UINT> &subgroup, vector<UINT>& usedgroup, vector<UINT> allIndx, UINT beginIndx,
		QHash<UINT, UINT> hashEdge);
	static bool LoopClosedFreeEdges(UINT& startIdx,
		vector<pair<UINT, UINT>> vecEdgeIndx
		, vector<UINT>& subgroup,
		vector<int>& vecUsedIdx);
	static bool LoopClosedFreeEdges(UINT& startIdx,
		QMultiHash<UINT, UINT> hashEdgeIndx,
		vector<UINT>& subgroup);

	static double CalculateAngleBetweenDirs(gp_Dir dir1, gp_Dir dir2);
	static bool ReplaceVertex(vector<UINT> vecFaceIx, vector<UINT> vecFirstList, vector<UINT> vecSecondList);
	static bool FixTrigangle(vector<UINT> currentClosed);
	static bool OffsetSelectedSurface(double dDistance, vector<UINT> vecSelectedSurface);

	static void GetVertexByFaces(const vector<UINT> & vecFaceIx, std::vector<UINT>  & vecVertex);
	static void GetVertexBySurfaces(const vector<UINT> & vecSurfaceIx, std::vector<UINT>  & vecVertex);

	static vector<vector<UINT>> FindConnectedEdges(int nNum, vector<pair<UINT, UINT>>& edges);
	static void dfs(int node, const vector<vector<UINT>>& graph, vector<bool>& visited, vector<UINT>& component);

	static bool TurnClosedEdgePairToOrderedIndex(vector<pair<UINT, UINT>> vecEdgeIndx,
		vector<UINT>& subgroup);
};

