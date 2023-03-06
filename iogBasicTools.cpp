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
#include "iogBasicTools.h"
#include "iogOccKernelTools.h"
#include <math.h>

#include "algorithm.h"

#define Mesh_Linear_DefExRough 0.0016
#define Mesh_Linear_DefRough 0.0008
#define Mesh_Linear_DefMedium 0.0004
#define Mesh_Linear_DefFine 0.0002
#define Mesh_Linear_DefExFine 0.0001

#define Mesh_Angle_DefExRough 0.8
#define Mesh_Angle_DefRough 0.4
#define Mesh_Angle_DefMedium 0.2
#define Mesh_Angle_DefFine 0.1
#define Mesh_Angle_DefExFine 0.05 

#define nThree 3
#define mergeTolerance 10                             //sqrt3 relation
#define pointEqualTol 0.001 
#define parallTolerance 1E-20
#define angleTolerance 20 

#define Minor_Modify 2
#define Major_Modify 2

int nTrans;
double dOpacity;

//////////////////////////////////////////////////////////////////
//

iogBasicTools::iogBasicTools()
{
}


iogBasicTools::~iogBasicTools()
{
}

//////////////////////////////////////////////////////////////////
//

//void iogBasicTools::GetPartName(char * pName, char* pFilePath)
//{
//	if (!pName)
//	{
//		return;
//	}
//
//	char pFilename[1024];
//	for (int i = 0; i < strlen(pFilePath); i++)
//	{
//		if (pFilePath[i] != '.')
//		{
//			pFilename[i] = pFilePath[i];
//		}
//		else
//		{
//			pFilename[i] = '\0';
//			break;
//		}
//	}
//
//	char * pStr = strrchr(pFilename, '/') + 1;
//	if (!pStr)
//	{
//		return;
//	}
//	for (int i = 0; i < strlen(pStr); i++)
//	{
//		pName[i] = pStr[i];
//	}
//	pName[strlen(pStr)] = '\0';
//}

void iogBasicTools::GetPartName(char * pName, char* pFilePath)
{
	if (!pName)
	{
		return;
	}

	QString qstrFilePath = QString::fromLocal8Bit(pFilePath);
	int nIndexOfLast = qstrFilePath.lastIndexOf(".");
	int nIndexOfBegin = qstrFilePath.lastIndexOf("/");
	int nNameLength = nIndexOfLast - nIndexOfBegin;
	QString qstrMid = qstrFilePath.mid(nIndexOfBegin + 1, nNameLength - 1);

	QByteArray ba = qstrMid.toLocal8Bit();
	char* pStr = ba.data();

	for (int i = 0; i < strlen(pStr); i++)
	{
		pName[i] = pStr[i];
	}
	pName[strlen(pStr)] = '\0';
}

QString iogBasicTools::RePartName(QString & strName)
{
	comPartMgr * pPartMgr = comPart::Manager();
	if (!pPartMgr)
	{
		return QString();
	}

	vector<UINT> vecParts;
	pPartMgr->GetAllObject(vecParts);
	size_t tParts = vecParts.size();
	QList<QString> lstPartName;

	if (tParts == 0)
	{
		return strName;
	}
	else if (tParts > 0)
	{
		for (size_t i = 0; i < tParts; i++)
		{
			UINT uPart = vecParts[i];
			comPart * pPart = (comPart *)(pPartMgr->Object(uPart));
			if (!pPart)
			{
				return QString();
			}

			const char * pName = pPart->Name();
			if (!pName)
			{
				return QString();
			}

			QString strPart(pName);
			bool bStart = strPart.startsWith(strName);
			if (bStart)
			{
				lstPartName.append(pName);
			}
		}
	}

	int nStart = lstPartName.size();
	QList<UINT> lstPartNum;
	QString strMid = QObject::tr("-");
	bool bOriIn = lstPartName.contains(strName);
	if (nStart == 0 || !bOriIn)
	{
		return strName;
	}
	else if (nStart > 0)
	{
		for (int i = 0; i < nStart; i++)
		{
			QString strPartName = lstPartName[i];
			QStringList slstPartName = strPartName.split(strName);
			int nNameLen = slstPartName.size();
			if (nNameLen == 2)
			{
				QString strSec = slstPartName[nNameLen - 1];
				bool bStartMid = strSec.startsWith(strMid);
				if (bStartMid)
				{
					QStringList slstSec = strSec.split(strMid);
					if (slstSec.size() == 2)
					{
						QString strSecSec = slstSec[nNameLen - 1];
						bool bTrans = false;
						UINT uNum = strSecSec.toUInt(&bTrans);
						if (bTrans)
						{
							int nAllNum = lstPartNum.size();
							if (nAllNum == 0)
							{
								lstPartNum.append(uNum);
							}
							else if (nAllNum > 0)
							{
								bool bIn = lstPartNum.contains(uNum);
								if (!bIn)
								{
									lstPartNum.append(uNum);
								}
							}
						}
					}
				}
			}
		}
	}

	UINT uPartIdx;
	int nAll = lstPartNum.size();
	for (int i = 1; i < nAll + 2; i++)
	{
		bool bIn = lstPartNum.contains(i);
		if (!bIn)
		{
			uPartIdx = i;
			break;
		}
	}

	QString strRet = strName + strMid + QString::number(uPartIdx);
	return strRet;
}

bool iogBasicTools::DiscretizeShape(double dLinearDelVal, double dAngleDelVal, int nAllFaces, bool firstloop, UINT uMaxSurfIx, double dScale, std::vector<string> vecUniqName,
	std::vector<std::pair<TopoDS_Shape, std::string>> vecnamemap, std::vector<std::pair<TopoDS_Shape, std::string>> vecFaceNameMap,
	char* filepath, void* entryShape, QHash<QString, UINT>& hashVertIdx)
{
	if (!entryShape)
	{
		return false;
	}
	TopoDS_HShape* pHShape = static_cast<TopoDS_HShape*>(entryShape);
	if (!pHShape)
	{
		return false;
	}
	TopoDS_Shape mainshape = pHShape->Shape();

	comAssemblyMgr * pAssMgr = comAssembly::Manager();
	comPartMgr * pPartMgr = comPart::Manager();
	comSurfaceMgr * pSurfMgr = comSurface::Manager();
	comActorMgr * pActorMgr = comActor::Manager();
	int nAssIndex = 0, nPrtIndex = 0, nSurfIndex = 0;
	if (!pAssMgr || !pPartMgr || !pSurfMgr || !pActorMgr)
	{
		return false;
	}
	comViewMgr * pViewMgr = comView::Manager();
	UINT uViewIdx = pViewMgr->CurrentView();
	comView * pView = (comView *)(pViewMgr->Object(uViewIdx));
	if (!pView)
	{
		return false;
	}
	int nShadingType = pView->ViewType();

	dOpacity = -1;
	bool bTrans = false;
	nTrans = pView->Transparent();
	if (nTrans == 1)
	{
		bTrans = true;
		dOpacity = ConfigGetOpacityFactor();
	}

	char pName[128];
	GetPartName(pName, filepath);
	QString strTempName(pName);
	QString strPartName = RePartName(strTempName);
	QByteArray barPartName = strPartName.toLocal8Bit();
	char * pPartName = barPartName.data();
	if (!pPartName)
	{
		return false;
	}
	int nthisAssIndex = 0;
	if (mainshape.ShapeType() == TopAbs_COMPOUND)
	{
		std::string strAssName = pPartName;
		QString qstrAss = QString::fromStdString(strAssName);
		QString qstrAssTemp = ReNameAssembly(qstrAss);
		strAssName = qstrAssTemp.toStdString();

		comAssembly * pAss = new comAssembly(strAssName.c_str());
		if (!pAss)
		{
			return false;
		}
		pAssMgr->Create(pAss);
		nthisAssIndex = pAssMgr->MaxIndex();
		nAssIndex++;
	}
	else
	{
		return false;
	}

	//bounding box always in milimeter
	//Bnd_Box box;
	//BRepBndLib::Add(mainshape, box, Standard_True);

	Message_ProgressRange messProgress;
	IMeshTools_Parameters meshParas;

	meshParas.ControlSurfaceDeflection = false;
	meshParas.Angle = dAngleDelVal;
	meshParas.Deflection = dLinearDelVal;
	meshParas.InParallel = true;
	meshParas.Relative = true;
	meshParas.MinSize = Precision::Confusion();
	//BRepMesh_IncrementalMesh incrementalmesh(mainshape, meshParas, messProgress);
	BRepMesh_IncrementalMesh incrementalmesh(mainshape, dLinearDelVal, true, dAngleDelVal, true);

	comFaceMgr * pFaceMgr = comFace::Manager();
	comVerticeMgr * pVertMgr = comVertice::Manager();
	if (!pFaceMgr || !pVertMgr)
	{
		return false;
	}
	UINT nVert = pVertMgr->MaxIndex();
	
	int nBeginFace = 0;
	int nBeginVtx = 0;
	//QHash<QString, UINT> hashVertIdx;

	UINT nMaxPartIx = pPartMgr->MaxIndex();
	UINT nMaxFaceIx = pFaceMgr->MaxIndex();

	if (!DiscretizeLoopProcess(nAllFaces, nBeginFace, nBeginVtx, 
		meshParas, firstloop, uMaxSurfIx, vecUniqName, vecnamemap, vecFaceNameMap, nVert, nthisAssIndex,
		nShadingType, mainshape, hashVertIdx))
	{
		return false;
	}

	TheExe.ProgressSetValue(100);

	if(dScale!=1)
		GridScale(firstloop, nMaxPartIx, nMaxFaceIx, uMaxSurfIx, dScale);

	vector<UINT> vecSurfIx, vecUsedSurfIx;
	pSurfMgr->GetAllObject(vecSurfIx);
	for (UINT i = 0; i < vecSurfIx.size(); i++)
	{
		if (!firstloop)
		{
			if (i < uMaxSurfIx + 1)
			{
				continue;
			}
		}

		vecUsedSurfIx.push_back(i);
	}

	/*algNormalTopology * pNormalTopo = new algNormalTopology();
	if (!pNormalTopo)
	{
		return true;
	}
	bool bSurfTopo = pNormalTopo->CreateTopoBySurfs(vecUsedSurfIx);
	if (!bSurfTopo)
	{
		return true;
	}
	UINT uAdjust = pNormalTopo->AdjustNormalBySurfs(true, vecUsedSurfIx);*/

	return true;
}

bool iogBasicTools::DiscretizeLoopProcess(int nAllFaces, int& nCurrentFace, int& nBeginVtx, 
	IMeshTools_Parameters meshParas, bool firstloop, UINT uMaxSurfIx, std::vector<string> vecUniqName,
	std::vector<std::pair<TopoDS_Shape, std::string>> vecnamemap, std::vector<std::pair<TopoDS_Shape, std::string>> vecFaceNameMap,
	UINT nVert, int nCurrentAssIndex, int nShadingType,
	TopoDS_Shape mainshape, QHash<QString, UINT>& hashVertIdx)
{
	comAssemblyMgr * pAssMgr = comAssembly::Manager();
	comPartMgr * pPartMgr = comPart::Manager();
	comSurfaceMgr * pSurfMgr = comSurface::Manager();
	comActorMgr * pActorMgr = comActor::Manager();
	comFaceMgr * pFaceMgr = comFace::Manager();
	comVerticeMgr * pVertMgr = comVertice::Manager();

	try
	{
		TopoDS_Iterator subiter(mainshape);
		for (; subiter.More(); subiter.Next())
		{
			TopoDS_Shape currentShape = subiter.Value();
			if (currentShape.ShapeType() == TopAbs_SHELL)
			{	
				//empty part
				std::vector<UINT> vecParts;
				TheFunc.GetAssemblyParts(nCurrentAssIndex, vecParts);
				if (vecParts.size() < 1)
				{
					string strNum = std::to_string(pPartMgr->MaxIndex() + 2);
					string strPrtName = "Part" + strNum;
					comPart * pPrt = new comPart(strPrtName.c_str());
					if (!pPrt)
					{
						return false;
					}
					pPartMgr->Create(pPrt);
					pPrt->SetAssembly(nCurrentAssIndex);
					pPrt->SetColor(ConfigNextColor());
				}
				//set current index
				comSurface * pSurface = NULL;
				UINT npartIndex = pPartMgr->MaxIndex();
				//surface name
				string strSurfaceName;
				for (int i = 0; i < vecFaceNameMap.size(); i++)
				{
					std::pair<TopoDS_Shape, string> currentpair = vecFaceNameMap[i];
					TopoDS_Shape currentpairShell = currentpair.first;
					if (currentpairShell.ShapeType() != TopAbs_SHELL)
					{
						continue;
					}

					if (currentShape.IsSame(currentpairShell))
					{
						strSurfaceName = currentpair.second;
						break;
					}
				}
				/*if (strSurfaceName == "")
				{
					strSurfaceName = std::to_string(pSurfMgr->MaxIndex() + 2);
				}*/
				QString qstrSurf = QString::fromStdString(strSurfaceName);
				qstrSurf = ReNameSurf(qstrSurf);
				strSurfaceName = qstrSurf.toLocal8Bit().data();
				pSurface = new comSurface((char*)strSurfaceName.c_str());
				if (!pSurface)
				{
					continue;
				}
				pSurfMgr->Create(pSurface);
				pSurface->SetPart(npartIndex);
				pSurface->SetColor(ConfigNextColor());

				//get all faces, get facet
				std::vector<TopoDS_Shape> vecFacelist;
				TopExp_Explorer faceExp(currentShape, TopAbs_FACE);
				for (; faceExp.More(); faceExp.Next())
				{
					vecFacelist.push_back(faceExp.Current());
				}

				for(int i = 0; i < vecFacelist.size();i++)
				{
					nCurrentFace++;
					TheExe.ProgressSetValue(100 * nCurrentFace / nAllFaces);

					TopLoc_Location topLocation;
					TopoDS_Face currentface = TopoDS::Face(vecFacelist[i]);

					//triangle face
					Handle(Poly_Triangulation) currentfacetri = BRep_Tool::Triangulation(currentface, topLocation);
					if (!currentfacetri)
					{
						continue;
					}
					gp_Trsf trsfInfo = topLocation.Transformation();

					int nTotalNode = currentfacetri->NbNodes();
					int nTotalTri = currentfacetri->NbTriangles();
					for (int j = 1; j <= nTotalTri; j++)
					{
						std::vector<UINT> vecCurrentNodeIdx;
						std::vector<double> vecTriNodePosition;
						std::vector<double> vecTriNormal;
						Poly_Triangle currenttri = currentfacetri->Triangle(j);
						int nTriNode1 = currenttri.Value(1);
						int nTriNode2 = currenttri.Value(2);
						int nTriNode3 = currenttri.Value(3);
						
						gp_Pnt pntNode1 = currentfacetri->Node(nTriNode1).Transformed(trsfInfo);
						gp_Pnt pntNode2 = currentfacetri->Node(nTriNode2).Transformed(trsfInfo);
						gp_Pnt pntNode3 = currentfacetri->Node(nTriNode3).Transformed(trsfInfo);

						//avoid "gp_Dir::Crossed() - result vector has zero norm"
						if (pntNode1.X() == pntNode2.X() &&
							pntNode1.Y() == pntNode2.Y() &&
							pntNode1.Z() == pntNode2.Z())
						{
							continue;
						}
						gp_Dir calculateNormalAssistDir1(pntNode1.X() - pntNode2.X(), pntNode1.Y() - pntNode2.Y(),
							pntNode1.Z() - pntNode2.Z());

						if (pntNode1.X() == pntNode3.X() && pntNode1.Y() == pntNode3.Y() &&
							pntNode1.Z() == pntNode3.Z())
						{
							continue;
						}
						gp_Dir calculateNormalAssistDir2(pntNode1.X() - pntNode3.X(), pntNode1.Y() - pntNode3.Y(),
							pntNode1.Z() - pntNode3.Z());

						bool isParall2AssistDir = false;
						if (calculateNormalAssistDir1.IsParallel(calculateNormalAssistDir2, parallTolerance))
						{
							isParall2AssistDir = true;
						}
						if (isParall2AssistDir)
						{
							continue;
						}

						//bool bIfHasNormal = currentfacetri->HasNormals();
						//gp_Dir triNomalTest = currentfacetri->Normal(i);

						gp_Dir triNormal = calculateNormalAssistDir1.Crossed(calculateNormalAssistDir2);
						double dNormal[3] = { triNormal.X(), triNormal.Y(), triNormal.Z() };
						//double dNormal[3] = { 0, 0, 1 };

						UINT uVerts[nThree];

						if (!Pnt2Inx(pntNode1, hashVertIdx, pVertMgr, nBeginVtx, nVert, uVerts[0]))
						{
							continue;
						}
						if (!Pnt2Inx(pntNode2, hashVertIdx, pVertMgr, nBeginVtx, nVert, uVerts[1]))
						{
							continue;
						}
						if (!Pnt2Inx(pntNode3, hashVertIdx, pVertMgr, nBeginVtx, nVert, uVerts[2]))
						{
							continue;
						}

						if (uVerts[0] == uVerts[1] || uVerts[0] == uVerts[2] || uVerts[1] == uVerts[2])
						{
							continue;
						}
						
						comFace * pFace = new comFace(uVerts);
						if (!pFace)
						{
							return false;
						}
						pFace->SetNormal(dNormal);
						pFaceMgr->Create(pFace);
						pFace->SetSurface(pSurfMgr->MaxIndex());
					}

					
				}
				comActor * pActor = new comActor(comActor::Act_Geom);
				if (!pActor)
				{
					return false;
				}
				pActorMgr->Create(pActor);
				pActor->SetEntity(pSurfMgr->MaxIndex());
				pActor->SetEntityType(comActor::Ent_Surface);
				pActor->SetDisplay(nShadingType);
				if (nTrans == 1)
				{
					pActor->SetOpacity(dOpacity);
				}
			}
			else if(currentShape.ShapeType() < TopAbs_FACE)
			{
				if (currentShape.ShapeType() == TopAbs_COMPOUND)
				{
					string strAssName;
					for (int i = 0; i < vecnamemap.size(); i++)
					{
						std::pair<TopoDS_Shape, string> currentpair = vecnamemap[i];
						if (currentShape.IsSame(currentpair.first))
						{
							strAssName = currentpair.second;
						}
					}
					if (strAssName == "")
					{
						string strNum = std::to_string(pAssMgr->MaxIndex() + 1);
						strAssName = "Assembly" + strNum;
					}

					QString qstrAss = QString::fromStdString(strAssName);
					QString qstrAssTemp = ReNameAssembly(qstrAss);
					strAssName = qstrAssTemp.toLocal8Bit().data();

					comAssembly * pAss = new comAssembly(strAssName.c_str());
					
					if (!pAss)
					{
						return false;
					}
					pAssMgr->Create(pAss);
					pAss->SetParent(nCurrentAssIndex);
					int nthisAssIndex = pAssMgr->MaxIndex();
					DiscretizeLoopProcess(nAllFaces, nCurrentFace, nBeginVtx, meshParas, firstloop, uMaxSurfIx, vecUniqName,
						vecnamemap, vecFaceNameMap, nVert, nthisAssIndex,
						 nShadingType, currentShape, hashVertIdx);
				}
				else if (currentShape.ShapeType() == TopAbs_SOLID)
				{
					string strPrtName;
					for (int i = 0; i < vecnamemap.size(); i++)
					{
						std::pair<TopoDS_Shape, string> currentpair = vecnamemap[i];
						if (currentShape.IsSame(currentpair.first))
						{
							strPrtName = currentpair.second;
						}
					}
					if (strPrtName == "")
					{
						string strNum = std::to_string(pPartMgr->MaxIndex() + 2);
						strPrtName = "Part" + strNum;
					}

					QString qstrPart = QString::fromStdString(strPrtName);
					QString qstrPartTemp = ReNamePart(qstrPart);
					strPrtName = qstrPartTemp.toLocal8Bit().data();

					comPart * pPrt = new comPart(strPrtName.c_str());
					if (!pPrt)
					{
						return false;
					}
					pPartMgr->Create(pPrt);
					pPrt->SetAssembly(nCurrentAssIndex);
					pPrt->SetColor(ConfigNextColor());

					int nthisAssIndex = pAssMgr->MaxIndex();
					DiscretizeLoopProcess(nAllFaces, nCurrentFace, nBeginVtx, meshParas, firstloop, uMaxSurfIx,
						vecUniqName, vecnamemap, vecFaceNameMap, nVert, nthisAssIndex, nShadingType,
						currentShape, hashVertIdx);
				}
			}
			else
			{
				continue;
			}
		}
		return true;	
	}
	catch (...)
	{
		return false;
	}
}

QString iogBasicTools::ReNameAssembly(QString & strAssemblyName)
{
	comAssemblyMgr * pAssemblyMgr = comAssembly::Manager();
	if (!pAssemblyMgr)
	{
		return strAssemblyName;
	}

	vector<UINT> vecAssemblys;
	pAssemblyMgr->GetAllObject(vecAssemblys);
	size_t tAssemblys = vecAssemblys.size();
	QList<QString> lstAssemblyName;

	if (tAssemblys == 0)
	{
		return strAssemblyName;
	}
	else if (tAssemblys > 0)
	{
		for (size_t i = 0; i < tAssemblys; i++)
		{
			UINT uAssembly = vecAssemblys[i];
			comAssembly * pAssembly = (comAssembly *)(pAssemblyMgr->Object(uAssembly));
			if (!pAssembly)
			{
				return strAssemblyName;
			}

			const char * pName = pAssembly->Name();
			if (!pName)
			{
				return strAssemblyName;
			}

			QString strAssembly = QString::fromLocal8Bit(pName);
			bool bStart = strAssembly.startsWith(strAssemblyName);
			if (bStart)
			{
				lstAssemblyName.append(strAssembly);
			}
		}
	}

	int nStart = lstAssemblyName.size();
	QList<UINT> lstAssemblyNum;
	QString strMid = QObject::tr("-");
	bool bOriIn = lstAssemblyName.contains(strAssemblyName);

	if (nStart == 0 || !bOriIn)
	{
		return strAssemblyName;
	}
	else if (nStart > 0)
	{
		for (int i = 0; i < nStart; i++)
		{
			QString strTemp = lstAssemblyName[i];
			QStringList slstAssemblyName = strTemp.split(strAssemblyName);
			int nNameLen = slstAssemblyName.size();

			if (nNameLen == 2)
			{
				QString strSec = slstAssemblyName[nNameLen - 1];
				bool bStartMid = strSec.startsWith(strMid);
				if (bStartMid)
				{
					QStringList slstSec = strSec.split(strMid);
					if (slstSec.size() == 2)
					{
						QString strSecSec = slstSec[nNameLen - 1];
						bool bTrans = false;
						UINT uNum = strSecSec.toUInt(&bTrans);
						if (bTrans)
						{
							int nAllNum = lstAssemblyNum.size();
							if (nAllNum == 0)
							{
								lstAssemblyNum.append(uNum);
							}
							else if (nAllNum > 0)
							{
								bool bIn = lstAssemblyNum.contains(uNum);
								if (!bIn)
								{
									lstAssemblyNum.append(uNum);
								}
							}
						}
					}
				}
			}
		}
	}

	UINT uAssemblyIdx;
	int nAll = lstAssemblyNum.size();
	for (int i = 1; i < nAll + 2; i++)
	{
		bool bIn = lstAssemblyNum.contains(i);
		if (!bIn)
		{
			uAssemblyIdx = i;
			break;
		}
	}

	QString strRet = strAssemblyName + strMid + QString::number(uAssemblyIdx);
	return strRet;
}

QString iogBasicTools::ReNamePart(QString & strPartName)
{
	comPartMgr * pPartMgr = comPart::Manager();
	if (!pPartMgr)
	{
		return strPartName;
	}

	vector<UINT> vecParts;
	pPartMgr->GetAllObject(vecParts);
	size_t tParts = vecParts.size();
	QList<QString> lstPartName;

	if (tParts == 0)
	{
		return strPartName;
	}
	else if (tParts > 0)
	{
		for (size_t i = 0; i < tParts; i++)
		{
			UINT uPart = vecParts[i];
			comPart * pPart = (comPart *)(pPartMgr->Object(uPart));
			if (!pPart)
			{
				return strPartName;
			}

			const char * pName = pPart->Name();
			if (!pName)
			{
				return strPartName;
			}

			QString strPart = QString::fromLocal8Bit(pName);
			bool bStart = strPart.startsWith(strPartName);
			if (bStart)
			{
				lstPartName.append(strPart);
			}
		}
	}

	int nStart = lstPartName.size();
	QList<UINT> lstPartNum;
	QString strMid = QObject::tr("-");
	bool bOriIn = lstPartName.contains(strPartName);

	if (nStart == 0 || !bOriIn)
	{
		return strPartName;
	}
	else if (nStart > 0)
	{
		for (int i = 0; i < nStart; i++)
		{
			QString strTemp = lstPartName[i];
			QStringList slstPartName = strTemp.split(strPartName);
			int nNameLen = slstPartName.size();

			if (nNameLen == 2)
			{
				QString strSec = slstPartName[nNameLen - 1];
				bool bStartMid = strSec.startsWith(strMid);
				if (bStartMid)
				{
					QStringList slstSec = strSec.split(strMid);
					if (slstSec.size() == 2)
					{
						QString strSecSec = slstSec[nNameLen - 1];
						bool bTrans = false;
						UINT uNum = strSecSec.toUInt(&bTrans);
						if (bTrans)
						{
							int nAllNum = lstPartNum.size();
							if (nAllNum == 0)
							{
								lstPartNum.append(uNum);
							}
							else if (nAllNum > 0)
							{
								bool bIn = lstPartNum.contains(uNum);
								if (!bIn)
								{
									lstPartNum.append(uNum);
								}
							}
						}
					}
				}
			}
		}
	}

	UINT uPartIdx;
	int nAll = lstPartNum.size();
	for (int i = 1; i < nAll + 2; i++)
	{
		bool bIn = lstPartNum.contains(i);
		if (!bIn)
		{
			uPartIdx = i;
			break;
		}
	}

	QString strRet = strPartName + strMid + QString::number(uPartIdx);
	return strRet;
}

QString iogBasicTools::ReNameSurf(QString & strSurfName)
{
	comSurfaceMgr * pSurfMgr = comSurface::Manager();
	if (!pSurfMgr)
	{
		return strSurfName;
	}

	vector<UINT> vecSurfs;
	pSurfMgr->GetAllObject(vecSurfs);
	size_t tSurfs = vecSurfs.size();
	QList<QString> lstSurfName;

	if (tSurfs == 0)
	{
		return strSurfName;
	}
	else if (tSurfs > 0)
	{
		for (size_t i = 0; i < tSurfs; i++)
		{
			UINT uSurf = vecSurfs[i];
			comSurface * pSurf = (comSurface *)(pSurfMgr->Object(uSurf));
			if (!pSurf)
			{
				return strSurfName;
			}

			const char * pName = pSurf->Name();
			if (!pName)
			{
				return strSurfName;
			}

			QString strSurf = QString::fromLocal8Bit(pName);
			bool bStart = strSurf.startsWith(strSurfName);
			if (bStart)
			{
				lstSurfName.append(strSurf);
			}
		}
	}

	int nStart = lstSurfName.size();
	QList<UINT> lstSurfNum;
	QString strMid = QObject::tr("-");
	bool bOriIn = lstSurfName.contains(strSurfName);

	if (nStart == 0 || !bOriIn)
	{
		return strSurfName;
	}
	else if (nStart > 0)
	{
		for (int i = 0; i < nStart; i++)
		{
			QString strTemp = lstSurfName[i];
			QStringList slstSurfName = strTemp.split(strSurfName);
			int nNameLen = slstSurfName.size();
			if (nNameLen == 2)
			{
				QString strSec = slstSurfName[nNameLen - 1];
				bool bStartMid = strSec.startsWith(strMid);
				if (bStartMid)
				{
					QStringList slstSec = strSec.split(strMid);
					if (slstSec.size() == 2)
					{
						QString strSecSec = slstSec[nNameLen - 1];
						bool bTrans = false;
						UINT uNum = strSecSec.toUInt(&bTrans);
						if (bTrans)
						{
							int nAllNum = lstSurfNum.size();
							if (nAllNum == 0)
							{
								lstSurfNum.append(uNum);
							}
							else if (nAllNum > 0)
							{
								bool bIn = lstSurfNum.contains(uNum);
								if (!bIn)
								{
									lstSurfNum.append(uNum);
								}
							}
						}
					}
				}
			}
		}
	}

	UINT uPartIdx;
	int nAll = lstSurfNum.size();
	for (int i = 1; i < nAll + 2; i++)
	{
		bool bIn = lstSurfNum.contains(i);
		if (!bIn)
		{
			uPartIdx = i;
			break;
		}
	}

	QString strRet = strSurfName + strMid + QString::number(uPartIdx);
	return strRet;
}

bool iogBasicTools::Heal(TopoDS_Shape &entryShape, double dtolerance,
	double dholeRad, double dfilletRad)
{
	//AutoMergeEdges(entryShape);
	/*if (!AutoFillInnerGaps(entryShape, tolerance))
	{
		return false;
	}*/
	if(dholeRad > Precision::Confusion())
	{
		if (!AutoFillHoles(entryShape, dholeRad))
		{
			return false;
		}
	}
	/*if (dtolerance > Precision::Confusion())
	{
		if (!AutoRemoveFreeEdges(entryShape, dtolerance))
		{
			return false;
		}
	}*/
	/*if (dfilletRad > Precision::Confusion())
	{
		if (!AutoRemoveFillets(entryShape, dfilletRad))
		{
			return false;
		}
	}*/

	return true;
}

bool iogBasicTools::AutoFillInnerGaps(TopoDS_Shape &entryShape, double tolerance)
{
	if (entryShape.ShapeType() != TopAbs_COMPOUND)
	{
		return false;
	}
	TopTools_ListOfShape facelist;
	TopExp_Explorer faceExp(entryShape, TopAbs_FACE);
	for (; faceExp.More(); faceExp.Next())
	{
		TopoDS_Shape currentShape = faceExp.Current();
		if (!facelist.Contains(currentShape))
		{
			facelist.Append(currentShape);
		}
	}
	TopTools_ListOfShape listuniqueedges;
	std::vector<TopoDS_Shape> edges, neededges;

	for (auto iter : facelist)
	{
		TopExp_Explorer edgeex;
		for (edgeex.Init(iter, TopAbs_EDGE); edgeex.More(); edgeex.Next())
		{
			TopoDS_Shape currntedge = edgeex.Current();
			if (!listuniqueedges.Contains(currntedge))
			{
				listuniqueedges.Append(currntedge);
			}
		}
	}

	for (auto iter : listuniqueedges)
	{
		edges.push_back(iter);
	}

	for (int i = 0; i < edges.size(); i++)
	{
		int count = 0;
		for (int j = 0; j < edges.size(); j++)
		{
			if (iogOccKernelTools::IsShapeGeomSame(edges[i], edges[j], TopAbs_EDGE))
				count++;
		}
		if (count == 1)
			neededges.push_back(edges[i]);
	}

	if (neededges.size() == 0)
	{
		return true;
	}

	//auto group closed edges
	TopTools_ListOfShape usedshapes;
	std::vector<std::vector<TopoDS_Shape>> vecgrouped_edges;
	for (int i = 0; i < neededges.size(); i++)
	{
		TopTools_ListOfShape edgeset, listededges;
		TopoDS_Shape edge1 = neededges[i];
		std::vector<TopoDS_Shape> vecxedgegroup;
		if (usedshapes.Contains(edge1))
		{
			continue;
		}
		LoopFindAdjacentEdge(usedshapes, edge1, edgeset, neededges, listededges);
		for (auto iter : listededges)
		{
			ShapeAnalysis_Edge sae;
			TopoDS_Edge edgex = TopoDS::Edge(iter);
			TopoDS_Vertex v11 = sae.FirstVertex(edgex);
			TopoDS_Vertex v12 = sae.LastVertex(edgex);
			gp_Pnt pt1 = BRep_Tool::Pnt(v11);
			gp_Pnt pt2 = BRep_Tool::Pnt(v12);
			vecxedgegroup.push_back(iter);
		}
		if (vecxedgegroup.size() > 0)
		{
			vecgrouped_edges.push_back(vecxedgegroup);
		}
	}

	//process individual closed wires
	TopTools_ListOfShape filledfaces;
	for (int i = 0; i < vecgrouped_edges.size(); i++)
	{
		std::vector<TopoDS_Shape> vecXgroup = vecgrouped_edges[i];
		BRepFill_Filling bff;
		for (int j = 0; j < vecXgroup.size(); j++)
		{
			ShapeAnalysis_Edge sae;
			TopoDS_Edge edgex = TopoDS::Edge(vecXgroup[j]);
			TopoDS_Vertex v11 = sae.FirstVertex(edgex);
			TopoDS_Vertex v12 = sae.LastVertex(edgex);
			gp_Pnt pt1 = BRep_Tool::Pnt(v11);
			gp_Pnt pt2 = BRep_Tool::Pnt(v12);
			bff.Add(edgex, GeomAbs_C0);
		}
		//continue;
		bff.Build();
		if (!bff.IsDone())
		{
			continue;
		}
		TopoDS_Face fillface = bff.Face();

		GProp_GProps gg;
		BRepGProp::SurfaceProperties(fillface, gg);
		if (gg.Mass() < tolerance + 0.1)
		{
			filledfaces.Append(fillface);
		}
	}

	BRep_Builder B;
	TopoDS_Compound resultcompound;
	B.MakeCompound(resultcompound);
	for (auto iter : filledfaces)
	{
		B.Add(resultcompound, iter);
	}
	TopoDS_Iterator shapeiter(entryShape);
	for (; shapeiter.More(); shapeiter.Next())
	{
		B.Add(resultcompound, shapeiter.Value());
	}
	entryShape = resultcompound;
	return true;
}

bool iogBasicTools::LoopFindAdjacentEdge(TopTools_ListOfShape& usedshapes, TopoDS_Shape& startedge,
	TopTools_ListOfShape& edgeset, std::vector<TopoDS_Shape>& vecneededges, TopTools_ListOfShape& xedgegroup)
{
	if (edgeset.Contains(startedge))
	{
		return false;
	}

	ShapeAnalysis_Edge sae;
	TopoDS_Vertex v11 = sae.FirstVertex(TopoDS::Edge(startedge));
	TopoDS_Vertex v12 = sae.LastVertex(TopoDS::Edge(startedge));
	gp_Pnt pt1 = BRep_Tool::Pnt(v11);
	gp_Pnt pt2 = BRep_Tool::Pnt(v12);

	for (int j = 0; j < vecneededges.size(); j++)
	{
		TopoDS_Shape edge2 = vecneededges[j];
		if (edge2.IsSame(startedge))
		{
			continue;
		}
		if (usedshapes.Contains(edge2))
		{
			continue;
		}
		ShapeAnalysis_Edge sae2;
		TopoDS_Vertex v21 = sae2.FirstVertex(TopoDS::Edge(edge2));
		TopoDS_Vertex v22 = sae2.LastVertex(TopoDS::Edge(edge2));
		gp_Pnt pt3 = BRep_Tool::Pnt(v21);
		gp_Pnt pt4 = BRep_Tool::Pnt(v22);
		if (iogOccKernelTools::IsShapeGeomSame(v11, v21, TopAbs_VERTEX) || iogOccKernelTools::IsShapeGeomSame(v11, v22, TopAbs_VERTEX) ||
			iogOccKernelTools::IsShapeGeomSame(v12, v21, TopAbs_VERTEX) || iogOccKernelTools::IsShapeGeomSame(v12, v22, TopAbs_VERTEX))
		{
			if (!usedshapes.Contains(startedge))
			{
				usedshapes.Append(startedge);
			}
			if (!usedshapes.Contains(edge2))
			{
				usedshapes.Append(edge2);
			}
			if(!xedgegroup.Contains(startedge))
			{
				xedgegroup.Append(startedge);
			}
			if (!xedgegroup.Contains(edge2))
			{
				xedgegroup.Append(edge2);
			}
			if (!LoopFindAdjacentEdge(usedshapes, edge2, edgeset, vecneededges, xedgegroup))
			{
				return false;
			}
		}
	}
	return true;
}

bool iogBasicTools::AutoFillHoles(TopoDS_Shape &entryShape, double maxRadius)
{
	if (maxRadius <= 0)
	{
		return true;
	}
	ShapeUpgrade_RemoveInternalWires sri(entryShape);
	sri.RemoveFaceMode() = true;
	sri.MinArea() = pow(maxRadius, 2)*M_PI;
	sri.Perform();
	entryShape = sri.GetResult();
	return true;
}

bool iogBasicTools::AutoRemoveFreeEdges(TopoDS_Shape &entryShape, double tolerance)
{
	if (entryShape.ShapeType() != TopAbs_COMPOUND)
	{
		return false;
	}
	TopTools_ListOfShape facelist;
	TopExp_Explorer faceExp(entryShape, TopAbs_FACE);
	for (; faceExp.More(); faceExp.Next())
	{
		TopoDS_Shape currentShape = faceExp.Current();
		if (!facelist.Contains(currentShape))
		{
			facelist.Append(currentShape);
		}
	}
	TopTools_ListOfShape listuniqueedges;
	std::vector<TopoDS_Shape> edges, neededges;

	for (auto iter : facelist)
	{
		TopExp_Explorer edgeex;
		for (edgeex.Init(iter, TopAbs_EDGE); edgeex.More(); edgeex.Next())
		{
			TopoDS_Shape currntedge = edgeex.Current();
			if (!listuniqueedges.Contains(currntedge))
			{
				listuniqueedges.Append(currntedge);
			}
		}
	}

	for (auto iter : listuniqueedges)
	{
		edges.push_back(iter);
	}

	for (int i = 0; i < edges.size(); i++)
	{
		int count = 0;
		for (int j = 0; j < edges.size(); j++)
		{
			if (iogOccKernelTools::IsShapeGeomSame(edges[i], edges[j], TopAbs_EDGE))
				count++;
		}
		if (count == 1)
			neededges.push_back(edges[i]);
	}

	if (neededges.size() == 0)
	{
		return true;
	}

	//TopoDS_Compound compound;
	//BRep_Builder b;
	//b.MakeCompound(compound);
	//for (int i = 0; i < neededges.size(); i++)
	//{
	//	b.Add(compound, neededges[i]);
	//}


	BRepTools_ReShape brs;
	for (int i = 0; i < neededges.size(); i++)
	{
		brs.Remove(neededges[i]);
		entryShape = brs.Apply(entryShape);
	}
	//entryShape = brs.Apply(entryShape);
	return true;
}

bool iogBasicTools::AutoRemoveFillets(TopoDS_Shape &entryShape, double dmaxRad)
{
	TopTools_ListOfShape cylinderFaceList, preselectFaceList, needFaceList;
	TopTools_IndexedMapOfShape aShapeMap;
	TopExp::MapShapes(entryShape, TopAbs_FACE, aShapeMap);

	for (auto i = aShapeMap.cbegin(); i != aShapeMap.cend(); i++)
	{
		cylinderFaceList.Append(*i);
	}

	for (auto i = aShapeMap.cbegin(); i != aShapeMap.cend(); i++)
	{
		BRepAdaptor_Surface brs(TopoDS::Face(*i));
		if (brs.GetType() == GeomAbs_Cylinder)
		{
			TopExp_Explorer edgex(*i, TopAbs_EDGE);
			bool ifadd = false;
			int nFilletEdge = 0;
			for (; edgex.More(); edgex.Next())
			{
				nFilletEdge++;
				TopoDS_Shape currentshape = edgex.Current();
				BRepAdaptor_Curve bac(TopoDS::Edge(currentshape));
				GeomAbs_CurveType type = bac.GetType();
				double dstart = bac.FirstParameter();
				double dend = bac.LastParameter();
				if (bac.GetType() == GeomAbs_Circle)
				{
					gp_Pnt startpt = bac.Value(dstart);
					gp_Pnt endpt = bac.Value(dend);
					gp_Circ circ = bac.Circle();
					gp_Pnt centerpt = circ.Location();
					if (circ.Radius() > dmaxRad)
					{
						continue;
					}
					gp_Dir dir1(startpt.X() - centerpt.X(),
						startpt.Y() - centerpt.Y(),
						startpt.Z() - centerpt.Z());
					gp_Dir dir2(endpt.X() - centerpt.X(),
						endpt.Y() - centerpt.Y(),
						endpt.Z() - centerpt.Z());
					double dangle = dir1.Angle(dir2);
					if (0 < dangle <= M_PI)
					{
						ifadd = true;
						break;
					}
				}
				if (bac.GetType() == GeomAbs_BSplineCurve)
				{
					gp_Pnt pt1 = bac.Value(dstart);
					gp_Pnt pt2 = bac.Value(dstart + (-dstart + dend) / 4);
					gp_Pnt pt3 = bac.Value(dstart + (-dstart + dend) / 2);
					gp_Pnt pt4 = bac.Value(dstart + 3 * (-dstart + dend) / 4);
					gp_Pnt pt5 = bac.Value(dend);

					double d1 = pt2.Distance(pt1);
					double d2 = pt3.Distance(pt2);
					double d3 = pt4.Distance(pt3);
					double d4 = pt5.Distance(pt4);

					if (abs(d1 - d2) < 0.001 && abs(d2 - d3) < 0.001 &&abs(d3 - d4) < 0.001)
					{
						ifadd = true;
						break;
					}
				}
			}
			if (nFilletEdge % 4 == 0 && ifadd && !preselectFaceList.Contains(*i)) //
			{
				preselectFaceList.Append(*i);
			}
		}
	}

	for (auto iter : preselectFaceList)
	{
		if (iogOccKernelTools::IsAIndipendentInTheList(iter, preselectFaceList))
		{
			needFaceList.Append(iter);
		}
	}

	if (needFaceList.Size() > 0)
	{
		BRepAlgoAPI_Defeaturing adf;
		adf.SetShape(entryShape);
		adf.AddFacesToRemove(needFaceList);
		adf.SetRunParallel(true);
		adf.SetToFillHistory(false);
		adf.Build();

		if (!adf.IsDone())
			return false;

		entryShape = adf.Shape();
	}
	return true;
}

bool iogBasicTools::AutoMergeEdges(TopoDS_Shape &entryShape)
{
	TopExp_Explorer facex(entryShape, TopAbs_FACE);
	TopTools_ListOfShape allvtxs, avoidvtxs, needvtxs, uselessEdges;
	for (; facex.More(); facex.Next())
	{
		TopoDS_Face currentface = TopoDS::Face(facex.Current());
		TopTools_ListOfShape vtxs;
		if (!iogOccKernelTools::GetPlanarFaceApexs(currentface, vtxs, uselessEdges))
		{
			continue;
		}
		
		TopTools_ListOfShape edges;
		TopExp_Explorer edgex(currentface, TopAbs_EDGE);
		for (; edgex.More(); edgex.Next())
		{
			edges.Append(edgex.Current());
		}
		TopoDS_Vertex startVtx = TopoDS::Vertex(*vtxs.begin());
		std::vector<TopoDS_Shape> vecNewEdges;
		if (!iogOccKernelTools::ReOrgnizeEdgeOrderWire(edges, startVtx, vecNewEdges))
		{
			continue;
		}
		std::vector<std::vector<TopoDS_Shape>> vecGroups;
		if (!iogOccKernelTools::GroupEdgesInWire(vecNewEdges, vecGroups))
		{
			continue;
		}
		TopoDS_Wire newwire;
		if (!iogOccKernelTools::ReOrderEdgesInWire(vecGroups, newwire))
		{
			continue;
		}

		BRepTools_ReShape brs;
		TopExp_Explorer wirex(currentface, TopAbs_WIRE);
		TopoDS_Wire oriWire;
		for (; wirex.More(); wirex.Next())
		{
			oriWire = TopoDS::Wire(wirex.Current());
		}
		brs.Replace(oriWire, newwire);
		entryShape = brs.Apply(entryShape);
	}

	return true;
}

//bool iogBasicTools::ClearFreeEdges(bool firstloop, UINT uMaxFaceIx,
//	TopoDS_Shape entryshape, UINT maxsurfaceidx)
//{
//	double dPointEqual = pointEqualTol;
//
//	std::vector<std::pair<UINT, gp_Pnt>> vecPairVtxIndex;
//	comFaceMgr *pFaceMgr = comFace::Manager();
//	comVerticeMgr * pVertMgr = comVertice::Manager();
//	std::vector<TopoDS_Face> alltopofaces;
//
//	//freeEdge
//	vector<UINT> vecFaceIx, vecusedFaceIx;
//	pFaceMgr->GetAllObject(vecFaceIx);
//
//	for (UINT i = 0; i < vecFaceIx.size(); i++)
//	{
//		if (!firstloop)
//		{
//			if (i < uMaxFaceIx + 1)
//			{
//				continue;
//			}
//		}
//
//		vecusedFaceIx.push_back(i);
//	}
//
//	algCalcOutline algcal;
//	algcal.SetFace(vecusedFaceIx);
//	algcal.Calculate();
//
//	vector<vector<pair<basePoint, basePoint>>> vecBoundary;
//	algcal.GetBoundary(vecBoundary);
//
//	vector<vector<algEdge>> vecAlgEdges;
//	vector<algEdge> vecfreealg;
//	algcal.GetBoundary(vecAlgEdges);
//	for (int i = 0; i < vecAlgEdges.size(); i++)
//	{
//		vector<algEdge> xvec = vecAlgEdges[i];
//		for (int j = 0; j < xvec.size(); j++)
//		{
//			algEdge xedge = xvec[j];
//			vecfreealg.push_back(xedge);
//			UINT index1 = xedge.Vertice1();
//			comVertice* pVertice1 = (comVertice*)pVertMgr->Object(index1);
//			double dCoord1[3];
//			pVertice1->Xyz(dCoord1);
//			gp_Pnt pt1(dCoord1[0], dCoord1[1], dCoord1[2]);
//			vecPairVtxIndex.push_back(std::pair<UINT, gp_Pnt>(index1, pt1));
//
//			UINT index2 = xedge.Vertice2();
//			comVertice* pVertice2 = (comVertice*)pVertMgr->Object(index2);
//			double dCoord2[3];
//			pVertice2->Xyz(dCoord2);
//			gp_Pnt pt2(dCoord2[0], dCoord2[1], dCoord2[2]);
//			vecPairVtxIndex.push_back(std::pair<UINT, gp_Pnt>(index2, pt2));
//		}
//	}
//
//	TopoDS_Compound compound;
//	BRep_Builder b;
//	b.MakeCompound(compound);
//
//	std::vector<std::vector<TopoDS_Edge>> groups;
//	std::vector<TopoDS_Edge> vecFreeEdge;
//	int freeedgesize = vecFreeEdge.size();
//	for (int i = 0; i < vecBoundary.size(); i++)
//	{
//		vector<pair<basePoint, basePoint>> vecbase = vecBoundary[i];
//		for (int j = 0; j < vecbase.size(); j++)
//		{
//			pair<basePoint, basePoint> pairbase = vecbase[j];
//			basePoint bp1 = pairbase.first;
//			basePoint bp2 = pairbase.second;
//
//			gp_Pnt pt1(bp1.X(), bp1.Y(), bp1.Z());
//			gp_Pnt pt2(bp2.X(), bp2.Y(), bp2.Z());
//
//			TopoDS_Edge newedge = BRepBuilderAPI_MakeEdge(pt1, pt2).Edge();
//			vecFreeEdge.push_back(newedge);
//			b.Add(compound, newedge);
//		}
//	}
//
//	//iogOccKernelTools::GroupEdgesInTriangles(dPointEqual, vecFreeEdge, groups);
//	SplitGroupTriangles(dPointEqual, vecFreeEdge, groups);
//
//	int tri = 0;
//	std::vector<std::vector<TopoDS_Edge>> vecnextgroups;
//	for (int i = 0; i < groups.size(); i++)
//	{
//		//deal with 3
//		if (groups[i].size() == 3)
//		{
//			std::vector<gp_Pnt> vecPnt;
//			ShapeAnalysis_Edge sae;
//			TopoDS_Edge edge1 = groups[i][0];
//			TopoDS_Vertex vtx1 = sae.FirstVertex(edge1);
//			gp_Pnt pt1 = BRep_Tool::Pnt(vtx1);
//			AddPntToVec(pt1, vecPnt);
//			TopoDS_Vertex vtx2 = sae.LastVertex(edge1);
//			gp_Pnt pt2 = BRep_Tool::Pnt(vtx2);
//			AddPntToVec(pt2, vecPnt);
//
//			TopoDS_Edge edge2 = groups[i][1];
//			TopoDS_Vertex vtx3 = sae.FirstVertex(edge2);
//			gp_Pnt pt3 = BRep_Tool::Pnt(vtx3);
//			AddPntToVec(pt3, vecPnt);
//			TopoDS_Vertex vtx4 = sae.LastVertex(edge2);
//			gp_Pnt pt4 = BRep_Tool::Pnt(vtx4);
//			AddPntToVec(pt4, vecPnt);
//
//			TopoDS_Edge edge3 = groups[i][2];
//			TopoDS_Vertex vtx5 = sae.FirstVertex(edge3);
//			gp_Pnt pt5 = BRep_Tool::Pnt(vtx5);
//			AddPntToVec(pt5, vecPnt);
//			TopoDS_Vertex vtx6 = sae.LastVertex(edge3);
//			gp_Pnt pt6 = BRep_Tool::Pnt(vtx6);
//			AddPntToVec(pt6, vecPnt);
//
//			gp_Pnt ptx1 = vecPnt[0];
//			gp_Pnt ptx2 = vecPnt[1];
//			gp_Pnt ptx3 = vecPnt[2];
//
//			gp_Dir dir1(ptx2.X() - ptx1.X(), ptx2.Y() - ptx1.Y(), ptx2.Z() - ptx1.Z());
//			gp_Dir dir2(ptx3.X() - ptx2.X(), ptx3.Y() - ptx2.Y(), ptx3.Z() - ptx2.Z());
//
//			gp_Dir dirNormal = dir1.Crossed(dir2);
//			double dNormal[3];
//			dNormal[0] = dirNormal.X();
//			dNormal[1] = dirNormal.Y();
//			dNormal[2] = dirNormal.Z();
//
//			UINT index1, index2, index3;
//			std::map<UINT, gp_Pnt>::iterator iter;
//			for (int j = 0; j < vecPairVtxIndex.size(); j++)
//			{
//				if (vecPairVtxIndex[j].second.IsEqual(ptx1, dPointEqual))
//					index1 = vecPairVtxIndex[j].first;
//				if (vecPairVtxIndex[j].second.IsEqual(ptx2, dPointEqual))
//					index2 = vecPairVtxIndex[j].first;
//				if (vecPairVtxIndex[j].second.IsEqual(ptx3, dPointEqual))
//					index3 = vecPairVtxIndex[j].first;
//			}
//			comVerticeMgr * pVtxMgr = comVertice::Manager();
//			if (index1 > pVtxMgr->MaxIndex() || index2 > pVtxMgr->MaxIndex() ||
//				index3 > pVtxMgr->MaxIndex())
//			{
//				continue;
//			}
//
//			UINT uVerts[3];
//			uVerts[0] = index1;
//			uVerts[1] = index2;
//			uVerts[2] = index3;
//
//			comFace * pFace = new comFace(uVerts);
//			if (!pFace)
//			{
//				return false;
//			}
//			pFace->SetNormal(dNormal);
//			pFaceMgr->Create(pFace);
//			comSurfaceMgr * pSurfMgr = comSurface::Manager();
//			if (!pSurfMgr)
//			{
//				return false;
//			}
//			pFace->SetSurface(pSurfMgr->MaxIndex());
//			tri++;
//		}
//		else
//		{
//			vecnextgroups.push_back(groups[i]);
//		}
//	}
//
//	return true;
//}

bool iogBasicTools::ClearFreeEdges(bool bIfSTP, double scale, bool firstloop, UINT uMaxFaceIx,
	TopoDS_Shape entryshape, UINT maxsurfaceidx, QHash<QString, UINT> hashVertIdx)
{
	double dPointEqual = pointEqualTol;

	std::vector<std::pair<UINT, gp_Pnt>> vecPairVtxIndex;
	comFaceMgr *pFaceMgr = comFace::Manager();
	comVerticeMgr * pVertMgr = comVertice::Manager();
	std::vector<TopoDS_Face> alltopofaces;

	//freeEdge
	vector<UINT> vecFaceIx, vecusedFaceIx;
	pFaceMgr->GetAllObject(vecFaceIx);

	for (UINT i = 0; i < vecFaceIx.size(); i++)
	{
		if (!firstloop)
		{
			if (i < uMaxFaceIx + 1)
			{
				continue;
			}
		}

		vecusedFaceIx.push_back(i);
	}

	algCalcOutline algcal;
	algcal.SetFace(vecusedFaceIx);
	algcal.Calculate();

	vector<vector<pair<basePoint, basePoint>>> vecBoundary, vecBoundaryUse;
	algcal.GetBoundary(vecBoundary); 

	for (int i = 0; i < vecBoundary.size(); i++)
	{
		vector<pair<basePoint, basePoint>> currentvec = vecBoundary[i];
		vector<pair<basePoint, basePoint>> nextvec;
		for (int j = 0; j < currentvec.size(); j++)
		{
			pair<basePoint, basePoint> pairbase = currentvec[j];
			basePoint newPt1, newPt2;
			double dVCoord1[3], dVCoord2[3];
			if (bIfSTP)
			{
				dVCoord1[0] = pairbase.first.X() * 1000;
				dVCoord1[1] = pairbase.first.Y() * 1000;
				dVCoord1[2] = pairbase.first.Z() * 1000;

				dVCoord2[0] = pairbase.second.X() * 1000;
				dVCoord2[1] = pairbase.second.Y() * 1000;
				dVCoord2[2] = pairbase.second.Z() * 1000;

				newPt1.Set(dVCoord1);
				newPt2.Set(dVCoord2);
				nextvec.push_back(pair<basePoint, basePoint>(newPt1, newPt2));
			}
			else
			{
				dVCoord1[0] = pairbase.first.X() / scale;
				dVCoord1[1] = pairbase.first.Y() / scale;
				dVCoord1[2] = pairbase.first.Z() / scale;

				dVCoord2[0] = pairbase.second.X() / scale;
				dVCoord2[1] = pairbase.second.Y() / scale;
				dVCoord2[2] = pairbase.second.Z() / scale;

				newPt1.Set(dVCoord1);
				newPt2.Set(dVCoord2);
				nextvec.push_back(pair<basePoint, basePoint>(newPt1, newPt2));
			}
		}
		vecBoundaryUse.push_back(nextvec);
	}

	std::vector<std::pair<UINT, UINT>> vecIdxPair;
	vector<vector<algEdge>> vecAlgEdges;
	vector<algEdge> vecfreealg;
	algcal.GetBoundary(vecAlgEdges);
	for (int i = 0; i < vecAlgEdges.size(); i++)
	{
		vector<algEdge> xvec = vecAlgEdges[i];
		for (int j = 0; j < xvec.size(); j++)
		{
			algEdge xedge = xvec[j];
			//vecfreealg.push_back(xedge);
			UINT index1 = xedge.Vertice1();
			UINT uVertMax = pVertMgr->MaxIndex();
			comVertice* pVertice1 = (comVertice*)pVertMgr->Object(index1);
			double dCoord1[3];
			pVertice1->Xyz(dCoord1);
			gp_Pnt pt1(dCoord1[0], dCoord1[1], dCoord1[2]);
			vecPairVtxIndex.push_back(std::pair<UINT, gp_Pnt>(index1, pt1));

			UINT index2 = xedge.Vertice2();
			comVertice* pVertice2 = (comVertice*)pVertMgr->Object(index2);
			double dCoord2[3];
			pVertice2->Xyz(dCoord2);
			gp_Pnt pt2(dCoord2[0], dCoord2[1], dCoord2[2]);
			vecPairVtxIndex.push_back(std::pair<UINT, gp_Pnt>(index2, pt2));
			vecIdxPair.push_back(std::pair<UINT, UINT>(index1, index2));
		}
	}

	std::vector<std::vector<TopoDS_Edge>> groups;
	std::vector<TopoDS_Edge> vecFreeEdge;

	/*TopoDS_Compound compound;
	BRep_Builder b;
	b.MakeCompound(compound);
	for (int i = 0; i < vecBoundary.size(); i++)
	{
		vector<pair<basePoint, basePoint>> vecbase = vecBoundary[i];
		for (int j = 0; j < vecbase.size(); j++)
		{
			pair<basePoint, basePoint> pairbase = vecbase[j];
			basePoint bp1 = pairbase.first;
			basePoint bp2 = pairbase.second;

			gp_Pnt pt1(bp1.X(), bp1.Y(), bp1.Z());
			gp_Pnt pt2(bp2.X(), bp2.Y(), bp2.Z());

			if (pt1.Distance(pt2) <= Precision::Confusion())
			{
				continue;
			}
			TopoDS_Edge newedge = BRepBuilderAPI_MakeEdge(pt1, pt2).Edge();
			vecFreeEdge.push_back(newedge);
			b.Add(compound, newedge);
		}
		int a = 0;
	}*/
	
	//FilterFreeEdgeVtxIndexes(dPointEqual, vecPairVtxIndex, vecFreeEdge, vecIdxPair);
	vector<vector<UINT>> groupedIndex;
	GetClosedVertexIdxGroup(vecBoundaryUse, hashVertIdx, groupedIndex);

	//determine two vetex
	for (int i = 0; i < groupedIndex.size(); i++)
	{
		//vector<UINT> currentClosed = groupedIndex[i];
		vector<UINT> currentClosedPre = groupedIndex[i];
		vector<UINT> currentClosed;
		for (int j = 0; j < currentClosedPre.size() - 1; j++)
		{
			currentClosed.push_back(currentClosedPre[j]);
		}

		if (currentClosed.size() > 3)
		{
			//jude if to sew
			vector<UINT> vecVtx;
			for (int j = 0; j < currentClosed.size() - 2; j++)
			{
				UINT ubegin = currentClosed[j];
				UINT umid = currentClosed[j + 1];
				UINT uend = currentClosed[j + 2];

				if (ubegin == umid || uend == umid || ubegin == uend)
				{
					continue;
				}

				comVertice* pVtxBegin = (comVertice*)pVertMgr->Object(ubegin);
				double dCoord1[3];
				pVtxBegin->Xyz(dCoord1);
				gp_Pnt pt1(dCoord1[0], dCoord1[1], dCoord1[2]);
				comVertice* pVtxMid = (comVertice*)pVertMgr->Object(umid);
				double dCoord2[3];
				pVtxMid->Xyz(dCoord2);
				gp_Pnt pt2(dCoord2[0], dCoord2[1], dCoord2[2]);
				comVertice* pVtxEnd = (comVertice*)pVertMgr->Object(uend);
				double dCoord3[3];
				pVtxEnd->Xyz(dCoord3);
				gp_Pnt pt3(dCoord3[0], dCoord3[1], dCoord3[2]);

				gp_Dir dir1(pt2.X() - pt1.X(), pt2.Y() - pt1.Y(), pt2.Z() - pt1.Z());
				gp_Dir dir2(pt2.X() - pt3.X(), pt2.Y() - pt3.Y(), pt2.Z() - pt3.Z());

				double dAngle = CalculateAngleBetweenDirs(dir1, dir2);
				if (dAngle < angleTolerance)
				{
					vecVtx.push_back(umid);
				}
				/*else if (10 < dAngle && dAngle < 90)
					break;*/
			}

			//First vtx
			{
				UINT uVbeginF = currentClosed[currentClosed.size() - 2];
				UINT uVmidF = currentClosed[currentClosed.size() - 1];
				UINT uVendF = currentClosed[0];

				if (uVbeginF == uVmidF || uVendF == uVmidF || uVbeginF == uVendF)
				{
					continue;
				}

				comVertice* pVVtxBeginF = (comVertice*)pVertMgr->Object(uVbeginF);
				double dVCoord1F[3];
				pVVtxBeginF->Xyz(dVCoord1F);
				gp_Pnt ptV1F(dVCoord1F[0], dVCoord1F[1], dVCoord1F[2]);
				comVertice* pVVtxMidF = (comVertice*)pVertMgr->Object(uVmidF);
				double dVCoord2F[3];
				pVVtxMidF->Xyz(dVCoord2F);
				gp_Pnt ptV2F(dVCoord2F[0], dVCoord2F[1], dVCoord2F[2]);
				comVertice* pVVtxEndF = (comVertice*)pVertMgr->Object(uVendF);
				double dVCoord3F[3];
				pVVtxEndF->Xyz(dVCoord3F);
				gp_Pnt ptV3F(dVCoord3F[0], dVCoord3F[1], dVCoord3F[2]);

				gp_Dir dirV1F(ptV2F.X() - ptV1F.X(), ptV2F.Y() - ptV1F.Y(), ptV2F.Z() - ptV1F.Z());
				gp_Dir dirV2F(ptV2F.X() - ptV3F.X(), ptV2F.Y() - ptV3F.Y(), ptV2F.Z() - ptV3F.Z());

				double dVAngleF = CalculateAngleBetweenDirs(dirV1F, dirV2F);
				if (dVAngleF < angleTolerance)
				{
					vecVtx.push_back(uVmidF);
				}
			}

			//second
			{
				UINT uVbegin = currentClosed[currentClosed.size() - 1];
				UINT uVmid = currentClosed[0];
				UINT uVend = currentClosed[1];

				if (uVbegin == uVmid || uVend == uVmid || uVbegin == uVend)
				{
					continue;
				}

				comVertice* pVVtxBegin = (comVertice*)pVertMgr->Object(uVbegin);
				double dVCoord1[3];
				pVVtxBegin->Xyz(dVCoord1);
				gp_Pnt ptV1(dVCoord1[0], dVCoord1[1], dVCoord1[2]);
				comVertice* pVVtxMid = (comVertice*)pVertMgr->Object(uVmid);
				double dVCoord2[3];
				pVVtxMid->Xyz(dVCoord2);
				gp_Pnt ptV2(dVCoord2[0], dVCoord2[1], dVCoord2[2]);
				comVertice* pVVtxEnd = (comVertice*)pVertMgr->Object(uVend);
				double dVCoord3[3];
				pVVtxEnd->Xyz(dVCoord3);
				gp_Pnt ptV3(dVCoord3[0], dVCoord3[1], dVCoord3[2]);

				gp_Dir dirV1(ptV2.X() - ptV1.X(), ptV2.Y() - ptV1.Y(), ptV2.Z() - ptV1.Z());
				gp_Dir dirV2(ptV2.X() - ptV3.X(), ptV2.Y() - ptV3.Y(), ptV2.Z() - ptV3.Z());

				double dVAngle = CalculateAngleBetweenDirs(dirV1, dirV2);
				if (dVAngle < angleTolerance)
				{
					vecVtx.push_back(uVmid);
				}
			}

			if (vecVtx.size() != 2)
				continue;

			vector<UINT> vecFirstList, vecSecondList;
			int nBegin, nEnd, nRealBegin, nRealEnd;
			for (int j = 0; j < currentClosed.size(); j++)
			{
				if (currentClosed[j] == vecVtx[0])
				{
					nBegin = j;
					nRealBegin = j;
				}
				else if (currentClosed[j] == vecVtx[1])
				{
					nEnd = j;
					nRealEnd = j;
				}
			}
			if (nBegin > nEnd)
			{
				nRealBegin = nEnd;
				nRealEnd = nBegin;
			}
			for (int j = nRealBegin + 1; j < nRealEnd; j++)
			{
				vecFirstList.push_back(currentClosed[j]);
			}
			
			for (int j = nRealEnd + 1; j < currentClosed.size(); j++)
			{
				vecSecondList.push_back(currentClosed[j]);
			}
			for (int j = 0; j < nRealBegin; j++)
			{
				vecSecondList.push_back(currentClosed[j]);
			}
			
			if (vecFirstList.size() == vecSecondList.size())
				ReplaceVertex(vecusedFaceIx, vecFirstList, vecSecondList);
			else if (vecFirstList.size() > vecSecondList.size() && vecSecondList.size() != 0)
			{
				int nLess = vecFirstList.size() - vecSecondList.size();
				vector<UINT> vecCurrentList;
				for (int k = nLess; k < vecFirstList.size(); k++)
				{
					vecCurrentList.push_back(vecFirstList[k]);
				}
				ReplaceVertex(vecusedFaceIx, vecSecondList, vecCurrentList);
			}
			else if (vecFirstList.size() < vecSecondList.size() && vecFirstList.size() != 0)
			{
				int nLess = vecSecondList.size() - vecFirstList.size();
				vector<UINT> vecCurrentList;
				for (int k = nLess; k < vecSecondList.size(); k++)
				{
					vecCurrentList.push_back(vecSecondList[k]);
				}
				ReplaceVertex(vecusedFaceIx, vecFirstList, vecCurrentList);
			}
			else if (vecFirstList.size() < vecSecondList.size() && vecFirstList.size() == 0) 
			{
				for (int i = 0; i < vecSecondList.size() - 1; i++)
				{
					vector<UINT> vecSub;
					vecSub.push_back(vecVtx[0]);
					vecSub.push_back(vecSecondList[i]);
					vecSub.push_back(vecSecondList[i + 1]);
					FixTrigangle(vecSub);
				}
				vector<UINT> vecLast;
				vecLast.push_back(vecVtx[0]);
				vecLast.push_back(vecVtx[1]);
				vecLast.push_back(vecSecondList[vecSecondList.size() - 1]);
				FixTrigangle(vecLast);
			}
			else if (vecFirstList.size() > vecSecondList.size() && vecSecondList.size() == 0)
			{
				for (int i = 0; i < vecFirstList.size() - 1; i++)
				{
					vector<UINT> vecSub;
					vecSub.push_back(vecVtx[0]);
					vecSub.push_back(vecFirstList[i]);
					vecSub.push_back(vecFirstList[i + 1]);
					FixTrigangle(vecSub);
				}
				vector<UINT> vecLast;
				vecLast.push_back(vecVtx[0]);
				vecLast.push_back(vecVtx[1]);
				vecLast.push_back(vecFirstList[vecFirstList.size() - 1]);
				FixTrigangle(vecLast);
			}
		}
		else if (currentClosed.size() == 3)
		{
			//sew as a triangle
			FixTrigangle(currentClosed);
		}
	}

	/*std::vector<std::vector<UINT>> vecTris, vecRealTris;
	FindTriangleIndex(vecIdxPair, vecTris);
	for (int i = 0; i < vecTris.size(); i++)
	{
		bool bIfAdd = true;
		std::vector<UINT> vecCom1 = vecTris[i];
		UINT u11 = vecCom1[0];
		UINT u12 = vecCom1[1];
		UINT u13 = vecCom1[2];

		for (int j = 0; j < vecRealTris.size(); j++)
		{
			std::vector<UINT> vecCom2 = vecRealTris[j];
			UINT u21 = vecCom2[0];
			UINT u22 = vecCom2[1];
			UINT u23 = vecCom2[2];
			if ((u11 == u21 && u12 == u22 && u13 == u23) ||
				(u11 == u21 && u12 == u23 && u13 == u22) ||
				(u11 == u22 && u12 == u21 && u13 == u23) ||
				(u11 == u22 && u12 == u23 && u13 == u21) ||
				(u11 == u23 && u12 == u21 && u13 == u22) ||
				(u11 == u23 && u12 == u22 && u13 == u21))
			{
				bIfAdd = false;
				break;
			}
		}

		if (bIfAdd)
		{
			vecRealTris.push_back(vecCom1);
		}
	}*/
	
	//deal with 3
	/*for (int i = 0; i < vecRealTris.size(); i++)
	{	
		double dProgress = i * 100 / vecRealTris.size();
		TheExe.ProgressSetValue(dProgress);

		std::vector<UINT> vecCom1 = vecRealTris[i];
		UINT index1 = vecCom1[0];
		UINT index2 = vecCom1[1];
		UINT index3 = vecCom1[2];

		gp_Pnt ptx1, ptx2, ptx3;
		for (int j = 0; j < vecPairVtxIndex.size(); j++)
		{
			if (vecPairVtxIndex[j].first == index1)
			{
				ptx1 = vecPairVtxIndex[j].second;
				break;
			}
		}
		for (int j = 0; j < vecPairVtxIndex.size(); j++)
		{
			if (vecPairVtxIndex[j].first == index2)
			{
				ptx2 = vecPairVtxIndex[j].second;
				break;
			}
		}
		for (int j = 0; j < vecPairVtxIndex.size(); j++)
		{
			if (vecPairVtxIndex[j].first == index3)
			{
				ptx3 = vecPairVtxIndex[j].second;
				break;
			}
		}
		
		if (ptx1.IsEqual(ptx2, pointEqualTol) ||
			ptx2.IsEqual(ptx3, pointEqualTol) ||
			ptx1.IsEqual(ptx3, pointEqualTol))
		{
			continue;
		}
		gp_Dir dir1(ptx2.X() - ptx1.X(), ptx2.Y() - ptx1.Y(), ptx2.Z() - ptx1.Z());
		gp_Dir dir2(ptx3.X() - ptx2.X(), ptx3.Y() - ptx2.Y(), ptx3.Z() - ptx2.Z());

		if (dir1.IsParallel(dir2, parallTolerance))
		{
			continue;
		}
		
		gp_Dir dirNormal = dir1.Crossed(dir2);
		double dNormal[3];
		dNormal[0] = dirNormal.X();
		dNormal[1] = dirNormal.Y();
		dNormal[2] = dirNormal.Z();
		
		comVerticeMgr * pVtxMgr = comVertice::Manager();
		if (index1 > pVtxMgr->MaxIndex() || index2 > pVtxMgr->MaxIndex() ||
			index3 > pVtxMgr->MaxIndex())
		{
			continue;
		}

		UINT uVerts[3];
		uVerts[0] = index1;
		uVerts[1] = index2;
		uVerts[2] = index3;

		comFace * pFace = new comFace(uVerts); 
		if (!pFace)
		{
			return false;
		}
		pFace->SetNormal(dNormal);
		pFaceMgr->Create(pFace);
		comSurfaceMgr * pSurfMgr = comSurface::Manager();
		if (!pSurfMgr)
		{
			return false;
		}
		pFace->SetSurface(pSurfMgr->MaxIndex());
	}*/

	//deal with self-intersect edges
	/*std::vector<std::pair<UINT, UINT>> vecnearestpt;
	for (int i = 0; i < vecnextgroups.size(); i++)
	{
		std::vector<TopoDS_Edge> vecsubgroup = vecnextgroups[i];
		if (vecsubgroup.size() > 3)
		{
			FindMergePairs(entryshape, vecsubgroup, vecnearestpt);
		}
	}*/

	return true;
}

bool iogBasicTools::FixTrigangle(vector<UINT> currentClosed)
{
	comSurfaceMgr * pSurfMgr = comSurface::Manager();
	comFaceMgr * pFaceMgr = comFace::Manager();
	comVerticeMgr * pVertMgr = comVertice::Manager();

	UINT uVerts[3];
	uVerts[0] = currentClosed[0];
	uVerts[1] = currentClosed[1];
	uVerts[2] = currentClosed[2];

	comFace * pFace = new comFace(uVerts);
	comVertice* vtx1 = (comVertice*)(pVertMgr->Object(uVerts[0]));
	double dCoord1[3];
	vtx1->Xyz(dCoord1);
	gp_Pnt pt1(dCoord1[0], dCoord1[1], dCoord1[2]);
	comVertice* vtx2 = (comVertice*)(pVertMgr->Object(uVerts[1]));
	double dCoord2[3];
	vtx2->Xyz(dCoord2);
	gp_Pnt pt2(dCoord2[0], dCoord2[1], dCoord2[2]);
	comVertice* vtx3 = (comVertice*)(pVertMgr->Object(uVerts[2]));
	double dCoord3[3];
	vtx3->Xyz(dCoord3);
	gp_Pnt pt3(dCoord3[0], dCoord3[1], dCoord3[2]);

	gp_Dir calculateNormalAssistDir1(pt1.X() - pt2.X(), pt1.Y() - pt2.Y(),
		pt1.Z() - pt2.Z());
	gp_Dir calculateNormalAssistDir2(pt1.X() - pt3.X(), pt1.Y() - pt3.Y(),
		pt1.Z() - pt3.Z());
	gp_Dir triNormal = calculateNormalAssistDir1.Crossed(calculateNormalAssistDir2);
	double dNormal[3] = { triNormal.X(), triNormal.Y(), triNormal.Z() };
	pFace->SetNormal(dNormal);

	pFace->SetNormal(dNormal);
	pFaceMgr->Create(pFace);
	pFace->SetSurface(pSurfMgr->MaxIndex());

	return true;
}

bool iogBasicTools::ReplaceVertex(vector<UINT> vecFaceIx, vector<UINT> vecFirstList, vector<UINT> vecSecondList)
{
	comFaceMgr * pFaceMgr = comFace::Manager();
	comVerticeMgr * pVertMgr = comVertice::Manager();
	algTopology topology;
	topology.SetFace(vecFaceIx);
	for (int i = 0; i < vecFirstList.size(); i++)
	{
		UINT currentIndx = vecFirstList[i];
		vector<UINT> faceVec;
		topology.GetFacesByVertex(currentIndx, faceVec);
		for (int j = 0; j < faceVec.size(); j++)
		{
			UINT uVerts[3];
			comFace* currentFace = (comFace*)(pFaceMgr->Object(faceVec[j]));
			currentFace->GetVertices(uVerts);	

			vector<UINT> vecNewIdx;
			if (uVerts[0] == currentIndx)
			{
				uVerts[0] = vecSecondList[vecFirstList.size() - 1 - i];
				vecNewIdx.push_back(vecSecondList[vecFirstList.size() - 1 - i]);
				vecNewIdx.push_back(uVerts[1]);
				vecNewIdx.push_back(uVerts[2]);
			}
			else if (uVerts[1] == currentIndx)
			{
				uVerts[1] = vecSecondList[vecFirstList.size() - 1 - i];
				vecNewIdx.push_back(vecSecondList[vecFirstList.size() - 1 - i]);
				vecNewIdx.push_back(uVerts[0]);
				vecNewIdx.push_back(uVerts[2]);
			}
			else if (uVerts[2] == currentIndx)
			{
				uVerts[2] = vecSecondList[vecFirstList.size() - 1 - i];
				vecNewIdx.push_back(vecSecondList[vecFirstList.size() - 1 - i]);
				vecNewIdx.push_back(uVerts[1]);
				vecNewIdx.push_back(uVerts[0]);
			}
			else
			{
				continue;
			}

			if (uVerts[0] == uVerts[1] || uVerts[0] == uVerts[2] || uVerts[1] == uVerts[2])
			{
				continue;
			}

			currentFace->SetVertices(uVerts);
			currentFace->SetModified(Major_Modify);

			comVertice* vtx1 = (comVertice*)(pVertMgr->Object(vecNewIdx[0]));
			double dCoord1[3];
			vtx1->Xyz(dCoord1);
			gp_Pnt pt1(dCoord1[0], dCoord1[1], dCoord1[2]);
			comVertice* vtx2 = (comVertice*)(pVertMgr->Object(vecNewIdx[1]));
			double dCoord2[3];
			vtx2->Xyz(dCoord2);
			gp_Pnt pt2(dCoord2[0], dCoord2[1], dCoord2[2]);
			comVertice* vtx3 = (comVertice*)(pVertMgr->Object(vecNewIdx[2]));
			double dCoord3[3];
			vtx3->Xyz(dCoord3);
			gp_Pnt pt3(dCoord3[0], dCoord3[1], dCoord3[2]);

			gp_Dir calculateNormalAssistDir1(pt1.X() - pt2.X(), pt1.Y() - pt2.Y(),
				pt1.Z() - pt2.Z());
			gp_Dir calculateNormalAssistDir2(pt1.X() - pt3.X(), pt1.Y() - pt3.Y(),
				pt1.Z() - pt3.Z());
			gp_Dir triNormal = calculateNormalAssistDir1.Crossed(calculateNormalAssistDir2);
			double dNormal[3] = { triNormal.X(), triNormal.Y(), triNormal.Z() };
			currentFace->SetNormal(dNormal);
		}
	}
	return true;
}

double iogBasicTools::CalculateAngleBetweenDirs(gp_Dir dir1, gp_Dir dir2)
{
	double dCosTheta = (dir1.X()*dir2.X() + dir1.Y()*dir2.Y() + dir1.Z()*dir2.Z()) /
		(sqrt(pow(dir1.X(), 2) + pow(dir1.Y(), 2) + pow(dir1.Z(), 2))*
			sqrt(pow(dir2.X(), 2) + pow(dir2.Y(), 2) + pow(dir2.Z(), 2)));

	return (180 / M_PI)*acos(dCosTheta);
}

bool iogBasicTools::FindTriangleIndex(
	std::vector<std::pair<UINT, UINT>> vecIdxPair,
	std::vector<std::vector<UINT>>& tris)
{
	for (int i = 0; i < vecIdxPair.size(); i++)
	{
		UINT index11 = vecIdxPair[i].first;
		UINT index12 = vecIdxPair[i].second;
		for (int j = 0; j < vecIdxPair.size(); j++)
		{
			if (i == j)
				continue;
			UINT index21 = vecIdxPair[j].first;
			UINT index22 = vecIdxPair[j].second;
			if (index11 == index21 && index12 != index22)
			{
				for (int k = 0; k < vecIdxPair.size(); k++)
				{
					if (i == k || j == k)
						continue;
					UINT index31 = vecIdxPair[k].first;
					UINT index32 = vecIdxPair[k].second;
					if ((index31 == index12 && index32 == index22) ||
						(index31 == index22 && index32 == index12))
					{
						std::vector<UINT> vecCurrent;
						vecCurrent.push_back(index11); 
						vecCurrent.push_back(index12);
						vecCurrent.push_back(index22);

						tris.push_back(vecCurrent);
					}
				}
			}
			/*else if (index11 == index22 && index12 != index21)
			{
				for (int k = 0; k < vecIdxPair.size(); k++)
				{
					if (i == k || j == k)
						continue;
					UINT index31 = vecIdxPair[k].first;
					UINT index32 = vecIdxPair[k].second;
					if ((index31 == index12 && index32 == index21) ||
						(index31 == index21 && index32 == index12))
					{
						std::vector<UINT> vecCurrent;
						vecCurrent.push_back(index11);
						vecCurrent.push_back(index12);
						vecCurrent.push_back(index21);

						tris.push_back(vecCurrent);
					}
				}
			}*/
		}
	}
	return true;
}

bool iogBasicTools::FilterFreeEdgeVtxIndexes(
	double dPointEqual,
	std::vector<std::pair<UINT, gp_Pnt>> vecPairVtxIndex,
	std::vector<TopoDS_Edge> vecFreeEdge,
	std::vector<std::pair<UINT, UINT>>& vecIdxPair)
{
	ShapeAnalysis_Edge sae;
	
	for (int i = 0; i < vecFreeEdge.size(); i++)
	{
		TopoDS_Edge edge = vecFreeEdge[i];
		TopoDS_Vertex v11 = sae.FirstVertex(edge);
		gp_Pnt pt11 = BRep_Tool::Pnt(v11);
		TopoDS_Vertex v12 = sae.LastVertex(edge);
		gp_Pnt pt12 = BRep_Tool::Pnt(v12);
		UINT index1, index2;
		for (int j = 0; j < vecPairVtxIndex.size(); j++)
		{
			if (vecPairVtxIndex[j].second.IsEqual(pt11, dPointEqual))
			{
				index1 = vecPairVtxIndex[j].first;
				break;
			}
		}
		for (int j = 0; j < vecPairVtxIndex.size(); j++)
		{
			if (vecPairVtxIndex[j].second.IsEqual(pt12, dPointEqual))
			{
				index2 = vecPairVtxIndex[j].first;
				break;
			}
		}
		vecIdxPair.push_back(std::pair<UINT, UINT>(index1, index2));
	}
	return true;
}

bool iogBasicTools::SplitGroupTriangles(double dPointEqual, std::vector<TopoDS_Edge>& vecEdges,
	std::vector<std::vector<TopoDS_Edge>>& vecvecGroups)
{
	if (vecEdges.size() > 200)
	{
		std::vector<TopoDS_Edge> veccurrent, vecnext;
		for (int i = 0; i < 100; i++)
		{
			veccurrent.push_back(vecEdges[i]);
		}
		for (int i = 100; i < vecEdges.size(); i++)
		{
			vecnext.push_back(vecEdges[i]);
		}
		iogOccKernelTools::GroupEdgesInTriangles(dPointEqual, veccurrent, vecvecGroups);
		SplitGroupTriangles(dPointEqual, vecnext, vecvecGroups);
	}

	return true;
}

bool iogBasicTools::FindMergePairs(TopoDS_Shape entryshape,
	std::vector<TopoDS_Edge> vecEdges, std::vector<std::pair<UINT, UINT>>& vecnearestpt)
{
	std::vector<gp_Pnt> vecReorderedpts;
	ShapeAnalysis_Edge anEdgeAnalyser;
	std::vector<gp_Pnt> vecpt1, vecpt2;
	ShapeAnalysis_WireOrder aWireOrder;
	for (int i = 0; i < vecEdges.size(); i++)
	{
		TopoDS_Vertex aVf = anEdgeAnalyser.FirstVertex(vecEdges[i]);
		TopoDS_Vertex aVl = anEdgeAnalyser.LastVertex(vecEdges[i]);
		gp_Pnt aPf = BRep_Tool::Pnt(aVf);
		gp_Pnt aPl = BRep_Tool::Pnt(aVl);
		aWireOrder.Add(aPf.XYZ(), aPl.XYZ());
	}
	for (int i = 1; i <= aWireOrder.NbEdges(); i++)
	{
		TopoDS_Edge anEdge = vecEdges.at(i - 1);
		TopoDS_Vertex aVf = anEdgeAnalyser.FirstVertex(vecEdges[i]);
		gp_Pnt aPf = BRep_Tool::Pnt(aVf);
		vecReorderedpts.push_back(aPf);
	}

	//find apex
	std::vector<int> vecapex;
	int nReorder = vecReorderedpts.size();
	if ( nReorder < 3)
	{
		return false;
	}
	gp_Dir dirfst = DirOf2Pnts(vecReorderedpts[0], vecReorderedpts[nReorder - 1]);
	gp_Dir dirlst = DirOf2Pnts(vecReorderedpts[0], vecReorderedpts[1]);
	if (dirfst.Angle(dirlst) < M_PI / 2)
	{
		vecapex.push_back(0);
	}
	for (int i = 1; i < nReorder - 1; i++)
	{
		gp_Dir xdirfst = DirOf2Pnts(vecReorderedpts[i], vecReorderedpts[i - 1]);
		gp_Dir xdirlst = DirOf2Pnts(vecReorderedpts[i], vecReorderedpts[i + 1]);
		if (xdirfst.Angle(xdirlst) < M_PI / 2)
		{
			vecapex.push_back(i);
		}
	}

	return true;
}

gp_Dir iogBasicTools::DirOf2Pnts(gp_Pnt pt1, gp_Pnt pt2)
{
	gp_Dir dir(pt2.X() - pt1.X(), pt2.Y() - pt1.Y(), pt2.Z() - pt1.Z());
	return dir;
}

bool iogBasicTools::AddPntToVec(gp_Pnt pt1, std::vector<gp_Pnt> &vecPnt)
{
	bool ifadd1 = true;
	for (int j = 0; j < vecPnt.size(); j++)
	{
		if (pt1.Distance(vecPnt[j]) < 0.01)
		{
			ifadd1 = false;
			break;
		}
	}
	if (ifadd1)
		vecPnt.push_back(pt1);
	return true;
}

bool iogBasicTools::GridScale(bool firstloop, UINT nMaxPartIx, UINT nMaxFaceIx, UINT uMaxSurfIx, double dScale)
{
	//if uniformly scale,the normal of one face didn't change
	//else if ununiformly scale,the normal of one face changed
	//use below formula to calculate new face normal:
	//Op:local coordinte system origin coordinates in global coordinate system
	//v0:point 0   v1:point 1    v2:point 2
	//n0=(v1-v0)x(v2-v0);
	//v01:after scale point0   v11:after scale point 1    v21:after scale point 2
	//M:global to local coordinate system translation matrix(Global->Local)
	//S:scale matrix used in selected coordinate system(in local coordinate system)
	//v01=Op+[(M*S*M(-1))*(v0-Op)]
	//n01=[M*S*M(-1)*(v1-v0)]x[M*S*M(-1)*(v2-v0)]=([M*S(-1)*M(-1)])T*(n0)

	basePoint oriPoint(0,0,0);
	baseVector vecAxis(1, 0, 0);
	baseVector vecAxis1(0, 1, 0);
	baseCoordSys crdsysScale(oriPoint, vecAxis, vecAxis1);

	basePoint bpntCrsSysOri = crdsysScale.GetOrigin();
	baseVector bvecCrdSysOri(bpntCrsSysOri);

	baseMatrix3x3 bmatLocal2Global = crdsysScale.Axis();//M
	baseMatrix3x3 bmatGlobal2Local = bmatLocal2Global.Inversion();//M-1
	double dScaleArray[9]{ dScale,0,0,0,dScale,0,0,0,dScale };
	baseMatrix3x3 bmatScale(dScaleArray);

	baseMatrix3x3 bmatCoordinate = (bmatLocal2Global.MultiplyMatrix(bmatScale)).MultiplyMatrix(bmatGlobal2Local);//[M*S*M(-1)]

	comSurfaceMgr * pSurfMgr = comSurface::Manager();
	if (!pSurfMgr)
	{
		return false;
	}

	vector<UINT> vecSurfIx, vecUsedSurfIx;
	pSurfMgr->GetAllObject(vecSurfIx);

	for (UINT i = 0; i < vecSurfIx.size(); i++)
	{
		if (!firstloop)
		{
			if (i < uMaxSurfIx + 1)
			{
				continue;
			}
		}

		vecUsedSurfIx.push_back(i);
	}

	comVerticeMgr * pVerticeMgr = comVertice::Manager();
	if (!pVerticeMgr)
	{
		return false;
	}

	comPartMgr * pPartMgr = comPart::Manager();
	if (!pPartMgr)
	{
		return false;
	}

	vector<UINT> vecParts, vecAllParts;
	pPartMgr->GetAllObject(vecAllParts);
	for (UINT i = 0; i < vecAllParts.size(); i++)
	{
		if (!firstloop)
		{
			if (i < nMaxPartIx + 1)
			{
				continue;
			}
		}

		vecParts.push_back(i);
	}

	int nPartNum = vecParts.size();
	if (nPartNum == 0)
	{
		return false;
	}

	comFaceMgr * pFaceMgr = comFace::Manager();
	if (!pFaceMgr)
	{
		return false;
	}

	vector<UINT> vecFaces, vecAllFaces;
	pFaceMgr->GetAllObject(vecAllFaces);
	for (UINT i = 0; i < vecAllFaces.size(); i++)
	{
		if (!firstloop)
		{
			if (i < nMaxFaceIx + 1)
			{
				continue;
			}
		}

		vecFaces.push_back(i);
	}

	size_t tFaces = vecFaces.size();
	vector<UINT> vecVerts(nThree * tFaces);
	size_t tVertNum = 0;

	for (size_t i = 0; i < tFaces; i++)
	{
		UINT uFace = vecFaces[i];
		comFace *pFace = (comFace *)(pFaceMgr->Object(uFace));
		if (!pFace)
		{
			return false;
		}

		UINT uPart = TheFunc.GetPartBySurface(pFace->Surface());

		vector<UINT>::const_iterator itPart = std::find(vecParts.begin(), vecParts.end(), uPart);
		if (itPart != vecParts.end())
		{
			UINT uVertIdx[nThree];
			pFace->GetVertices(uVertIdx);

			vecVerts[tVertNum] = uVertIdx[0];
			vecVerts[tVertNum + 1] = uVertIdx[1];
			vecVerts[tVertNum + 2] = uVertIdx[2];
			tVertNum += nThree;
		}
	}

	vecVerts.resize(tVertNum);
	sort(vecVerts.begin(), vecVerts.end());
	vecVerts.erase(unique(vecVerts.begin(), vecVerts.end()), vecVerts.end());

	size_t tVerts = vecVerts.size();
	for (size_t i = 0; i < tVerts; i++)
	{
		UINT uVert = vecVerts[i];
		comVertice * pVert = (comVertice *)(pVerticeMgr->Object(uVert));
		if (!pVert)
		{
			return false;
		}

		double dCrds[nThree];
		pVert->Xyz(dCrds);

		baseVector bvecVert(dCrds);
		baseVector bvecLocal = bvecVert - bvecCrdSysOri;
		baseVector bvecNewVert = bvecCrdSysOri + bmatCoordinate.MultiplyVector(bvecLocal);

		dCrds[0] = bvecNewVert.X();
		dCrds[1] = bvecNewVert.Y();
		dCrds[2] = bvecNewVert.Z();

		pVert->SetXyz(dCrds);
	}

	size_t tParts = vecParts.size();
	for (int i = 0; i < tParts; i++)
	{
		UINT uPart = vecParts[i];
		comPart * pPart = (comPart *)(pPartMgr->Object(uPart));
		if (!pPart)
		{
			return false;
		}
		//pPart->SetModified(Major_Modify);
	}

	for (size_t i = 0; i < tFaces; i++)
	{
		UINT uFace = vecFaces[i];
		comFace * pFace = (comFace *)(pFaceMgr->Object(uFace));
		if (!pFace)
		{
			return false;
		}

		UINT uVerts[nThree];
		pFace->GetVertices(uVerts);

		double dCrd1[nThree];
		double dCrd2[nThree];
		double dCrd3[nThree];

		comVertice * pVert1 = (comVertice *)(pVerticeMgr->Object(uVerts[0]));
		if (!pVert1)
		{
			return false;
		}

		comVertice * pVert2 = (comVertice *)(pVerticeMgr->Object(uVerts[1]));
		if (!pVert2)
		{
			return false;
		}

		comVertice * pVert3 = (comVertice *)(pVerticeMgr->Object(uVerts[2]));
		if (!pVert3)
		{
			return false;
		}

		pVert1->Xyz(dCrd1);
		pVert2->Xyz(dCrd2);
		pVert3->Xyz(dCrd3);

		baseVector pVec1(dCrd2[0] - dCrd1[0], dCrd2[1] - dCrd1[1], dCrd2[2] - dCrd1[2]);
		baseVector pVec2(dCrd3[0] - dCrd1[0], dCrd3[1] - dCrd1[1], dCrd3[2] - dCrd1[2]);
		baseVector pNormal = pVec1.CrossProduct(pVec2);
		pNormal.Normalize();

		double dNormal[nThree]{ pNormal.X(),pNormal.Y(),pNormal.Z() };
		pFace->SetNormal(dNormal);
	}

	set<UINT> setSurfBdy;
	size_t tSurfs = vecUsedSurfIx.size();
	for (size_t i = 0; i < tSurfs; i++)
	{
		comSurface * pSurface = (comSurface *)(pSurfMgr->Object(vecUsedSurfIx[i]));
		if (!pSurface)
		{
			return false;
		}

		UINT uSurfBdy = pSurface->Boundary();
		setSurfBdy.insert(uSurfBdy);
	}
	return true;
}

bool iogBasicTools::Pnt2Inx(gp_Pnt pnt, QHash<QString, UINT>& hashVertIdx, comVerticeMgr * pVertMgr,
	int& nBeginVtx, UINT nVert, UINT& uVerts)
{
	QString strX = QString::number(pnt.X(),'f', mergeTolerance);
	QString strY = QString::number(pnt.Y(), 'f', mergeTolerance);
	QString strZ = QString::number(pnt.Z(), 'f', mergeTolerance);
	strX.truncate(strX.length() - 2);
	strY.truncate(strY.length() - 2);
	strZ.truncate(strZ.length() - 2);
	QString strCurrent = strX + "," + strY + "," + strZ + ",";

	//if brand new point
	if (!hashVertIdx.contains(strCurrent))
	{
		double dCrds1[3] = { pnt.X(), pnt.Y(), pnt.Z() };
		comVertice * pVertice1 = new comVertice(dCrds1);
		if (!pVertice1)
		{
			return false;
		}
		pVertMgr->Create(pVertice1);
		nBeginVtx++;
		hashVertIdx.insert(strCurrent, nVert + nBeginVtx);	
	}
	//else
	//{
	//	//maybe is different
	//	bool bIsDiff = true;
	//	for (int i = 0; i < vecPointPos.size(); i++)
	//	{
	//		std::vector<double> vecPos = vecPointPos[i];
	//		if (vecPos[0] == pnt.X() &&
	//			vecPos[1] == pnt.Y() &&
	//			vecPos[2] == pnt.Z())
	//		{
	//			bIsDiff = false;
	//			break;
	//		}
	//	}
	//	if (bIsDiff)
	//	{
	//		double dCrds1[3] = { pnt.X(), pnt.Y(), pnt.Z() };
	//		comVertice * pVertice1 = new comVertice(dCrds1);
	//		if (!pVertice1)
	//		{
	//			return false;
	//		}
	//		pVertMgr->Create(pVertice1);
	//		nBeginVtx++;
	//		uVerts = nVert + nBeginVtx;
	//	}
	//	else
	//	{
	//		uVerts = hashVertIdx.value(dSymble);
	//	}
	//}
	uVerts = hashVertIdx.value(strCurrent);
	return true;
}

bool iogBasicTools::GetClosedVertexIdxGroup(vector<vector<pair<basePoint, basePoint>>> inputEdges,
	QHash<QString, UINT> hashVertIdx,
	vector<vector<UINT>>& groupedIndex)
{
	QHash<UINT, UINT> hashEdge;
	vector<UINT> allIndx;
	vector<pair<UINT, UINT>> vecEdgeIndx;
	QMultiHash<UINT, UINT> hashEdgeIndx;

	//
	/*ofstream datafile;
	datafile.open("D:/data.txt", ios::out|ios::app);
	QHash<QString, UINT>::ConstIterator it = hashVertIdx.constBegin();
	while (it!= hashVertIdx.constEnd())
	{
		datafile << it.key().toStdString() << "\n";
	}
	datafile.close();*/
	//
	for (int i = 0; i < inputEdges.size(); i++)
	{
		vector<pair<basePoint, basePoint>> currentvec = inputEdges[i];
		for (int j = 0; j < currentvec.size(); j++)
		{
			pair<basePoint, basePoint> pairbase = currentvec[j];
			basePoint bp1 = pairbase.first;
			basePoint bp2 = pairbase.second;

			UINT ui1, ui2;

			QString strX1 = QString::number(bp1.X(), 'f', mergeTolerance);
			QString strY1 = QString::number(bp1.Y(), 'f', mergeTolerance);
			QString strZ1 = QString::number(bp1.Z(), 'f', mergeTolerance);
			strX1.truncate(strX1.length() - 2);
			strY1.truncate(strY1.length() - 2);
			strZ1.truncate(strZ1.length() - 2);
			QString strCurrent1 = strX1 + "," + strY1 + "," + strZ1 + ",";
			if (!hashVertIdx.contains(strCurrent1))
				continue;
			ui1 = hashVertIdx.value(strCurrent1);

			QString strX2 = QString::number(bp2.X(), 'f', mergeTolerance);
			QString strY2 = QString::number(bp2.Y(), 'f', mergeTolerance);
			QString strZ2 = QString::number(bp2.Z(), 'f', mergeTolerance);
			strX2.truncate(strX2.length() - 2);
			strY2.truncate(strY2.length() - 2);
			strZ2.truncate(strZ2.length() - 2);
			QString strCurrent2 = strX2 + "," + strY2 + "," + strZ2 + ",";
			if (!hashVertIdx.contains(strCurrent2))
				continue;
			ui2 = hashVertIdx.value(strCurrent2);

			bool ifadd1 = true, ifadd2 = true;
			for (int k = 0; k < allIndx.size(); k++)
			{
				if (allIndx[k] == ui1)
				{
					ifadd1 = false;
					break;
				}
			}
			for (int k = 0; k < allIndx.size(); k++)
			{
				if (allIndx[k] == ui2)
				{
					ifadd2 = false;
					break;
				}
			}

			if (ifadd1)
				allIndx.push_back(ui1);
			if (ifadd2)
				allIndx.push_back(ui2);

			vecEdgeIndx.push_back(pair<UINT, UINT>(ui1, ui2));

			hashEdgeIndx.insert(ui1, ui2);
		}
	}
	
	if (vecEdgeIndx.size() == 0)
		return false;
	
	vector<int> vecUsedIdx;

	//graph methode
	comVerticeMgr * pVertMgr = comVertice::Manager();
	int nAllVtx = pVertMgr->MaxIndex();
	vector<vector<UINT>> vompareVec = FindConnectedEdges(nAllVtx, vecEdgeIndx);
	groupedIndex = vompareVec;
	
	//my method
	//for (int i = 0; i < vecEdgeIndx.size(); i++)
	//{
	//	TheExe.ProgressSetValue(100* i / vecEdgeIndx.size());
	//	pair<UINT, UINT> startEdge = vecEdgeIndx[i];
	//	vecUsedIdx.push_back(i);
	//	vector<UINT> subgroup;
	//	UINT startFirst = startEdge.first;
	//	UINT startLast = startEdge.second;
	//	subgroup.push_back(startFirst);
	//	subgroup.push_back(startLast);
	//	//LoopClosedFreeEdges(startLast, vecEdgeIndx, subgroup, vecUsedIdx);
	//	hashEdgeIndx.remove(startFirst, startLast);
	//	LoopClosedFreeEdges(startLast, hashEdgeIndx, subgroup);
	//	int nlast = subgroup.size() - 1;
	//	if (subgroup[0] == subgroup[nlast])
	//		groupedIndex.push_back(subgroup);
	//}
	
	/*if (allIndx.size() == 0)
		return false;

	vector<UINT> usedgroup;
	for (int i = 0; i < allIndx.size(); i++)
	{
		vector<UINT> subgroup;
		UINT ubegin = allIndx[i];
		bool ifContinue = false;
		for (int j = 0; j < usedgroup.size(); j++)
		{
			if (ubegin == usedgroup[j])
				ifContinue = true;
		}
		if (ifContinue)
			continue;
		if (hashEdge.contains(ubegin))
		{
			UINT unext = hashEdge.value(ubegin);
			subgroup.push_back(ubegin);
			usedgroup.push_back(ubegin);
			if (LoopFindClosedFreeEdges(subgroup, usedgroup, allIndx, unext, hashEdge))
				groupedIndex.push_back(subgroup);
		}
	}*/

}

bool iogBasicTools::TurnClosedEdgePairToOrderedIndex(vector<pair<UINT, UINT>> vecEdgeIndx,
	vector<UINT>& subgroup)
{
	vector<int> vecUsedIdx;

	pair<UINT, UINT> startEdge = vecEdgeIndx[0];
	vecUsedIdx.push_back(0);
	UINT startFirst = startEdge.first;
	UINT startLast = startEdge.second;
	subgroup.push_back(startFirst);
	subgroup.push_back(startLast);
	LoopClosedFreeEdges(startLast, vecEdgeIndx, subgroup, vecUsedIdx);

	return true;
}

bool iogBasicTools::LoopClosedFreeEdges(UINT& startIdx, 
	vector<pair<UINT, UINT>> vecEdgeIndx,
	vector<UINT>& subgroup,
	vector<int>& vecUsedIdx)
{
	for (int i = 0; i < vecEdgeIndx.size(); i++)
	{
		bool bIfContinue = false;
		for (int j = 0; j < vecUsedIdx.size(); j++)
		{
			if (i == vecUsedIdx[j])
			{
				bIfContinue = true;
				break;
			}
		}
		if (bIfContinue)
			continue;

		pair<UINT, UINT> currentEdge = vecEdgeIndx[i];
		
		UINT nextFirst = currentEdge.first;
		UINT nextLast = currentEdge.second;
		if (startIdx == nextFirst)
		{
			vecUsedIdx.push_back(i);
			subgroup.push_back(nextLast);
			LoopClosedFreeEdges(nextLast, vecEdgeIndx, subgroup, vecUsedIdx);
		}
		else if (startIdx == nextLast)
		{
			vecUsedIdx.push_back(i);
			subgroup.push_back(nextFirst);
			LoopClosedFreeEdges(nextFirst, vecEdgeIndx, subgroup, vecUsedIdx);
		}
		else
			continue;
	}
	return true;
}

bool iogBasicTools::LoopClosedFreeEdges(UINT& startIdx,
	QMultiHash<UINT, UINT> hashEdgeIndx,
	vector<UINT>& subgroup)
{
	if (hashEdgeIndx.contains(startIdx))
	{
		UINT nNext = hashEdgeIndx.value(startIdx);
		hashEdgeIndx.remove(startIdx, nNext);
		subgroup.push_back(nNext);
		LoopClosedFreeEdges(nNext, hashEdgeIndx, subgroup);
	}
	else
	{
		UINT nNext = hashEdgeIndx.key(startIdx);
		if (!hashEdgeIndx.contains(nNext))
			return false;
		hashEdgeIndx.remove(nNext, startIdx);
		subgroup.push_back(nNext);
		LoopClosedFreeEdges(nNext, hashEdgeIndx, subgroup);
	}
	return true;
}

bool iogBasicTools::LoopFindClosedFreeEdges(vector<UINT> &subgroup, vector<UINT>& usedgroup, vector<UINT> allIndx, UINT beginIndx,
	QHash<UINT, UINT> hashEdge)
{
	UINT ubegin = beginIndx;
	
	for (int j = 0; j < subgroup.size(); j++)
	{
		if (ubegin == subgroup[j])
			return false;
	}

	if (hashEdge.contains(ubegin))
	{
		UINT unext = hashEdge.value(ubegin);
		subgroup.push_back(unext);
		usedgroup.push_back(unext);

		if (unext == subgroup[0])
			return true;
		else
			LoopFindClosedFreeEdges(subgroup, usedgroup, allIndx, unext, hashEdge);
	}
	return false;
}

bool iogBasicTools::OffsetSelectedSurface(double dDistance, vector<UINT> vecSelectedSurface)
{
	comVerticeMgr * pVertMgr = comVertice::Manager();
	comFaceMgr * pFaceMgr = comFace::Manager();
	vector<UINT> vecVertex;
	
	vector<UINT> vecFaceIx;
	TheFunc.GetSurfaceFaces(vecSelectedSurface, vecFaceIx);
	algTopology topology;
	topology.SetFace(vecFaceIx);

	GetVertexBySurfaces(vecSelectedSurface, vecVertex);
	if (vecVertex.size() == 0)
		return false;

	for (int i = 0; i < vecVertex.size(); i++)
	{
		comVertice* pVertice1 = (comVertice*)pVertMgr->Object(vecVertex[i]);
		vector<UINT> faceVec;
		topology.GetFacesByVertex(vecVertex[i], faceVec);

		double dOldCrd[3];
		pVertice1->Xyz(dOldCrd);

		//calculate direction
		gp_Dir dirDirection(0, 0, 0);
		for (int j = 0; j < faceVec.size(); j++)
		{
			double dNormal[3];
			comFace* currentFace = (comFace*)(pFaceMgr->Object(faceVec[j]));
			currentFace->Normal(dNormal);

			dirDirection.SetX(dirDirection.X() + dNormal[0]);
			dirDirection.SetX(dirDirection.Y() + dNormal[1]);
			dirDirection.SetX(dirDirection.Z() + dNormal[2]);
		}

		//apply distance
		double dNewCrd[3];
		dNewCrd[0] = dOldCrd[0] + dirDirection.X() * dDistance;
		dNewCrd[1] = dOldCrd[1] + dirDirection.Y() * dDistance;
		dNewCrd[2] = dOldCrd[2] + dirDirection.Z() * dDistance;
		pVertice1->SetXyz(dNewCrd);
		pVertice1->SetModified(Major_Modify);
	}
}

void iogBasicTools::GetVertexByFaces(const vector<UINT> & vecFaceIx, std::vector<UINT>  & vecVertex)
{
	vecVertex.clear();

	comFaceMgr * pFaceMgr = comFace::Manager();
	if (!pFaceMgr)
	{
		return;
	}

	size_t tFaces = vecFaceIx.size();
	if (tFaces == 0)
	{
		return;
	}

	vecVertex.resize(3 * tFaces);

	for (size_t i = 0; i < tFaces; ++i)
	{
		UINT uFace = vecFaceIx[i];
		comFace * pFace = (comFace *)(pFaceMgr->Object(uFace));
		if (!pFace)
		{
			return;
		}

		UINT uVerts[3];
		pFace->GetVertices(uVerts);

		vecVertex[i * 3] = uVerts[0];
		vecVertex[i * 3 + 1] = uVerts[1];
		vecVertex[i * 3 + 2] = uVerts[2];
	}

	std::sort(vecVertex.begin(), vecVertex.end());
	vecVertex.erase(unique(vecVertex.begin(), vecVertex.end()), vecVertex.end());

	return;
}

void iogBasicTools::GetVertexBySurfaces(const vector<UINT> & vecSurfaceIx, std::vector<UINT>  & vecVertex)
{
	vecVertex.clear();
	vector<UINT> vecFaceIx;
	TheFunc.GetSurfaceFaces(vecSurfaceIx, vecFaceIx);
	GetVertexByFaces(vecFaceIx, vecVertex);
}

vector<vector<UINT>> iogBasicTools::FindConnectedEdges(int nNum,
	vector<pair<UINT, UINT>>& edges)
{
	// 构建邻接表
	vector<vector<UINT>> graph(nNum);
	for (auto edge : edges)
	{
		UINT u = edge.first;
		UINT v = edge.second;
		graph[u].push_back(v);
		graph[v].push_back(u);
	}

	// 找到所有连通分量
	vector<bool> visited(nNum, false);
	vector<vector<UINT>> components;
	for (int node = 0; node < nNum; node++)
	{
		if (!visited[node]) 
		{
			vector<UINT> component;
			dfs(node, graph, visited, component);
			components.push_back(component);
		}
	}

	// 将连通分量转化为边集合
	vector<vector<pair<UINT, UINT>>> connected_edges;
	vector<vector<UINT>> vecFinal;
	for (auto component : components) 
	{
		if (component.size() > 1) 
		{
			unordered_set<UINT> nodes(component.begin(), component.end());
			vector<pair<UINT, UINT>> edges_in_component;
			for (auto edge : edges) 
			{
				UINT u = edge.first;
				UINT v = edge.second;
				if (nodes.count(u) && nodes.count(v))
				{
					edges_in_component.push_back(edge);
				}
			}
			connected_edges.push_back(edges_in_component);
		}
	}

	for (int i = 0; i < connected_edges.size(); i++)
	{
		vector<pair<UINT, UINT>> vecSub = connected_edges[i];
		vector<UINT> vecInside;

		if (vecSub.size() != 0)
		{
			TurnClosedEdgePairToOrderedIndex(vecSub, vecInside);
			vecFinal.push_back(vecInside);
		}	
	}

	return vecFinal;
}

void iogBasicTools::dfs(int node, const vector<vector<UINT>>& graph, vector<bool>& visited, vector<UINT>& component)
{
	visited[node] = true;
	component.push_back(node);
	for (int neighbor : graph[node]) 
	{
		if (!visited[neighbor]) 
		{
			dfs(neighbor, graph, visited, component);
		}
	}
}

//////////////////////////////////////////////////////////////////
//