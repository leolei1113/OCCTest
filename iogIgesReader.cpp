//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Xia Lei
//   Date   : 2022/09/05
//
//=============================================================================
#include "core.h"
#include "base.h"
#include "comobj.h"
#include "appinterface.h"
#include "qui.h"
#include "config.h"

#include "iogBinaryTree.h"
#include "iogIgesReader.h"
#include "iogOccKernelTools.h"

#define Mesh_Linear_Defl 0.5
#define Sew_Tole 0.1
#define mergeTolerance 9      

#define Igs_Import_MaxHoleArea   "iog_param/parameter/import_igs/holearea"
#define Igs_Import_Unit  "iog_param/parameter/import_igs/unit"
#define Igs_Import_UnitType   "iog_param/parameter/import_igs/unittype"
#define Igs_Import_DiscreType "iog_param/parameter/import_igs/discretype"

#define Igs_Import_DiscreLinear   "iog_param/parameter/import_igs/discretlinear"
#define Igs_Import_DiscreAngle   "iog_param/parameter/import_igs/discretangle"
#define Igs_Import_SewingTolerance   "iog_param/parameter/import_igs/sewingtolerance"
//////////////////////////////////////////////////////////////////
//

iogIgesReader::iogIgesReader(const char * pFilename)
{
	m_pFileName = _strdup(pFilename);
}

iogIgesReader::~iogIgesReader()
{
	free(m_pFileName);
}

//////////////////////////////////////////////////////////////////
//

bool iogIgesReader::ReadIGES()
{
	iogBasicTools iobasictools;
	char pName[128];
	iobasictools.GetPartName(pName, m_pFileName);

	string strImportinfo, strImport, strFilename;
	strImport = "Import ";
	strFilename = pName;
	strImportinfo = strImport + strFilename + ".iges";

	TheExe.ProgressInit(strImportinfo.c_str());
	TheExe.ProgressSetValue(0);

	QString strMsg;

	GetXmlValueD(Igs_Import_MaxHoleArea, m_dHoleRad);
	m_dHoleRad = abs(m_dHoleRad);

	GetXmlValueD(Igs_Import_Unit, m_nUnit); 
	GetXmlValueN(Igs_Import_DiscreType, m_nDiscreType);

	GetXmlValueD(Igs_Import_DiscreLinear, m_dLinearDelVal);
	GetXmlValueD(Igs_Import_DiscreAngle, m_dAngleDelVal);
	GetXmlValueD(Igs_Import_SewingTolerance, m_dSewingTol);

	if (m_dLinearDelVal == 0 || m_dAngleDelVal == 0)
	{
		QString strMsgError = QObject::tr("Linear Deflection and Angle Deflection can not be 0, please check.");
		TheExe.PrintMessage(strMsgError, apiExe::MSG_NORMAL);
		TheExe.ProgressExit();
		return false;
	}

	if (!GetIGESFile())
	{
		strMsg = QObject::tr("The file %1 was failed import").arg(m_pFileName);
		TheExe.PrintMessage(strMsg, apiExe::MSG_ERROR);
		return false;
	}
	else
	{
		strMsg = QObject::tr("The file %1 was successful import").arg(m_pFileName);
		TheExe.PrintMessage(strMsg, apiExe::MSG_NORMAL);
		std::string strFilePath = m_pFileName;
		ConfigAddRecentFile(strFilePath);
	}
	return true;
}

bool iogIgesReader::GetIGESFile()
{
	DWORD t1 = GetTickCount();
	Standard_CString path = (Standard_CString)m_pFileName;

	TopoDS_Shape igesshape;
	IGESCAFControl_Reader reader;
	
	IFSelect_ReturnStatus status = reader.ReadFile(path);
	if (status != IFSelect_RetDone)
	{
		return false;
	}

	if (status == IFSelect_RetDone)
	{
		reader.TransferRoots();
		igesshape = reader.OneShape();
	}

	if (igesshape.IsNull())
	{
		// Collecting resulting entities
		Standard_Integer nbs = reader.NbShapes();
		BRep_Builder builder;
		TopoDS_Compound cmp;
		builder.MakeCompound(cmp);
		for (Standard_Integer i = 1; i <= nbs; i++)
		{
			builder.Add(cmp, reader.Shape(i));
		}
		igesshape = cmp;
	}

	//map faces
	std::vector<TopoDS_Shape> vecFaces;
	TopTools_ListOfShape faceList;
	TopTools_IndexedMapOfShape aShapeMap;
	TopExp::MapShapes(igesshape, TopAbs_FACE, aShapeMap);

	for (auto i = aShapeMap.cbegin(); i != aShapeMap.cend(); i++)
	{
		faceList.Append(*i);
	}
	for (auto iter : faceList)
	{
		vecFaces.push_back(iter);
	}

	TheExe.ProgressSetValue(30);
	
	//retrieve all faces
	/*std::vector<TopoDS_Shape> vecAllFace;
	TopExp_Explorer facesex(igesshape, TopAbs_FACE);
	for (; facesex.More(); facesex.Next())
	{
		vecAllFace.push_back(facesex.Current());
	}*/

	std::vector<std::pair<TopoDS_Shape, std::string>> vecnamemap, vecFaceNameMap;
	
	Handle(TDocStd_Document) doc;
	Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
	anApp->NewDocument("BinOcaf", doc);
	reader.Transfer(doc);

	/*BinDrivers::DefineFormat(anApp);
	doc->ChangeStorageFormat("BinOcaf");
	TCollection_ExtendedString TPath = "d:\\text.cbf";
	anApp->SaveAs(doc, TPath);*/

	//namemap method1
	/*Handle(XCAFDoc_ShapeTool) assembly = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
	TDF_LabelSequence freeshapes;
	assembly->GetFreeShapes(freeshapes);
	for (int i = 1; i <= freeshapes.Length(); i++)
	{
		TDF_Label freeshape = freeshapes.Value(i);
		Handle(TDataStd_Name) name;
		TCollection_ExtendedString nametext;
		if (freeshape.FindAttribute(TDataStd_Name::GetID(), name))
		{
			nametext = name->Get();
			std::string s1 = TCollection_AsciiString(nametext).ToCString();

			TopoDS_Shape compareShape = assembly->GetShape(freeshape);
			vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
		}
	}*/

	//namemap method3
	Handle(XCAFDoc_ShapeTool) assembly = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
	TDF_LabelSequence freeshapes;
	assembly->GetFreeShapes(freeshapes);
	for (int i = 1; i <= freeshapes.Length(); i++)
	{
		TDF_Label freeshape = freeshapes.Value(i);
		Handle(TDataStd_Name) name;
		TCollection_ExtendedString nametext;

		if (freeshape.FindAttribute(TDataStd_Name::GetID(), name))
		{
			nametext = name->Get();
			std::string s1 = TCollection_AsciiString(nametext).ToCString();

			TopoDS_Shape compareShape = assembly->GetShape(freeshape);
			if (compareShape.ShapeType() != TopAbs_COMPOUND || compareShape.ShapeType() != TopAbs_SOLID)
				continue;

			vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
		}

		iogOccKernelTools::GetShapeNameMap(assembly, freeshape, vecnamemap);
	}

	//retrive namemap method 2
	QHash<QString, std::string> hashFace;
	std::vector<string> vecAllFaceName, vecUniqName;
	std::string strSewed = "Sewed Surface";
	vecUniqName.push_back(strSewed);

	Handle(XSControl_TransferReader) treader = reader.WS()->TransferReader();
	for (int i = 0; i < vecFaces.size(); i++)
	{
		Handle(Standard_Transient) transient = treader->EntityFromShapeResult(vecFaces[i], -1);
		Handle(IGESData_IGESEntity) item = Handle(IGESData_IGESEntity)::DownCast(transient);
		if (item)
		{
			TopoDS_Shape compareShape = vecFaces[i];
			Handle(TCollection_HAsciiString) hascii = item->NameValue();
			if (!hascii)
				continue;
			TCollection_AsciiString ascii = hascii->String();
			std::string s1 = ascii.ToCString();
			vecFaceNameMap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
			vecAllFaceName.push_back(s1);
			
			QString qstrCompare1;
			{
				BRepAdaptor_Surface adp_sur(TopoDS::Face(compareShape));
				double ustart = adp_sur.FirstUParameter();
				double uend = adp_sur.LastUParameter();
				double vstart = adp_sur.FirstVParameter();
				double vend = adp_sur.LastVParameter();
				gp_Pnt pntUsVsSurf = adp_sur.Value(ustart, vstart);
				double dXPntUsVsSurf = pntUsVsSurf.X();
				double dYPntUsVsSurf = pntUsVsSurf.Y();
				double dZPntUsVsSurf = pntUsVsSurf.Z();
				QString qstr1 = QString::number(dXPntUsVsSurf, 'f', 5) + ","
					+ QString::number(dYPntUsVsSurf, 'f', 5) + ","
					+ QString::number(dZPntUsVsSurf, 'f', 5) + ",";
				gp_Pnt pntUeVsSurf = adp_sur.Value(uend, vstart);
				double dXPntUeVsSurf = pntUeVsSurf.X();
				double dYPntUeVsSurf = pntUeVsSurf.Y();
				double dZPntUeVsSurf = pntUeVsSurf.Z();
				QString qstr3 = QString::number(dXPntUeVsSurf, 'f', 5) + ","
					+ QString::number(dYPntUeVsSurf, 'f', 5) + ","
					+ QString::number(dZPntUeVsSurf, 'f', 5) + ",";
				gp_Pnt pntUsVeSurf = adp_sur.Value(ustart, vend);
				double dXPntUsVeSurf = pntUsVeSurf.X();
				double dYPntUsVeSurf = pntUsVeSurf.Y();
				double dZPntUsVeSurf = pntUsVeSurf.Z();
				QString qstr5 = QString::number(dXPntUsVeSurf, 'f', 5) + ","
					+ QString::number(dYPntUsVeSurf, 'f', 5) + ","
					+ QString::number(dZPntUsVeSurf, 'f', 5) + ",";
				gp_Pnt pntUeVeSurf = adp_sur.Value(vend, vend);
				double dXPntUeVeSurf = pntUeVeSurf.X();
				double dYPntUeVeSurf = pntUeVeSurf.Y();
				double dZPntUeVeSurf = pntUeVeSurf.Z();
				QString qstr7 = QString::number(dXPntUeVeSurf, 'f', 5) + ","
					+ QString::number(dYPntUeVeSurf, 'f', 5) + ","
					+ QString::number(dZPntUeVeSurf, 'f', 5) + ",";
				gp_Pnt pntUfVfSurf = adp_sur.Value(0.5 * (ustart + uend), 0.5 * (ustart + uend));
				double dXPntUfVfSurf = pntUfVfSurf.X();
				double dYPntUfVfSurf = pntUfVfSurf.Y();
				double dZPntUfVfSurf = pntUfVfSurf.Z();
				QString qstr9 = QString::number(dXPntUfVfSurf, 'f', 5) + ","
					+ QString::number(dYPntUfVfSurf, 'f', 5) + ","
					+ QString::number(dZPntUfVfSurf, 'f', 5) + ",";

				qstrCompare1 = qstr1 + qstr3 + qstr5 + qstr7 + qstr9; //qstr1 + qstr3 + qstr5 + qstr7 + qstr9
			}

			hashFace.insert(qstrCompare1, s1);
		}	
	}

	for (int i = 0; i < vecAllFaceName.size(); i++)
	{
		bool bIfAdd = true;
		std::string strCompair1 = vecAllFaceName[i];
		for (int j = 0; j < vecUniqName.size(); j++)
		{
			std::string strCompair2 = vecUniqName[j];
			if (strCompair1 == strCompair2)
			{
				bIfAdd = false;
				break;
			}
		}
		if (bIfAdd)
		{
			vecUniqName.push_back(strCompair1);
		}
	}

	//test free edges
	/*TopoDS_Compound compound;
	BRep_Builder b;
	b.MakeCompound(compound);
	TopExp_Explorer shellex(igesshape, TopAbs_SHELL);
	for (; shellex.More(); shellex.Next())
	{
		TopoDS_Shell theShell = TopoDS::Shell(shellex.Current());
		ShapeAnalysis_Shell aCheckShell;
		aCheckShell.LoadShells(theShell);
		if (aCheckShell.HasBadEdges())
		{
			TopoDS_Compound badedges = aCheckShell.BadEdges();
			b.Add(compound, badedges);
		}
	}*/
	
	TheExe.ProgressSetValue(40);

	/*TopoDS_HShape* pHShape1 = new TopoDS_HShape(igesshape);
	TopoDS_HShape* pHShape = static_cast<TopoDS_HShape*>(pHShape1);
	TopoDS_Shape mainshape = pHShape->Shape();*/
	//sewing
	if (m_dSewingTol != 0)
	{
		BRepBuilderAPI_Sewing bas(m_dSewingTol);
		bas.Load(igesshape);
		bas.Perform();

		igesshape = bas.SewedShape();
	}

	TheExe.ProgressSetValue(100);
	TheExe.ProgressExit();

	//scale shape
	int nUnitSelIndex = -1;
	GetXmlValueN(Igs_Import_UnitType, nUnitSelIndex);

	int nCurLength = ConfigGetUnitCurrentValue(0);
	double dCurMeter = ConfigGetUnitFactor(0, nCurLength);//current unit to meter
	double dSelMeter = ConfigGetUnitFactor(0, nUnitSelIndex);//select unit to meter
	double scale = dSelMeter / dCurMeter;

	/*gp_Trsf theTransformation;
	theTransformation.SetScale(gp_Pnt(0, 0, 0), scale);
	BRepBuilderAPI_Transform  transform(theTransformation);
	transform.Perform(igesshape);
	igesshape = transform.ModifiedShape(igesshape);*/

	//avoid root is not compound
	TopoDS_Compound compound;
	BRep_Builder b;
	b.MakeCompound(compound);
	if (igesshape.ShapeType() != TopAbs_COMPOUND)
	{
		b.Add(compound, igesshape);
		igesshape = compound;
	}

	//fix shape
	iogBasicTools iobasictools;
	iobasictools.Heal(igesshape, 0.0001, m_dHoleRad, 5);

	string strBuildinfo = "Reconstructing...";
	TheExe.ProgressInit(strBuildinfo.c_str());
	TheExe.ProgressSetValue(0);

	/*ShapeUpgrade_UnifySameDomain samedomain(igesshape);
	samedomain.Build();
	igesshape = samedomain.Shape();*/

	int nCount = 0;
	std::vector<std::pair<TopoDS_Shape, std::string>> vecReCSNameMap, vecReFaceNameMap;
	QHash<QString, int> hashUsedFace;
	igesshape = iogOccKernelTools::ReBuildShape(faceList.Size(), nCount, igesshape, vecUniqName, vecnamemap, vecFaceNameMap, hashFace
	, vecReCSNameMap, vecReFaceNameMap, hashUsedFace);

	TopoDS_HShape* entryshape = new TopoDS_HShape(igesshape);
	TheExe.ProgressExit();

	//record max index
	comFaceMgr *pFaceMgr = comFace::Manager();
	comVerticeMgr * pVertMgr = comVertice::Manager();
	comSurfaceMgr *pSurfaceMgr = comSurface::Manager();
	UINT uMaxSurfIx = pSurfaceMgr->MaxIndex();
	UINT uMaxFaceIx = pFaceMgr->MaxIndex();
	UINT uMaxVtxIx = pVertMgr->MaxIndex();
	bool firstloop = false;

	if (uMaxFaceIx == CORE_INVALID_INDEX && uMaxSurfIx == CORE_INVALID_INDEX)
	{
		firstloop = true;
	}

	string strMeshinfo = "Start discretizing...";
	TheExe.ProgressInit(strMeshinfo.c_str());
	TheExe.ProgressSetValue(0);

	//export iges
	//IGESCAFControl_Writer writer;
	//writer.AddShape(igesshape);
	//Standard_CString outpath = "D:/newigess.igs";
	//writer.Write(outpath);

	QHash<QString, UINT> hashVertIdx;

	if (!iobasictools.DiscretizeShape(m_dLinearDelVal, m_dAngleDelVal, faceList.Size(), firstloop, uMaxSurfIx, scale, vecUniqName, vecReCSNameMap,
		vecReFaceNameMap, m_pFileName, entryshape, hashVertIdx))
	{
		return false;
	}
	TheExe.ProgressExit();

	string strFixinfo = "Refining grid...";
	TheExe.ProgressInit(strFixinfo.c_str());
	TheExe.ProgressSetValue(0);

	iobasictools.ClearFreeEdges(false, scale, firstloop, uMaxFaceIx, igesshape, uMaxSurfIx, hashVertIdx);
	iobasictools.ClearFreeEdges(false, scale, firstloop, uMaxFaceIx, igesshape, uMaxSurfIx, hashVertIdx);

	TheExe.ProgressSetValue(100);
	TheExe.ProgressExit();

	anApp->Close(doc);

	DWORD t2 = GetTickCount();
	double dWaittime = (double)(t2 - t1) / 1000;

	/*QString strMsg = QObject::tr("Import file using %1 seconds").arg(dWaittime);
	TheExe.PrintMessage(strMsg, apiExe::MSG_NORMAL);*/
	return true;
}

//////////////////////////////////////////////////////////////////
//