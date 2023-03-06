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

#include "iogStpReader.h"
#include "iogOccKernelTools.h"

#define Mesh_Linear_Defl 0.5
#define Sew_Tole 1.0e-1

#define Stp_Import_Tolerance   "iog_param/parameter/import_stp/tolerance"
#define Stp_Import_HoleRad  "iog_param/parameter/import_stp/holearea"
#define Stp_Import_FilletRad  "iog_param/parameter/import_stp/filletrad"
#define Stp_Import_Unit  "iog_param/parameter/import_stp/unit"
#define Stp_Import_UnitType "iog_param/parameter/import_stp/unittype"
#define Stp_Import_DiscreType "iog_param/parameter/import_stp/discretype"

#define Stp_Import_DiscreLinear   "iog_param/parameter/import_stp/discretlinear"
#define Stp_Import_DiscreAngle   "iog_param/parameter/import_stp/discretangle"
#define Stp_Import_SewingTolerance   "iog_param/parameter/import_stp/sewingtolerance"
//////////////////////////////////////////////////////////////////
//

iogStpReader::iogStpReader(const char * pFilename)
{
	m_pFileName = _strdup(pFilename);
}

iogStpReader::~iogStpReader()
{
	free(m_pFileName);
}

//////////////////////////////////////////////////////////////////
//

bool iogStpReader::ReadSTP()
{
	iogBasicTools iogbasictools;
	char pName[128];
	iogbasictools.GetPartName(pName, m_pFileName);

	string strImportinfo, strImport, strFilename;
	strImport = "Import ";
	strFilename = pName;
	strImportinfo = strImport + strFilename + ".stp";

	TheExe.ProgressInit(strImportinfo.c_str());
	TheExe.ProgressSetValue(0);

	QString strMsg;

	GetXmlValueD(Stp_Import_Tolerance, m_dTolerance);
	if (abs(m_dTolerance) < MATH_ZERO_D12)
	{
		m_dTolerance = MATH_ZERO_D12;
	}

	GetXmlValueD(Stp_Import_HoleRad, m_dHoleRad);
	GetXmlValueD(Stp_Import_FilletRad, m_dFilletRad);

	GetXmlValueD(Stp_Import_Unit, m_nUnit);
	GetXmlValueN(Stp_Import_DiscreType, m_nDiscreType);

	GetXmlValueD(Stp_Import_DiscreLinear, m_dLinearDelVal);
	GetXmlValueD(Stp_Import_DiscreAngle, m_dAngleDelVal);
	GetXmlValueD(Stp_Import_SewingTolerance, m_dSewingTol);

	if (m_dLinearDelVal == 0 || m_dAngleDelVal == 0)
	{
		QString strMsgError = QObject::tr("Linear Deflection and Angle Deflection can not be 0, please check.");
		TheExe.PrintMessage(strMsgError, apiExe::MSG_NORMAL);
		TheExe.ProgressExit();
		return false;
	}

	if (!GetSTPFile())
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

bool iogStpReader::GetSTPFile()
{
	DWORD dStart = GetTickCount();
	Standard_CString strPath = (Standard_CString)m_pFileName;

	STEPCAFControl_Reader cafreader;
	if (cafreader.ReadFile(strPath) != IFSelect_RetDone)
	{
		return false;
	}

	/*Handle(TDocStd_Document) doc;
	Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
	anApp->NewDocument("BinOcaf", doc);  //BinOcaf
	cafreader.SetColorMode(true);
	cafreader.SetNameMode(true);
	cafreader.SetLayerMode(true);
	cafreader.SetSHUOMode(true);
	cafreader.Transfer(doc);
	iogOccKernelTools::GetDocShape(doc, igesshape);
	helix test
	TopoDS_Shape helix = iogOccKernelTools::makeSpiralHelix(30, 10, 20, 8, 0, true);*/

	STEPControl_Reader reader = cafreader.Reader();

	reader.TransferRoots();
	TopoDS_Shape igesshape = reader.OneShape();

	TopLoc_Location topLocation = igesshape.Location();
	gp_Trsf trf = topLocation.Transformation();

	bool bIfSolid = true;
	iogOccKernelTools::IfModelRealSolid(igesshape, bIfSolid);

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
	//sewing
	if (!bIfSolid)
	{
		if (m_dSewingTol != 0)
		{
			BRepBuilderAPI_Sewing bas(m_dSewingTol);
			bas.Load(igesshape);
			bas.Perform();
			int nFreeEdge = bas.NbFreeEdges();

			igesshape = bas.SewedShape();
		}
	}

	/*BinDrivers::DefineFormat(anApp);
	doc->ChangeStorageFormat("BinOcaf");
	TCollection_ExtendedString TPath = "d:\\text.cbf";
	anApp->SaveAs(doc, TPath);*/

	std::vector<std::pair<TopoDS_Shape, std::string>> vecCSNameMap, vecFaceNameMap;
	QHash<QString, std::string> hashFace;

	//retrive namemap method 1
	/*XCAFDoc_DataMapOfShapeLabel xdm = cafreader.GetShapeLabelMap();
	XCAFDoc_DataMapOfShapeLabel::iterator iter(xdm);
	for (iter = xdm.begin(); iter != xdm.end(); iter++)
	{
		TopoDS_Shape compareShape = iter.Iterator().Key();
		TopAbs_ShapeEnum compareType = compareShape.ShapeType();
		if (compareType != TopAbs_COMPOUND || compareType != TopAbs_SOLID)
			continue;
		TDF_Label namelabel = iter.Iterator().Value();
		Handle(TDataStd_Name) name;
		TCollection_ExtendedString nametext;
		if (namelabel.FindAttribute(TDataStd_Name::GetID(), name))
		{
			nametext = name->Get();
			std::string s1 = TCollection_AsciiString(nametext).ToCString();
			vecCSNameMap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
		}
	}*/
	//retrive namemap method 2
	std::vector<string> vecAllFaceName, vecUniqName;
	std::string strSewed = "Sewed Surface";
	vecUniqName.push_back(strSewed);

	Handle(XSControl_TransferReader) treader = reader.WS()->TransferReader();
	////method1, large data is too slow(120000 faces)
	/*for (int i=0; i < vecFaces.size(); i++)
	{
		Handle(Standard_Transient) transient = treader->EntityFromShapeResult(vecFaces[i], 1);
		Handle(StepRepr_RepresentationItem) item = Handle(StepRepr_RepresentationItem)::DownCast(transient);
		if (item)
		{
			TopoDS_Shape compareShape = vecFaces[i];
			Handle(TCollection_HAsciiString) hascii = item->Name();
			TCollection_AsciiString ascii = hascii->String();
			std::string s1 = ascii.ToCString();
			vecnamemap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
			vecAllFaceName.push_back(s1);
		}
	}*/
	////method2
	Handle(XSControl_WorkSession) workSession = reader.WS();
	Handle(Interface_InterfaceModel) model = workSession->Model();
	Handle(Transfer_TransientProcess) transPro = treader->TransientProcess();
	int nEntities = model->NbEntities();
	for (int i = 1; i <= nEntities; i++)
	{
		Handle(Standard_Transient) transient = model->Value(i);
		auto dynamicType = transient->DynamicType();
		Handle(Transfer_Binder) binder = transPro->Find(transient);
		if (!binder)
			continue;

		TopoDS_Shape compareShape = TransferBRep::ShapeResult(binder);
		if(compareShape.IsNull())
			continue;

		if (transient->DynamicType()->SubType("StepShape_Face"))
		{
			Handle(StepRepr_RepresentationItem) item = Handle(StepRepr_RepresentationItem)::DownCast(transient);
			if (item)
			{
				Handle(TCollection_HAsciiString) hascii = item->Name();
				if (!hascii)
					continue;
				TCollection_AsciiString ascii = hascii->String();
				std::string s1 = ascii.ToCString();

				if (s1 == "")
					continue;

				vecFaceNameMap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
				vecAllFaceName.push_back(s1);

				TopAbs_ShapeEnum type = compareShape.ShapeType();
				if (type != TopAbs_FACE)
				{
					continue;
				}
				
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
		else if (transient->DynamicType()->SubType("StepRepr_NextAssemblyUsageOccurrence"))
		{
			Handle(StepRepr_NextAssemblyUsageOccurrence) item = Handle(StepRepr_NextAssemblyUsageOccurrence)::DownCast(transient);
			if (item)
			{
				Handle(TCollection_HAsciiString) hascii = item->Name();
				if (!hascii)
					continue;
				TCollection_AsciiString ascii = hascii->String();
				std::string s1 = ascii.ToCString();

				if (s1 == "")
					continue;

				vecCSNameMap.push_back(std::pair<TopoDS_Shape, std::string>(compareShape, s1));
			}
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
	//retrive namemap method 3
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

		iogOccKernelTools::GetShapeNameMap(assembly, freeshape, vecnamemap);
	}*/

	//avoid root is not compound
	if (igesshape.ShapeType() != TopAbs_COMPOUND)
	{
		BRep_Builder builder;
		TopoDS_Compound cmp;
		builder.MakeCompound(cmp);
		builder.Add(cmp, igesshape);
		igesshape = cmp;
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

	//stp can recognize model unit
	TColStd_SequenceOfAsciiString lengthnames, anglenames, solidanglenames;
	reader.FileUnits(lengthnames, anglenames, solidanglenames);
	string strlength = "";
	for (int i = 1; i <= lengthnames.Length(); i++)
	{
		TCollection_AsciiString aStr = lengthnames(i);
		strlength = aStr.ToCString();
	}

	//scale shape
	int nUnitSelIndex = -1;
	if (strlength == "metre")
		nUnitSelIndex = 0;
	else if (strlength == "centimetre")
		nUnitSelIndex = 1;
	else if (strlength == "millimetre")
		nUnitSelIndex = 2;
	else if (strlength == "kilometre")
		nUnitSelIndex = 3;
	else if (strlength == "inch" || strlength == "INCH")
		nUnitSelIndex = 4;
	else if (strlength == "foot" || strlength == "FOOT")
		nUnitSelIndex = 5;
	else
	{
		nUnitSelIndex = 0;
	}
	
	int nCurLength = ConfigGetUnitCurrentValue(0);
	double dCurMeter = ConfigGetUnitFactor(0, nCurLength);//current unit to meter
	double dSelMeter = ConfigGetUnitFactor(0, nUnitSelIndex);//select unit to meter
	double scale = dSelMeter / dCurMeter;	

	/*gp_Trsf theTransformation;
	theTransformation.SetScale(gp_Pnt(0, 0, 0), scale);
	BRepBuilderAPI_Transform  transform(theTransformation);
	transform.Perform(igesshape);
	igesshape = transform.ModifiedShape(igesshape);*/

	//fix shape
	iogBasicTools iogbasictools;
	//iogbasictools.Heal(igesshape, m_dTolerance, m_dHoleRad, m_dFilletRad);

	//test free edges
	/*ShapeAnalysis_ShapeContents ssc;
	ssc.Perform(igesshape);
	int freeedge = ssc.NbFreeEdges();*/

	TheExe.ProgressSetValue(100);
	TheExe.ProgressExit();

	//ShapeUpgrade_UnifySameDomain samedomain(igesshape);
	//samedomain.Build();
	//igesshape = samedomain.Shape();

	//export stp
	/*STEPControl_Writer writer;
	writer.Transfer(igesshape, STEPControl_AsIs);
	Standard_CString outpath = "D:/newstp1.stp";
	writer.Write(outpath);*/

	std::vector<std::pair<TopoDS_Shape, std::string>> vecReCSNameMap, vecReFaceNameMap;
	string strBuildinfo = "Reconstructing...";
	TheExe.ProgressInit(strBuildinfo.c_str());
	TheExe.ProgressSetValue(0);
	int nCount = 0;
	QHash<QString, int> hashUsedFace;
	igesshape = iogOccKernelTools::ReBuildShape(faceList.Size(), nCount, igesshape, vecUniqName,
		vecCSNameMap, vecFaceNameMap, hashFace, vecReCSNameMap, vecReFaceNameMap, hashUsedFace);
	TopoDS_HShape* entryshape = new TopoDS_HShape(igesshape);
	TheExe.ProgressExit();
	
	string strMeshinfo = "Start discretizing...";
	TheExe.ProgressInit(strMeshinfo.c_str());
	TheExe.ProgressSetValue(0);

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

	QHash<QString, UINT> hashVertIdx;

	//geometry repair trial
	/*BRepCheck_Analyzer analyzer(igesshape);
	if (!analyzer.IsValid())
	{
		for (TopExp_Explorer exp(igesshape, TopAbs_FACE); exp.More(); exp.Next())
		{
			TopoDS_Face face = TopoDS::Face(exp.Current());
			Handle(BRepCheck_Result) result = analyzer.Result(face);
			BRepCheck_ListOfStatus infos = result->Status();
			int a = 0;
		}	
	}*/

	if (!iogbasictools.DiscretizeShape(m_dLinearDelVal, m_dAngleDelVal, faceList.Size(), firstloop, uMaxSurfIx, scale,
		vecUniqName, vecReCSNameMap, vecReFaceNameMap, m_pFileName,
		entryshape, hashVertIdx))
	{
		return false;
	}
	TheExe.ProgressExit();

	//record max index
	if (uMaxFaceIx == CORE_INVALID_INDEX && uMaxSurfIx == CORE_INVALID_INDEX)
	{
		firstloop = true;
	}

	string strFixinfo = "Refining grid...";
	DWORD dEnd = GetTickCount();
	double dWaittime = (double)(dEnd - dStart) / 1000;

	TheExe.ProgressInit(strFixinfo.c_str());
	TheExe.ProgressSetValue(0);
	iogbasictools.ClearFreeEdges(true, scale, firstloop, uMaxFaceIx, igesshape, uMaxSurfIx, hashVertIdx);
	iogbasictools.ClearFreeEdges(true, scale, firstloop, uMaxFaceIx, igesshape, uMaxSurfIx, hashVertIdx);

	TheExe.ProgressSetValue(100);
	TheExe.ProgressExit();

	//CDM_CanCloseStatus sts = anApp->CanClose(doc);
	//anApp->Close(doc);

	dEnd = GetTickCount();
	dWaittime = (double)(dEnd - dStart)/1000;

	QString strMsg = QObject::tr("Import file using %1 seconds").arg(dWaittime);
	TheExe.PrintMessage(strMsg, apiExe::MSG_NORMAL);
	return true;
}
//////////////////////////////////////////////////////////////////
//