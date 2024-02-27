#include "OCCTest.h"
#include "OCCBasicTools.h"

#include <Geom_TrimmedCurve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Circle.hxx>

#include <GC_MakeArcOfCircle.hxx>

#include <Geom_BSplineCurve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Compound.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>

#include <BRepOffsetAPI_MakePipe.hxx>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Section.hxx>

#include <BRep_Builder.hxx>
#include <BRepAdaptor_Surface.hxx>

#include <BRepTools.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Dir.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Circ.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Torus.hxx>
#include <gp_Cone.hxx>

#include <BRepOffsetAPI_MakeOffset.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>
#include <STEPControl_Controller.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI_Writer.hxx>

#include <TopExp_Explorer.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepOffsetAPI_MakeDraft.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFeat_MakeDPrism.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepFill_Draft.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BndLib_Add3dCurve.hxx>
#include <BRepFill_Sweep.hxx>
#include <GeomFill_LocationDraft.hxx>
#include <BRepFill_DraftLaw.hxx>
#include <Geom_Line.hxx>
#include <Adaptor3d_HCurve.hxx>
#include <BRepFill_ShapeLaw.hxx>
#include <BRepLib.hxx>
#include <Xw_Window.hxx>
#include <BRepFill_Filling.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <NCollection_Mat4.hxx>
#include <GC_MakePlane.hxx>

#include <Vector>

#define PI 3.14159265354
int nero = 0;



//void OCCTest::ConvertEllipse2Polygon(gp_Elips& elips, int n, TopoDS_Wire& my_wire)
//{
//	//计算椭圆周长c
//	Standard_Real a = elips.MajorRadius();
//	Standard_Real b = elips.MinorRadius();
//	Standard_Real c_elips = 2 * PI * b + 4*(a - b);
//	Standard_Real inter_distance = c_elips / n;
//
//	std::vector<gp_Pnt> point_list;
//	TopoDS_Edge ellipse_edge = BRepBuilderAPI_MakeEdge(elips);
//	TopoDS_Wire ellipse_wwire = BRepBuilderAPI_MakeWire(ellipse_edge);
//	BRepAdaptor_CompCurve com_curve(ellipse_wwire);
//	GCPnts_UniformAbscissa uni_abs = GCPnts_UniformAbscissa(com_curve, n, -1);
//	if (uni_abs.IsDone())
//	{
//		gp_Pnt this_point;
//		for (Standard_Integer i = 1; i <= n; i++)
//		{
//			Standard_Real real = uni_abs.Parameter(i);	
//			com_curve.D0(real, this_point);
//			point_list.push_back(this_point);
//		}
//	}
//
//	if (point_list.size() != n)
//		return;
//
//	std::vector<TopoDS_Edge> allEdges;
//	std::vector<Standard_Real> disatances;
//	for (int i = 0; i < point_list.size(); i++)
//	{
//		Standard_Real distance;
//		if (i != point_list.size() - 1)
//		{
//			allEdges.push_back(BRepBuilderAPI_MakeEdge(point_list[i], point_list[i + 1]));
//			distance = point_list[i].Distance(point_list[i + 1]);
//			disatances.push_back(distance);
//		}
//		else
//		{
//			allEdges.push_back(BRepBuilderAPI_MakeEdge(point_list[i], point_list[0]));
//			distance = point_list[i].Distance(point_list[0]);
//			disatances.push_back(distance);
//		}
//	}
//	OCCBasicTools::GetOrderWireFromEdges(allEdges, my_wire);
//
//}

static void ComputeTrsf(const TopoDS_Wire& W,
	const gp_Dir& D,
	Bnd_Box& Box,
	gp_Trsf& Tf)
{
	// Calculate approximate barycenter
	BRepTools_WireExplorer Exp(W);
	// Class BRep_Tool without fields and without Constructor :
	//  BRep_Tool BT;
	gp_XYZ Bary(0., 0., 0.);
	Standard_Integer nb;

	for (nb = 0; Exp.More(); Exp.Next()) {
		//    Bary += BT.Pnt(Exp.CurrentVertex()).XYZ();
		Bary += BRep_Tool::Pnt(Exp.CurrentVertex()).XYZ();
		nb++;
	}
	Bary /= nb;

	// Calculate the Transformation  
	gp_Ax3 N(Bary, D);
	Tf.SetTransformation(N);
	BRepAdaptor_Curve AC;
	//  BndLib_Add3dCurve BC;  

	  // transformation to the wire
	TopoDS_Wire TheW = W;
	TopLoc_Location Loc(Tf);
	TheW.Location(Loc);


	// Calculate the box
	Box.SetVoid();
	for (Exp.Init(TheW); Exp.More(); Exp.Next()) {
		AC.Initialize(Exp.Current());
		//    BC.Add(AC, 0.1, Box);
		BndLib_Add3dCurve::Add(AC, 0.1, Box);
	}
}

static Standard_Boolean GoodOrientation(const Bnd_Box& B,
	const Handle(BRepFill_LocationLaw)& Law,
	const gp_Dir& D)
{
	Standard_Real f, l, r, t;
	Standard_Real aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;

	B.Get(aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
	gp_Pnt P1(aXmin, aYmin, aZmin), P2(aXmax, aYmax, aZmax);
	gp_Vec V(P1, P2);

	Law->CurvilinearBounds(Law->NbLaw(), f, l);
	r = V.Magnitude() / l;

	Standard_Integer ii, Ind;
	//#ifndef OCCT_DEBUG
	Standard_Integer Nb = (Standard_Integer)(4 + (10 * r));
	//#else
	//  Standard_Integer Nb = 4+(10*r);
	//#endif
	r = l / Nb;

	Nb++; // Number of points

	TColgp_Array1OfPnt Pnts(1, Nb);
	Handle(Adaptor3d_HCurve) AC;
	gp_XYZ Bary(0., 0., 0.);

	for (ii = 1; ii <= Nb; ii++) {
		Law->Parameter((ii - 1) * r, Ind, t);
		AC = Law->Law(Ind)->GetCurve();
		AC->D0(t, Pnts(ii));
		Bary += Pnts(ii).XYZ();
	}

	Bary /= Nb;
	gp_Pnt Centre(Bary);
	gp_Vec Normal(D.XYZ());
	Standard_Real Angle = 0;
	gp_Vec Ref(Centre, Pnts(1));

	for (ii = 2; ii <= Nb; ii++) {
		gp_Vec R(Centre, Pnts(ii));
		Angle += Ref.AngleWithRef(R, Normal);
		Ref = R;
	}

	return (Angle >= 0);
}

TopoDS_Face OCCTest::GetFaceFromPntList(std::vector<gp_Pnt> PntsList)
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

bool OCCTest::fillet(TopoDS_Shape& shape)
{
	BRepFilletAPI_MakeFillet  Rake(shape);
	ChFi3d_FilletShape  fillet_shape = ChFi3d_Rational;
	Rake.SetFilletShape(fillet_shape);

	TColgp_Array1OfPnt2d  ParAndRad(1, 2);
	Standard_Integer i = ParAndRad.Length();
	ParAndRad(2).SetCoord(1, 2);
	ParAndRad(1).SetCoord(2, 2);

	TopTools_ListOfShape ListOfEdge;

	TopExp_Explorer  ex(shape, TopAbs_EDGE);
	ListOfEdge.Append(ex.Value());
	ex.Next();                                             //所有棱柱的第一条棱边
	ListOfEdge.Append(ex.Value());
	ex.Next();
	ex.Next();
	ex.Next();
	ListOfEdge.Append(ex.Value());                        //到这边是三棱柱
	ex.Next();
	ex.Next();
	ex.Next();
	ex.Next();
	ex.Next();
	ListOfEdge.Append(ex.Value());                          //到这边是四棱柱
	ex.Next();
	ex.Next();
	ex.Next();
	ex.Next();
	ex.Next();
	ex.Next();
	ex.Next();
	ex.Next();
	ListOfEdge.Append(ex.Value());                          //到这边是五棱柱

	TopTools_ListIteratorOfListOfShape itls;
	for (itls.Initialize(ListOfEdge); itls.More(); itls.Next())
	{
		Rake.Add(1, TopoDS::Edge(itls.Value()));
	}
	Rake.Build();
	if (Rake.IsDone())
	{
		shape = Rake.Shape();
		return true;
	}
	
	return false;
}

bool OCCTest::OffSet(TopoDS_Shape& shape, double value, bool if_filleted)
{
	BRepOffsetAPI_MakeOffsetShape make_offset;
	if (if_filleted == false)
	{
		make_offset.PerformByJoin(shape, value, 0.01, BRepOffset_Skin, Standard_False, Standard_False, GeomAbs_Intersection, Standard_False);
		shape = make_offset.Shape();
	}
	
	return true;
}

void OCCTest::ConvertEllipse2Polygon(gp_Elips& elips, int n, TopoDS_Wire& my_wire)
{
	double num1 = n - 1;
	double num2 = n;
	double Jishu_xishu = num1 / num2;
	//准备工作
	std::vector<gp_Pnt> point_list;

	//计算椭圆周长c
	Standard_Real a = elips.MajorRadius();
	Standard_Real b = elips.MinorRadius();

	int mo_num = n % 2;
	if (mo_num == 1)
	{
		//奇数边
		point_list.push_back(gp_Pnt(a, 0, 0));
		//一边散点
		int divid_num = (n - 1) / 2;
		for (int i = 1; i <= divid_num; i++)
		{
			Standard_Real x = a - (Jishu_xishu + 1) * a * i / divid_num;
			Standard_Real y = sqrt(pow(b,2) - pow(b, 2) * pow(x, 2) / pow(a, 2));
			point_list.push_back(gp_Pnt(x, y, 0));
		}
		for (int i = 1; i <= divid_num; i++)
		{
			Standard_Real x = a - (Jishu_xishu + 1) * a * (divid_num-i+1) / divid_num;
			Standard_Real y = -1 * sqrt(pow(b, 2) - pow(b, 2) * pow(x, 2) / pow(a, 2));
			point_list.push_back(gp_Pnt(x, y, 0));
		}
	}
	else
	{
		//偶数边 --- 按长度
		//point_list.push_back(gp_Pnt(a, 0, 0));
		////一边散点
		//int divid_num = (n - 2) / 2;
		//for (int i = 1; i <= divid_num; i++)
		//{
		//	Standard_Real x = a - 2 * a * i / (divid_num + 1);
		//	Standard_Real y = sqrt(pow(b, 2) - pow(b, 2) * pow(x, 2) / pow(a, 2));
		//	point_list.push_back(gp_Pnt(x, y, 0));
		//}
		//point_list.push_back(gp_Pnt(-1 * a, 0, 0));
		//for (int i = 1; i <= divid_num; i++)
		//{
		//	Standard_Real x = a - 2 * a * (divid_num - i + 1) / (divid_num + 1);
		//	Standard_Real y = -1 * sqrt(pow(b, 2) - pow(b, 2) * pow(x, 2) / pow(a, 2));
		//	point_list.push_back(gp_Pnt(x, y, 0));
		//}

		//偶数边--按角度
		point_list.push_back(gp_Pnt(a, 0, 0));
		double theta = (360 / n) * PI / 180;
		for (int i = 1; i < n; i++)
		{
			Standard_Real x = a * cos(theta * i);
			Standard_Real y = b * sin(theta * i);
			point_list.push_back(gp_Pnt(x, y, 0));
		}
	}
	std::vector<TopoDS_Edge> allEdges;

	for (int i = 0; i < point_list.size(); i++)
	{
		TopoDS_Edge edge_x;
		if (i != point_list.size() - 1)
		{
			edge_x = BRepBuilderAPI_MakeEdge(point_list[i], point_list[i + 1]);
			allEdges.push_back(edge_x);
		}
		else
		{
			edge_x = BRepBuilderAPI_MakeEdge(point_list[i], point_list[0]);
			allEdges.push_back(edge_x);
		}
	}
	OCCBasicTools::GetOrderWireFromEdges(allEdges, my_wire);
}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(5, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 20, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 20, 0), gp_Pnt(0, 0, 0)));
//
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//	TopAbs_Orientation face_ori = ProfileFace.Orientation();
//
//	gp_Ax2 ax2(gp_Pnt(3, 3, 0), gp_Dir(0, 0, 1));
//	gp_Circ c2(ax2, 2);
//	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(c2);
//	TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge2);
//	TopAbs_Orientation wire_ori = wire2.Orientation();
//
//	gp_Ax2 ax3(gp_Pnt(4, 4, 0), gp_Dir(0, 0, 1));
//	gp_Circ c3(ax3, 2);
//	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(c3);
//	TopoDS_Wire wire3 = BRepBuilderAPI_MakeWire(edge3);
//	TopAbs_Orientation wire_ori3 = wire3.Orientation();
//
//	if(face_ori== wire_ori)
//	{
//		wire2.Reverse();
//	}
//	if (face_ori == wire_ori3)
//	{
//		wire3.Reverse();
//	}
//
//	ProfileFace = BRepBuilderAPI_MakeFace(ProfileFace, wire2);
//	//ProfileFace = BRepBuilderAPI_MakeFace(ProfileFace, wire3);
//
//	std::vector<TopoDS_Shape> remove_list;
//	remove_list.push_back(ProfileFace);
//
//	TopExp_Explorer ex;
//	for (ex.Init(ProfileFace, TopAbs_EDGE); ex.More(); ex.Next())
//	{
//		TopoDS_Edge edge = TopoDS::Edge(ex.Current());
//		remove_list.push_back(edge);
//	}
//
//	std::vector<TopoDS_Shape> remove_list1;
//	remove_list1.push_back(ProfileFace);
//	remove_list1.push_back(remove_list[5]);
//
//	//ProfileFace = BRepBuilderAPI_MakeFace(ProfileFace, wire2);
//
//	shape = ProfileFace;
//	//remove_list1.push_back(ProfileFace);
//	//removeEdge(remove_list1, shape);
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(5, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 20, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 20, 0), gp_Pnt(0, 0, 0)));
//
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//	TopAbs_Orientation face_ori = ProfileFace.Orientation();
//
//	gp_Ax2 ax2(gp_Pnt(3, 3, 0), gp_Dir(0, 0, 1));
//	gp_Circ c2(ax2, 2);
//	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(c2);
//	TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge2);
//	//TopAbs_Orientation wire_ori = wire2.Orientation();
//
//	gp_Ax2 ax3(gp_Pnt(4, 4, 0), gp_Dir(0, 0, 1));
//	gp_Circ c3(ax3, 2);
//	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(c3);
//	TopoDS_Wire wire3 = BRepBuilderAPI_MakeWire(edge3);
//
//
//	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
//	new_bop->AddArgument(wire2);
//	new_bop->AddTool(wire3);
//
//	if (face_ori == wire_ori)
//	{
//		wire2.Reverse();
//	}
//	
//
//	ProfileFace = BRepBuilderAPI_MakeFace(ProfileFace, wire2);
//	ProfileFace = BRepBuilderAPI_MakeFace(ProfileFace, wire3);
//
//	shape = ProfileFace;
//
//	return true;
//}
bool OCCTest::MakeAnnotation(std::vector<gp_Pnt> pnts, std::string text)
{
	Handle(V3d_Viewer) theViewer;
	Handle(V3d_View) aView = new V3d_View(theViewer);
	Handle(Graphic3d_GraphicDriver) driver = aView->Viewer()->Driver();
	Handle(Graphic3d_StructureManager) manager = new  Graphic3d_StructureManager(driver);
	Handle(Prs3d_Presentation) thePrs = new  Prs3d_Presentation(manager);
	//创建文字
	Handle(Graphic3d_Group) aGroup = thePrs->NewGroup();
	// change the text aspect
	Handle(Graphic3d_AspectText3d) aTextAspect = new Graphic3d_AspectText3d();
	aTextAspect->SetTextZoomable(true);
	aTextAspect->SetTextAngle(45.0);
	aGroup->SetPrimitivesAspect(aTextAspect);

	// add a text primitive to the structure
	Handle(Graphic3d_Text) aText = new Graphic3d_Text(16.0f);
	aText->SetText("Text");
	aText->SetPosition(gp_Pnt(1, 1, 1));
	aGroup->AddText(aText);

	////创建箭头
	//Standard_Integer aNbTria = 1;
	//Handle(Graphic3d_ArrayOfTriangles) anArray = new Graphic3d_ArrayOfTriangles(theVerticesMaxCount, theEdgesMaxCount, Graphic3d_ArrayFlags_VertexNormal);
	//// add vertices to the array
	//anArray->AddVertex(-1.0, 0.0, 0.0); // vertex 1
	//anArray->AddVertex(1.0, 0.0, 0.0); // vertex 2
	//anArray->AddVertex(0.0, 1.0, 0.0); // vertex 3
	//anArray->AddVertex(0.0, -1.0, 0.0); // vertex 4

	//// add edges to the array
	//anArray->AddEdges(1, 2, 3); // first triangle
	//anArray->AddEdges(1, 2, 4); // second triangle

	//// add the array to the structure
	//Handle(Graphic3d_Group) aGroup = thePrs->NewGroup();
	//aGroup->AddPrimitiveArray(anArray);
	//aGroup->SetGroupPrimitivesAspect(myDrawer->ShadingAspect()->Aspect());

	////显示
	//Handle(V3d_Viewer) theViewer;
	//Handle(AIS_InteractiveContext) aContext = new AIS_InteractiveContext(theViewer);

	//BRepPrimAPI_MakeWedge aWedgeMaker(theWedgeDX, theWedgeDY, theWedgeDZ, theWedgeLtx);
	//TopoDS_Solid aShape = aWedgeMaker.Solid();
	//Handle(AIS_Shape) aShapePrs = new AIS_Shape(aShape); // creation of the presentable object
	//aContext->Display(aShapePrs, AIS_Shaded, 0, true);   // display the presentable object and redraw 3d viewer

	return true;
}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//
//	
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	// create a default connection 
//	Handle(Aspect_DisplayConnection) aDisplayConnection;
//	// create a graphic driver from default connection 
//	Handle(OpenGl_GraphicDriver) aGraphicDriver = new OpenGl_GraphicDriver(aDisplayConnection);
//	// create a viewer 
//	V3d_Viewer* myViewer = new V3d_Viewer(aGraphicDriver);
//	// set parameters for V3d_Viewer 
//	// defines default lights 
//	// positional-light 0.3 0.0 0.0 
//	// directional-light V3d_XnegYposZpos 
//	// directional-light V3d_XnegYneg 
//	// ambient-light 
//	myViewer->SetDefaultLights();
//	// activates all the lights defined in this viewer 
//	myViewer->SetLightOn();
//	// set background color to black 
//	myViewer->SetDefaultBackgroundColor(Quantity_NOC_BLACK);
//
//	Handle(V3d_View) myView = myViewer->CreateView();
//	/*HWND wnd = gdk_win32_window_get_impl_hwnd(gtk_widget_get_window(drawingarea1));
//	Handle(WNT_Window) aWNTWindow = new WNT_Window((Aspect_Handle)wnd);
//	myView->SetWindow(aWNTWindow);*/
//
//	//交互-- - 显示区块
//	AIS_InteractiveContext*	myAISContext = new AIS_InteractiveContext(myViewer);
//	shape = BRepPrimAPI_MakeBox(10, 20, 30).Solid();
//	Handle(AIS_Shape) anAISShape = new AIS_Shape(shape);
//	myAISContext->Display(anAISShape,true);
//
//	Handle(Graphic3d_StructureManager) theStructManager;
//
//	Prs3d_Presentation* aPresentation = new Prs3d_Presentation(theStructManager);
//	Handle(Graphic3d_Group) TheGroup = Prs3d_Root::CurrentGroup(aPresentation);
//
//	/*TopoDS_Shape aFace = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 0, Standard_Real(aWidth), 0, Standard_Real(aHeight));
//	Handle(AIS_TexturedShape) aTextShape = new AIS_TexturedShape(aFace);
//	aTextShape->SetTextureFileName("./Modulos.bak/Logo.bmp");
//	aTextShape->SetTextureMapOn();
//	aTextShape->SetDisplayMode(3);
//	aContext->Display(aTextShape, Standard_False);
//	aContext->UpdateCurrentViewer();*/
//
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	gp_Ax2 ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 10));
//
//	gp_Circ c1(ax1, 5);
//
//	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(c1);
//
//	shape = BRepPrimAPI_MakePrism(edge1,gp_Vec(0,0,10)).Shape();
//	TopoDS_Face ProfileFace = TopoDS::Face(shape);
//	BRepAdaptor_Surface ap_s(ProfileFace);
//
//	gp_Cylinder cylin = ap_s.Cylinder();
//	gp_Ax1 ax = cylin.Axis();
//
//	gp_Pnt pt1 = ap_s.Value(ap_s.FirstUParameter(), ap_s.FirstVParameter());
//	gp_Pnt pt2 = ap_s.Value(ap_s.LastUParameter(), ap_s.LastVParameter());
//
//	Handle(Geom_Line) ga = new Geom_Line(ax);
//	GeomAPI_ProjectPointOnCurve apj1(pt1, ga);
//	GeomAPI_ProjectPointOnCurve apj2(pt2, ga);
//
//	gp_Pnt pjpt1 = apj1.Point(1);
//	gp_Pnt pjpt2 = apj2.Point(1);
//	
//	return true;
//}

//测试面积算顺时针、逆时针
//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	BRepBuilderAPI_MakeWire aWireMaker;
//	for (int i = 0; i < anEdges.size(); i++)
//	{
//		TopoDS_Edge aOrderedEdges = anEdges[i];
//		aWireMaker.Add(aOrderedEdges);
//	}
//	Profile = aWireMaker.Wire();
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//
//	BRepAdaptor_Surface adp(ProfileFace);
//	//gp_Dir normal = adp.Direction();
//
//	GProp_GProps gg;
//	BRepGProp::SurfaceProperties(ProfileFace, gg);
//	double mass = gg.Mass();
//	//double ifg = mass * normal.Z();
//
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Edge myedge= BRepBuilderAPI_MakeEdge(gp_Pnt(-1, 0, -1), gp_Pnt(5, 0, 0)).Edge();
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//	double first, last;
//	Handle(Geom_Curve) geo_cur = BRep_Tool::Curve(myedge, first, last);
//	//double totalLength = GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(geo_cur));
//	gp_Vec vec1 = geo_cur->DN(1, 1);
//	//gp_Pnt pt1 = geo_cur->Value(totalLength);
//	Handle(Geom_Surface) geo_sur = BRep_Tool::Surface(ProfileFace);
//	GeomAPI_ExtremaCurveSurface cur_sur(geo_cur, geo_sur);
//	int nm = cur_sur.NbExtrema();
//	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	//TopoDS_Wire Profile;
//	//std::vector<TopoDS_Edge> anEdges;
//	BRepBuilderAPI_MakeEdge first_edge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(5, 0, 0));
//	TopoDS_Edge edge1 = first_edge.Edge();
//	anEdges.push_back(edge1);
//
//	BRepBuilderAPI_MakeEdge sec_edge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(-5, 0, 0));
//	TopoDS_Edge edge2 = sec_edge.Edge();
//	anEdges.push_back(edge2);
//
//	bool if_oneline = CheckOneLine(anEdges);
//	////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	return true;
//}

bool OCCTest::CheckOneLine(std::vector<TopoDS_Edge> anEdges)
{
	if (anEdges.size() != 2)
	{
		std::cout << "the seclected lines' number is not 2" << std::endl;
		return false;
	}

	struct point_posi
	{
		double x;
		double y;
		double z;
	} line1_pt1, line1_pt2, line2_pt1, line2_pt2;

	std::vector<point_posi> posis;

	TopExp_Explorer ex;
	for (int i = 0; i < 2; i++)
	{
		TopoDS_Edge edge1 = anEdges[i];
		for (ex.Init(edge1, TopAbs_VERTEX); ex.More(); ex.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(ex.Current());
			gp_Pnt pt = BRep_Tool::Pnt(vertex);
			point_posi ps;
			ps.x = pt.X();
			ps.y = pt.Y();
			ps.z = pt.Z();
			posis.push_back(ps);
		}
	}

	if (posis.size() != 4)
		return false;

	line1_pt1 = posis[0];
	line1_pt2 = posis[1];
	line2_pt1 = posis[2];
	line2_pt2 = posis[3];

	gp_Vec vec1(line1_pt2.x - line1_pt1.x, line1_pt2.y - line1_pt1.y, line1_pt2.z - line1_pt1.z);
	gp_Vec vec2(line2_pt2.x - line2_pt1.x, line2_pt2.y - line2_pt1.y, line2_pt2.z - line2_pt1.z);

	double cos_ehzta = (vec1.X() * vec2.X() + vec1.Y() * vec2.Y() + vec1.Z() * vec2.Z()) / (sqrt(pow(vec1.X(), 2) + pow(vec1.Y(), 2) + pow(vec1.Z(), 2)) * sqrt(pow(vec2.X(), 2) + pow(vec2.Y(), 2) + pow(vec2.Z(), 2)));
	if (cos_ehzta == 1 || cos_ehzta == -1)
		return true;
	return false;
}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	BRepBuilderAPI_MakeEdge added_wire_make = BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(5, 0, 10));
//	TopoDS_Shape added_wire = added_wire_make.Shape();
//
//	TopoDS_Wire Profile;
//	gp_Vec vec1(0, 0, 10);
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//	BRepPrimAPI_MakePrism* make_prism1 = new BRepPrimAPI_MakePrism(ProfileFace, vec1);
//	BRepSweep_Prism prism = make_prism1->Prism();
//	TopoDS_Shape prism_shape = prism.Shape();
//
//	TopTools_ListOfShape total;
//	total.Append(prism_shape);
//	total.Append(added_wire);
//	//shape = new_bop->Shape();
//	imprintEdge(total, shape);
//
//	std::vector<TopoDS_Shape> list_edges;           //真正可以用的面
//	TopExp_Explorer Ex;
//	for(Ex.Init(shape, TopAbs_FACE); Ex.More(); Ex.Next())
//	{
//		TopoDS_Shape this_edge = Ex.Value();
//		bool if_exist = IfTopoShapeExist(list_edges, this_edge);
//		if (if_exist == false)
//		{
//			list_edges.push_back(this_edge);
//		}
//	}
//	std::vector<TopoDS_Face> list_faces;
//	for (int i = 0; i < list_edges.size(); i++)
//	{
//		TopoDS_Face face = TopoDS::Face(list_edges[i]);
//		list_faces.push_back(face);
//	}
//	
//	std::vector<gp_Pln> pln_list;
//	for (int i = 0; i < list_faces.size(); i++)
//	{
//		gp_Pln pln;
//		TurnFace2Plane(list_faces[i], pln);
//		pln_list.push_back(pln);
//	}
//	
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	//这部分造测试数据----相交体，点，向量
//	TopoDS_Wire Profile;
//	gp_Vec vec1(0, 0, 10);
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 10), gp_Pnt(10, 20, 10)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 20, 10), gp_Pnt(20, 20, 10)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(20, 20, 10), gp_Pnt(20, 10, 10)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(20, 10, 10), gp_Pnt(10, 10, 10)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//	BRepPrimAPI_MakePrism* make_prism1 = new BRepPrimAPI_MakePrism(ProfileFace, vec1);
//	BRepSweep_Prism prism = make_prism1->Prism();
//	TopoDS_Shape prism_shape = prism.Shape();
//
//	gp_Pnt pos(0, 0, 0);
//	gp_Dir dir(30, 40, 50);
//	TopTools_ListOfShape hits;
//	RayTest(prism_shape, pos, dir, true, hits);
//	
//	shape = prism_shape;
//	//shape = hits.First();
//	
//	return true;
//}

bool OCCTest::RayTest(TopoDS_Shape shape, gp_Pnt pos, gp_Dir dir, bool is_single, TopTools_ListOfShape& hits)
{
	//点，向量生成线
	gp_Lin ray_line(pos, dir);
	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
	//相交的接口
	BOPAlgo_Section* new_bop = new BOPAlgo_Section(aAllocator);

	BRepBuilderAPI_MakeEdge makeedge(ray_line);
	TopoDS_Edge edge = makeedge.Edge();
	TopoDS_Shape edgesh = edge;
	new_bop->AddArgument(shape);
	new_bop->AddArgument(edgesh);
	new_bop->Perform();
	//直线贯穿体，得到的是两个点
	hits = new_bop->Generated(edgesh);
	return true;

	//将体explorer遍历 面、线、点， 同样使用BOPAlgo_Section方法，如果Generated生成的TopTools_ListOfShape有值，size=1，返回与其相交的对象
	//如果只要第一个对象，我的想法是先进行体相交，取TopTools_ListOfShape：：first（）第一个相交点，再将体上与之相交的面、线、点返回
}



bool OCCTest::TurnFace2Plane(TopoDS_Face face, gp_Pln& plane)
{
	TColgp_Array1OfPnt pnt_vec;              //NCollection_Array1< gp_Pnt >
	TopExp_Explorer exp(face, TopAbs_VERTEX);
	int i = 0;
	for (; exp.More(); exp.Next())
	{
		i++;
		TopoDS_Shape this_shape = exp.Value();
		TopoDS_Vertex vertex = TopoDS::Vertex(this_shape);
		gp_Pnt Pnt = BRep_Tool::Pnt(vertex);
		pnt_vec.SetValue(i, Pnt);	
	}
	GProp_PEquation pe(pnt_vec, 0);
	if (!pe.IsPlanar())
		return false;
	else
		plane = pe.Plane();
	return true;
}



bool OCCTest::IfTopoShapeExist(std::vector<TopoDS_Shape> list, TopoDS_Shape me)
{
	for (int i = 0; i < list.size(); i++)
	{
		if (me == list[i])
			return true;
	}
	return false;
}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	/*gp_Ax2 ax1(gp_Pnt(3, 3, 0), gp_Dir(0, 0, 1));
//	gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
//	gp_Circ c1(ax1, 5);
//	gp_Circ c2(ax2, 10);
//	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(c1);
//	TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge1);
//	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(c2);
//	TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge2);
//
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(wire1);
//	TopoDS_Face ProfileFace2 = BRepBuilderAPI_MakeFace(wire2);
//
//	gp_Vec vec1(0, 0, 10), vec2(0, 0, 20);
//	BRepPrimAPI_MakePrism* make_prism1 = new BRepPrimAPI_MakePrism(ProfileFace2, vec1);
//	BRepPrimAPI_MakePrism* make_prism2 = new BRepPrimAPI_MakePrism(ProfileFace1, vec2);*/
//
//	gp_Ax2 ax1(gp_Pnt(5, 5, 0), gp_Dir(0, 0, 1));
//	gp_Circ c1(ax1, 1);
//	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(c1);
//	TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge1);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(wire1);
//
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(5, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 20, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 20, 0), gp_Pnt(-10, 5, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(-10, 5, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//
//	//建立棱柱
//	gp_Vec vec(0, 0, 20);
//	BRepPrimAPI_MakePrism* make_prism = new BRepPrimAPI_MakePrism(ProfileFace, vec);
//	BRepSweep_Prism prism = make_prism->Prism();
//	shape = prism.Shape();
//	fillet(shape);
//
//	BRepPrimAPI_MakePrism* make_prism2 = new BRepPrimAPI_MakePrism(ProfileFace1, vec);
//	TopoDS_Shape shapecut = make_prism2->Shape();
//	/*TopoDS_Shape shape1 = make_prism1->Shape();
//	TopoDS_Shape shape2 = make_prism2->Shape();*/
//
//	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
//	new_bop->AddTool(shape);
//	new_bop->AddArgument(shapecut);
//	new_bop->SetOperation(BOPAlgo_CUT21);
//	new_bop->Perform();
//	bool sus = new_bop->HasGenerated();
//	shape = new_bop->Shape();
//
//	std::vector< TopoDS_Face> face_list;
//	TopExp_Explorer ex;
//	for (ex.Init(shape, TopAbs_FACE); ex.More(); ex.Next())
//	{
//		TopoDS_Face face = TopoDS::Face(ex.Current());
//		face_list.push_back(face);
//	}
//	//shape = face_list[2];
//	TopTools_ListOfShape remove_list;
//	remove_list.Append(face_list[12]);
//	//shape = face_list[12];
//	removeFace(shape, remove_list, shape);
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(5, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//
//	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 5, 0), gp_Pnt(5, 5, 0)).Edge();
//
//	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
//	new_bop->AddArgument(edge1);
//	new_bop->AddTool(ProfileFace);
//	new_bop->SetOperation(BOPAlgo_CUT);
//	new_bop->Perform();
//	bool if_md = new_bop->HasModified();
//
//	//测试边数
//	std::vector<TopoDS_Edge> edges;
//	TopExp_Explorer ex;
//	for (ex.Init(ProfileFace, TopAbs_EDGE); ex.More(); ex.Next())
//	{
//		TopoDS_Edge myedge = TopoDS::Edge(ex.Current());
//		edges.push_back(myedge);
//	}
//	int sizee = edges.size();
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	////Annotation use
//	//TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(gp_Pnt(-15, 0, 0), gp_Pnt(-5, 0, 0)).Edge();
//	//TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(15, 0, 0)).Edge();
//
//	//double angle = std::atan2(0, 10);
//	//double length = 1.0;
//	//double add1 = sin(angle + M_PI / 3) * length;
//	//double add2 = sin(angle + M_PI - M_PI / 3) * length;
//	//double add3 = cos(angle + M_PI / 3) * length;
//	//double add4 = cos(angle + M_PI - M_PI / 3) * length;
//
//	//gp_Pnt arrowpt1(-15 + sin(angle + M_PI / 3) * length, 0 + cos(angle + M_PI / 3) * length, 0);
//	//gp_Pnt arrowpt2(-15 + sin(angle + M_PI - M_PI / 3) * length, 0 + cos(angle + M_PI - M_PI / 3) * length, 0);
//	//TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(gp_Pnt(-15, 0, 0), arrowpt1).Edge();
//	//TopoDS_Edge edge4 = BRepBuilderAPI_MakeEdge(gp_Pnt(-15, 0, 0), arrowpt2).Edge();
//	//
//	//gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
//	//gp_Circ c2(ax2, 5);
//	//TopoDS_Edge edge5 = BRepBuilderAPI_MakeEdge(c2).Edge();
//
//	//Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//	//BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
//	//new_bop->AddArgument(edge1);
//	//new_bop->AddTool(edge2);
//	//new_bop->AddTool(edge3);
//	//new_bop->AddTool(edge4);
//	//new_bop->AddTool(edge5);
//
//	//new_bop->SetOperation(BOPAlgo_FUSE);
//	//new_bop->Perform();
//	//shape = new_bop->Shape();
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	std::vector<TopoDS_Edge> edges;
//	edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(15, 10, 0)));
//	//edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(15, 10, 0), gp_Pnt(10, 20, 0)));
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(15, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(15, 10, 0), gp_Pnt(10, 20, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 20, 0), gp_Pnt(0, 20, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 20, 0), gp_Pnt(-5, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(-5, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//	shape = ProfileFace;
//	ShapeAnalysis_Edge anEdgeAnalyser;
//	std::vector<gp_Pnt> point_list;
//	for (int i = 0; i < anEdges.size(); i++)
//	{
//		TopoDS_Edge this_edge = anEdges[i];
//		TopoDS_Vertex aVf = anEdgeAnalyser.FirstVertex(this_edge);
//		gp_Pnt aPf = BRep_Tool::Pnt(aVf);
//		point_list.push_back(aPf);
//	}
//	TopoDS_Face newface;
//	//std::map<gp_Pnt, gp_Pnt> substitude_pt;
//	std::vector<gp_Pnt> substitude_pt;
//	moveEdges(point_list, edges, 10, newface, substitude_pt);
//	
//	shape = newface;
//	return true;
//}

int OCCTest::LocateTarget2Num(std::vector<gp_Pnt> pt_vec, gp_Pnt taget_point)
{
	for (int i = 0; i < pt_vec.size(); i++)
	{
		if (pt_vec[i].X()== taget_point.X()&& pt_vec[i].Y() == taget_point.Y()&& pt_vec[i].Z() == taget_point.Z())
			return i;
	}
	return -1;
}

bool OCCTest::swapSheet(TopoDS_Shape mutter_3d, TopoDS_Shape vatter, TopoDS_Shape& sohn_shape)
{
	//相交
	BRepAlgo_Section my_section(mutter_3d, vatter, Standard_True);
	my_section.Build();
	TopoDS_Shape intersect_e = my_section.Shape();
	TopoDS_Face face1, face2;
	my_section.HasAncestorFaceOn1(intersect_e, face1);
	my_section.HasAncestorFaceOn2(intersect_e, face2);
	return true;
}

bool OCCTest::removeFace(TopoDS_Shape mutter_3d, TopTools_ListOfShape remove_list, TopoDS_Shape& sohn_shape)
{
	BRepAlgoAPI_Defeaturing adf;
	adf.SetShape(mutter_3d);                    // Set the shape
	adf.AddFacesToRemove(remove_list);            // Add faces to remove
	adf.SetRunParallel(true);        // Define the processing mode (parallel or single)
	adf.SetToFillHistory(false);   // Define whether to track the shapes modifications
	adf.Build();

	if (!adf.IsDone())                       // Check for the errors
	{
		// error treatment
		Standard_SStream aSStream;
		adf.DumpErrors(aSStream);
		return false;
	}
	if (adf.HasWarnings())                   // Check for the warnings
	{
		// warnings treatment
		Standard_SStream aSStream;
		adf.DumpWarnings(aSStream);
	}
	sohn_shape = adf.Shape(); // Result shape
	return true;
}

bool OCCTest::removeEdge(std::vector<TopoDS_Shape> remove_list, TopoDS_Shape& sohn_shape)
{
	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
	new_bop->AddArgument(remove_list[0]);
	for (int i = 1; i < remove_list.size(); i++)
	{
		new_bop->AddTool(remove_list[i]);
	}
	new_bop->SetOperation(BOPAlgo_CUT);
	new_bop->Perform();
	sohn_shape = new_bop->Shape();
	return true;
}

bool OCCTest::createPoint(double coordx, double coordy, double coordz, TopoDS_Shape& output_shape)
{
	gp_Pnt point(coordx, coordy, coordz);
	TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(point);
	output_shape = vertex;
	return true;
}

bool OCCTest::moveEdges(std::vector<gp_Pnt> &point_list, std::vector<TopoDS_Edge> edges, double distance, TopoDS_Face& outputshape, std::vector<gp_Pnt>& substitude_pt)
{
	TopoDS_Wire Profile;
	TopoDS_Face face;
	if (point_list.size() == 0 && edges.size() == 0)
		return false;
	TopExp_Explorer ex;

	std::vector<TopoDS_Edge> allEdges;

	for (int i = 0; i < edges.size(); i++)
	{
		std::vector<gp_Pnt> point_list_line;
		std::vector<gp_Pnt> point_list_line1;
		TopoDS_Edge this_edge = edges[i];
		for (ex.Init(this_edge, TopAbs_VERTEX); ex.More(); ex.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(ex.Current());
			gp_Pnt pt = BRep_Tool::Pnt(vertex);
			point_list_line.push_back(pt);
		}
		for (int i = 0; i < point_list_line.size(); i++)
		{
			gp_Pnt pt = point_list_line[i];
			if(substitude_pt.size()!=0)
			{
				int if_exist = LocateTarget2Num(substitude_pt, pt);
				if (if_exist == -1)
				{
					point_list_line1.push_back(pt);
				}
				else
				{
					point_list_line1.push_back(substitude_pt[if_exist + 1]);
				}
			}
			else
			{
				point_list_line1.push_back(pt);
			}
		}
		std::vector<int> numvec;
		for (int i = 0; i < point_list_line1.size(); i++)
		{
			int ptnum = LocateTarget2Num(point_list, point_list_line1[i]);
			numvec.push_back(ptnum);
		}
		sort(numvec.begin(), numvec.end());
		gp_Pnt first_point = point_list[numvec[0]];         //一条边在整个图形中的第一点
		gp_Pnt second_point = point_list[numvec[1]];		//一条边在整个图形中的第二点

		gp_Pnt pre_first_point, post_second_point;          //第一点的前一点，以及第二点的后一点
		if (numvec[0] - 1 < 0)
			pre_first_point = point_list[point_list.size()-1];
		else
			pre_first_point = point_list[numvec[0] - 1];

		if (numvec[1] + 1 == point_list.size())
			post_second_point = point_list[0];
		else
			post_second_point = point_list[numvec[1] + 1];

		//计算两条边的方向
		double distance1 = sqrt(pow((first_point.X() - pre_first_point.X()), 2) + pow(first_point.Y() - pre_first_point.Y(), 2) + pow(first_point.Z() - pre_first_point.Z(), 2));
		gp_Vec vec1((first_point.X() - pre_first_point.X()) / distance1, (first_point.Y() - pre_first_point.Y()) / distance1, (first_point.Z() - pre_first_point.Z()) / distance1);

		double distance2 = sqrt(pow((second_point.X() - post_second_point.X()), 2) + pow(second_point.Y() - post_second_point.Y(), 2) + pow(second_point.Z() - post_second_point.Z(), 2));
		gp_Vec vec2((second_point.X() - post_second_point.X()) / distance1, (second_point.Y() - post_second_point.Y()) / distance1, (second_point.Z() - post_second_point.Z()) / distance1);

		gp_Vec vec3((second_point.X() - first_point.X()) / distance1, (second_point.Y() - first_point.Y()) / distance1, (second_point.Z() - first_point.Z()) / distance1);

		double sintheta1 = sqrt(1 - pow(vec1 * vec3, 2) / pow(vec1.Magnitude() * vec3.Magnitude(), 2));
		double sintheta2 = sqrt(1 - pow(vec2 * vec3, 2) / pow(vec2.Magnitude() * vec3.Magnitude(), 2));
		
		//计算最远点（可能有）
		/*TopoDS_Edge line1 = BRepBuilderAPI_MakeEdge(first_point, first_point.Translated(vec1));
		TopoDS_Edge line2 = BRepBuilderAPI_MakeEdge(second_point, second_point.Translated(vec2));

		double start, end;
		Handle(Geom_Curve) curv1 = BRep_Tool::Curve(line1, start, end);
		Handle(Geom_Curve) curv2 = BRep_Tool::Curve(line2, start, end);*/

		Geom_Line* lin1 = new Geom_Line(first_point, vec1);
		Geom_Line* lin2 = new Geom_Line(second_point, vec2);

		Handle(Geom_Curve) curv1(lin1);
		Handle(Geom_Curve) curv2(lin2);

		GeomAPI_ExtremaCurveCurve compute_inter(curv1, curv2);
		double lowest_dis = compute_inter.LowerDistance();
		double max_dis1;

		if(lowest_dis==0)
		{
			Extrema_ExtCC extre_cc = compute_inter.Extrema();
			gp_Pnt on_cur1, on_cur2;
			compute_inter.Points(1, on_cur1, on_cur2);
			max_dis1 = sqrt(pow(on_cur1.X()- first_point.X(), 2) + pow(on_cur1.Y() - first_point.Y(), 2) + pow(on_cur1.Z() - first_point.Z(), 2));
			if (distance / sintheta1 >= max_dis1)
			{
				substitude_pt.push_back(point_list[numvec[0]]);
				substitude_pt.push_back(on_cur1);
				substitude_pt.push_back(point_list[numvec[1]]);
				substitude_pt.push_back(on_cur1);
				point_list[numvec[0]] = on_cur1;
				for (std::vector<gp_Pnt>::iterator it = point_list.begin(); it != point_list.end(); )
				{
					if (it->X() == point_list[numvec[1]].X() && it->Y() == point_list[numvec[1]].Y() && it->Z() == point_list[numvec[1]].Z())
					{
						it = point_list.erase(it); //不能写成arr.erase(it);
						break;
					}
					else
					{
						++it;
					}
				}
			}
			else
			{
				gp_Pnt newpoint1 = first_point.Translated(distance * vec1 / sintheta1);
				gp_Pnt newpoint2 = second_point.Translated(distance * vec2 / sintheta2);

				substitude_pt.push_back(point_list[numvec[0]]);
				substitude_pt.push_back(newpoint1);
				point_list[numvec[0]] = newpoint1;
				//substitude_pt.insert(std::pair < gp_Pnt, gp_Pnt>(point_list[numvec[0]], newpoint1));
				
				substitude_pt.push_back(point_list[numvec[1]]);
				substitude_pt.push_back(newpoint2);
				point_list[numvec[1]] = newpoint2;
				//substitude_pt.insert(std::pair < gp_Pnt, gp_Pnt>(point_list[numvec[1]], newpoint2));
				
			}
		}
		else
		{
			//计算新线段的两个点
			gp_Pnt newpoint1 = first_point.Translated(distance * vec1 / sintheta1);
			gp_Pnt newpoint2 = second_point.Translated(distance * vec2 / sintheta2);

			point_list[numvec[0]] = newpoint1;
			point_list[numvec[1]] = newpoint2;
		}


		for (int i = 0; i < point_list.size(); i++)
		{
			TopoDS_Edge edge_x;
			if (i != point_list.size() - 1)
			{
				edge_x = BRepBuilderAPI_MakeEdge(point_list[i], point_list[i + 1]);
				allEdges.push_back(edge_x);
			}
			else
			{
				edge_x = BRepBuilderAPI_MakeEdge(point_list[i], point_list[0]);
				allEdges.push_back(edge_x);
			}
		}
		face = OCCBasicTools::GetFaceFromPntList(point_list);
		OCCBasicTools::GetOrderWireFromEdges(allEdges, Profile);
		
		point_list_line1.clear();
	}

	outputshape = face;
	return true;
}

bool OCCTest::healing(TopoDS_Shape& shape, HealOption parameter)
{
	return true;
}
//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//
//	gp_Vec vec1(0, 0, 10);
//	BRepPrimAPI_MakePrism* make_prism1 = new BRepPrimAPI_MakePrism(ProfileFace1, vec1);
//
//	TopoDS_Shape shape1 = make_prism1->Shape();
//
//	gp_Pnt pt(0, 0, 0);
//	TopoDS_Vertex vt = BRepBuilderAPI_MakeVertex(pt);
//
//	TopoDS_Edge tstpt = BRepBuilderAPI_MakeEdge(gp_Pnt(1, 1, 0), gp_Pnt(2, 2, 2)).Edge();
//
//	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
//
//	new_bop->AddArgument(tstpt);
//	new_bop->AddTool(shape1);
//	new_bop->SetOperation(BOPAlgo_CUT21);
//	new_bop->Perform();
//	bool ifm = new_bop->HasModified();
//
//
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	//测试逆矩阵回原来点
//	TopoDS_Vertex oript = BRepBuilderAPI_MakeVertex(gp_Pnt(0, 0, 1)).Vertex();
//	double ori_mtx[16] = { 1,2,3,5,2,1,3,5,3,3,3,5,0,0,0,1 };
//	gp_Trsf trsf;
//	trsf.SetValues(ori_mtx[0], ori_mtx[1], ori_mtx[2], ori_mtx[3], ori_mtx[4], ori_mtx[5], ori_mtx[6], ori_mtx[7], ori_mtx[8], ori_mtx[9], ori_mtx[10], ori_mtx[11]);
//	BRepBuilderAPI_Transform trans(trsf);
//	trans.Perform(oript);
//	bool ifd = trans.IsDone();
//	///////////////////////
//	nero++;
//	WriteLog("success", nero);
//	//test curve U
//	gp_Ax2 ax2(gp_Pnt(8, 8, 0), gp_Dir(0, 0, 1));
//	gp_Circ c2(ax2, 10);
//	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(c2);
//
//	TopoDS_Edge testedge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 2)).Edge();
//	double start, end;
//	Handle(Geom_Curve) testcv = BRep_Tool::Curve(testedge, start, end);
//	double fstu = testcv->FirstParameter();
//	double lstu = testcv->LastParameter();
//
//	TopoDS_Wire Profile,Profile1;
//	std::vector<TopoDS_Edge> anEdges, anEdges1;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//
//	//shape healing test
//	Handle(ShapeFix_Shape) sfs = new ShapeFix_Shape;
//	sfs->Init(ProfileFace);
//	//sfs->SetPrecision(Precision::Confusion());
//	bool pf = sfs->Perform();
//	ShapeExtend_Status status= ShapeExtend_DONE5;
//	pf = sfs->Status(status);
//
//	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges1, Profile1);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile1);
//
//	Handle(Geom_Surface) s1 = BRep_Tool::Surface(ProfileFace);
//	Handle(Geom_Surface) s2 = BRep_Tool::Surface(ProfileFace1);
//	if (s1 == s2)
//	{
//		std::cout << "same geo" << std::endl;
//	}
//	
//	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//	Handle(IntTools_Context) inttol = new IntTools_Context(aAllocator);
//	bool aresame = BOPTools_AlgoTools::AreFacesSameDomain(ProfileFace,ProfileFace1, inttol);
//
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 3, 0), gp_Pnt(7, 1, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(7, 1, 0), gp_Pnt(15, 3, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(15, 1, 0), gp_Pnt(8, 13, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(8, 13, 0), gp_Pnt(3, 3, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//
//	gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
//	gp_Ax2 ax1(gp_Pnt(8, 8, 0), gp_Dir(0, 0, 1));
//	gp_Circ c2(ax2, 10);
//
//
//	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(c2);
//	TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge2);
//
//
//	TopoDS_Face ProfileFace2 = BRepBuilderAPI_MakeFace(wire2);
//
//
//	gp_Vec vec1(0, 0, 10), vec2(0, 0, 20);
//	BRepPrimAPI_MakePrism* make_prism1 = new BRepPrimAPI_MakePrism(ProfileFace2, vec1);
//	BRepPrimAPI_MakePrism* make_prism2 = new BRepPrimAPI_MakePrism(ProfileFace1, vec2);
//
//	BRepPrimAPI_MakeCone* make_cone2 = new BRepPrimAPI_MakeCone(ax1, 7, 0, 20);
//
//	TopoDS_Shape shape1 = make_prism1->Shape();
//	TopoDS_Shape shape2 = make_cone2->Shape();
//	TopoDS_Shape shape3 = make_prism2->Shape();
//
//	std::vector<TopoDS_Shape> list;
//	list.push_back(shape1);
//	list.push_back(shape2);
//	list.push_back(shape3);
//
//	//=================
//	TopTools_ListOfShape total;
//	total.Append(shape1);
//	total.Append(shape2);
//	total.Append(shape3);
//
//	imprintEdge(total, shape);
//	return true;
//}

bool OCCTest::imprintEdge(TopTools_ListOfShape total, TopoDS_Shape&shape)        //std::vector<TopoDS_Shape> list
{
	/*TopTools_ListOfShape aLS;
	for (int i = 0; i < list.size(); i++)
	{
		aLS.Append(list[i]);
	}*/
	BOPAlgo_CellsBuilder aCBuilder;

	aCBuilder.SetArguments(total);
	aCBuilder.Perform();
	if (aCBuilder.HasErrors()) { // check error status
		return false;
	}

	TopTools_ListOfShape aLSToTake;
	TopTools_ListOfShape aLSToAvoid;

	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
	BOPAlgo_BOP* new_bop2 = new BOPAlgo_BOP(aAllocator);
	aLSToTake.Append(total.First());
	TopTools_ListOfShape::Iterator tooliter(total);
	tooliter.Next();
	for (; tooliter.More(); tooliter.Next())
	{
		new_bop->AddArgument(total.First());
		new_bop->AddTool(tooliter.Value());
		new_bop->SetOperation(BOPAlgo_COMMON);
		new_bop->Perform();
		TopoDS_Shape common_shape = new_bop->Shape();

		new_bop2->AddTool(common_shape);
		new_bop2->AddArgument(tooliter.Value());
		new_bop2->SetOperation(BOPAlgo_CUT);
		new_bop2->Perform();
		common_shape = new_bop2->Shape();
		aLSToAvoid.Append(common_shape);
	}
	/*for (int i = 1; i < list.size(); i++)
	{
		new_bop->AddArgument(list[0]);
		new_bop->AddTool(list[i]);
		new_bop->SetOperation(BOPAlgo_COMMON);
		new_bop->Perform();
		TopoDS_Shape common_shape = new_bop->Shape();

		new_bop2->AddTool(common_shape);
		new_bop2->AddArgument(list[i]);
		new_bop2->SetOperation(BOPAlgo_CUT);
		new_bop2->Perform();
		TopoDS_Shape delete_shape = new_bop2->Shape();
		aLSToAvoid.Append(delete_shape);

	}*/

	aCBuilder.AddToResult(aLSToTake, aLSToAvoid,1);

	aCBuilder.RemoveInternalBoundaries();
	shape = aCBuilder.Shape();
	ExportModel(shape, "D:\\imprint.brep");
	return true;
}

bool OCCTest::ExportModel(TopoDS_Shape shape, const std::string & path)
{
	char* curfile = (char*)(path.c_str());
	Standard_CString afile = (Standard_CString)curfile;
	Standard_Boolean ok = BRepTools::Write(shape, afile);
	return true;
}

bool OCCTest::DeleteEdges(const TopTools_IndexedMapOfShape& edges, const bool edgeOnly, TopoDS_Shape &shape)
{
	Handle(BRepTools_ReShape) ReShape = new BRepTools_ReShape;
	//
	for (int e = 1; e <= edges.Extent(); ++e)
	{
		ReShape->Remove(edges(e));

		if (!edgeOnly)
		{
			// Get rid of vertices
			for (TopExp_Explorer exp(edges(e), TopAbs_VERTEX); exp.More(); exp.Next())
			{
				ReShape->Remove(exp.Current());
			}
		}
	}
	//
	shape = ReShape->Apply(shape);
	return true; // Success
}

bool OCCTest::DetachFaces(const TopTools_IndexedMapOfShape& faces, TopoDS_Shape& shape)
{
	Handle(BRepTools_ReShape) ReShape = new BRepTools_ReShape;

	// Detach each face one by one
	for (int f = 1; f <= faces.Extent(); ++f)
	{
		const TopoDS_Shape& current_face = faces(f);

		// Copy face
		BRepBuilderAPI_Copy copier(current_face);
		const TopoDS_Shape& copy = copier.Shape();

		ReShape->Replace(current_face, copy);
	}

	shape = ReShape->Apply(shape);

	return true; // Success
}

bool OCCTest::SmallEdges(TopoDS_Shape& shape, const double len, const double ang_deg)
{
	ShapeBuild_ReShape* m_reShape = new ShapeBuild_ReShape();
	ShapeFix_Wireframe healer(shape);
	healer.SetPrecision(len);
	healer.SetLimitAngle(ang_deg); // Feature angle.
	healer.SetContext(m_reShape);
	//
	healer.FixSmallEdges();

	// Set output.
	shape = healer.Shape();

	//ShapeUpgrade_UnifySameDomain unify(m_input, true, false, true);
	//unify.SetLinearTolerance(len);
	//unify.SetAngularTolerance(ang_deg/180.0*M_PI);
	//unify.Build();

	//// Set output.
	//m_output = unify.Shape();

	return true; // Success.
}

void OCCTest::WriteLog(char* log, int ner)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	FILE* fp;
	fp = fopen("..\\log\\log.txt", "wt");
	fprintf(fp, "loginfo:%d:%d, error%d is:", st.wDay, st.wHour, ner);
	fprintf(fp, log);
	fclose(fp);
}

bool OCCTest::DetachFaces(TopoDS_Shape ent, std::vector<TopoDS_Face> facelist, bool iscreated, TopoDS_Shape& output)
{
	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
	
	if (iscreated == true)
	{
		if (facelist.size() > 1)
		{
			new_bop->AddArgument(facelist[0]);
			for (int i = 1; i < facelist.size(); i++)
			{
				new_bop->AddTool(facelist[i]);
			}
			new_bop->SetOperation(BOPAlgo_FUSE);
			new_bop->Perform();
			output = new_bop->Shape();
		}
	}
	else
	{
		std::vector<TopoDS_Face> faces;
		TopExp_Explorer ex;
		for (ex.Init(ent, TopAbs_FACE); ex.More(); ex.Next())
		{
			TopoDS_Face xface = TopoDS::Face(ex.Current());
			faces.push_back(xface);
		}
		std::vector<TopoDS_Face>::iterator iter;
		for (iter = faces.begin(); iter != faces.end(); ++iter)
		{
			for (int i = 0; i < facelist.size(); i++)
			{
				TopoDS_Face xface = facelist[i];
				if (iter->IsSame(xface))
				{
					iter = faces.erase(iter);
					continue;
				}	
			}
		}
		

		if (faces.size() > 1)
		{
			new_bop->AddArgument(faces[0]);
			for (int i = 1; i < faces.size(); i++)
			{
				new_bop->AddTool(faces[i]);
			}
			new_bop->SetOperation(BOPAlgo_FUSE);
			new_bop->Perform();
			output = new_bop->Shape();
		}
	}
	return true;
}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	std::vector<TopoDS_Edge> edges, edgelist;
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//
//	//建立棱柱
//	gp_Vec vec(0, 0, 10);
//	BRepPrimAPI_MakePrism* make_prism = new BRepPrimAPI_MakePrism(ProfileFace, vec);
//
//	shape = make_prism->Shape();
//	TopExp_Explorer ex;
//	for (ex.Init(shape, TopAbs_EDGE); ex.More(); ex.Next())
//	{
//		TopoDS_Edge xface = TopoDS::Edge(ex.Current());
//		edges.push_back(xface);
//	}
//	edgelist.push_back(edges[0]);
//	edgelist.push_back(edges[1]);
//	edgelist.push_back(edges[2]);
//
//	//DetachFaces(shape, facelist, false, shape);
//
//	Handle(BRepTools_ReShape) ReShape = new BRepTools_ReShape;
//	ReShape->Remove(edges[0]);
//	ReShape->Remove(edges[1]);
//	ReShape->Remove(edges[13]);
//	ReShape->Apply(shape);
//	shape = ReShape->Value(shape);
//
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//
//	std::vector<TopoDS_Edge> anEdges;
//	std::vector<gp_Pnt> vtxs,needvtxs;
//
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> Edges;
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//	gp_Vec vec1(0, 0, 10);
//	TopoDS_Shape box = BRepPrimAPI_MakePrism(ProfileFace1, vec1).Shape();
//	shape = TopoDS::Solid(box);
//	/*for (int i = 0; i < anEdges.size(); i++)
//	{
//		TopExp_Explorer ex;
//		for (ex.Init(anEdges[i], TopAbs_VERTEX); ex.More(); ex.Next())
//		{
//			TopoDS_Vertex xvtx = TopoDS::Vertex(ex.Current());
//			gp_Pnt xpt = BRep_Tool::Pnt(xvtx);
//			vtxs.push_back(xpt);
//		}
//	}*/
//	//for (int i = 0; i < vtxs.size(); i++)
//	//{
//	//	gp_Pnt xvtx = vtxs[i];
//	//	int num = count(vtxs.begin(), vtxs.end(), xvtx);
//	//	/*if (num == 1)
//	//	{
//	//		needvtxs.push_back(xvtx);
//	//	}*/
//	//}
//	TopExp_Explorer ex, ex1;
//	TopoDS_Shape addpt = BRepBuilderAPI_MakeVertex(gp_Pnt(5, 0, 0)).Vertex();
//	TopoDS_Shape addpt1 = BRepBuilderAPI_MakeVertex(gp_Pnt(5, 0, 0)).Vertex();
//	Handle(BRepTools_ReShape) reshape = new BRepTools_ReShape();
//	for (ex1.Init(box, TopAbs_EDGE); ex1.More(); ex1.Next())
//	{
//		TopoDS_Edge xedge = TopoDS::Edge(ex1.Current());
//		anEdges.push_back(xedge);
//	}
//	std::vector<TopoDS_Shape> vtxre;
//	for (ex.Init(anEdges[1], TopAbs_VERTEX); ex.More(); ex.Next())
//	{
//		vtxre.push_back(ex.Current());
//	}
//	reshape->Replace(vtxre[0], addpt);
//	//reshape->Replace(vtxre[1], addpt1);
//
//	TopoDS_Shape t = box;
//	shape = reshape->Apply(t);
//	//shape.set
//	/*bool suc = reshape->IsNewShape(box);
//	shape = reshape->Value(box);*/
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//
//	std::vector<TopoDS_Edge> anEdges;
//	std::vector<gp_Pnt> vtxs, needvtxs;
//
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> Edges;
//	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(gp_Pnt(1, 1, 0), gp_Pnt(2, 2, 2)).Edge();
//	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(gp_Pnt(2, 2, 2), gp_Pnt(4, 3, 3)).Edge();
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(1, 1, 0)));
//	Edges.push_back(edge1);
//	Edges.push_back(edge2);
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(4, 3, 3), gp_Pnt(8, 7, 1)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(8, 7, 1), gp_Pnt(10, 2, 7)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges, Profile);
//
//	TopoDS_Shape addpt = BRepBuilderAPI_MakeVertex(gp_Pnt(2.5, 2, 1.5)).Vertex();
//	Handle(BRepTools_ReShape) reshape = new BRepTools_ReShape();
//
//	TopExp_Explorer ex, ex1;
//	/*for (ex.Init(edge1, TopAbs_VERTEX); ex.More(); ex.Next())
//	{
//		reshape->Replace(ex.Current(), addpt);
//		break;
//	}*/
//	for (ex1.Init(edge2, TopAbs_VERTEX); ex1.More(); ex1.Next())
//	{
//		reshape->Replace(ex1.Current(), addpt);
//		break;
//	}
//	TopoDS_Shape t = Profile;
//	shape = reshape->Apply(t);
//
//	//shape.set
//	/*bool suc = reshape->IsNewShape(box);
//	shape = reshape->Value(box);*/
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	std::map<double, gp_Pnt> testlist;
//	gp_Pnt basept1(5, 5, 5);
//	gp_Pnt basept2(0, 1, 5);
//	gp_Pnt basept3(3, 3, 3);
//	testlist.insert(std::pair<double, gp_Pnt>(8.2, basept1));
//	testlist.insert(std::pair<double, gp_Pnt>(-0.15, basept2));
//	testlist.insert(std::pair<double, gp_Pnt>(3, basept3));
//
//	std::vector<TopoDS_Edge> anEdges;
//	std::vector<gp_Pnt> vtxs, needvtxs;
//
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> Edges;
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//	gp_Vec vec1(0, 0, 10);
//	TopoDS_Shape box = BRepPrimAPI_MakePrism(ProfileFace1, vec1).Shape();
//	
//	gp_Pnt basept(5, 5, 5);
//	gp_Dir basedir(0, 0, 1);
//	Handle(Geom_Plane) geoplane = new Geom_Plane(basept, basedir);
//	TopoDS_Face input_face = BRepBuilderAPI_MakeFace(geoplane, Precision::Confusion());
//
//	BRepAdaptor_Surface adps(input_face);
//
//	TopTools_ListOfShape total;
//	total.Append(box);
//	total.Append(input_face);
//	BOPAlgo_CellsBuilder aCBuilder;
//
//	aCBuilder.SetArguments(total);
//	aCBuilder.Perform();
//	if (aCBuilder.HasErrors()) { // check error status
//		return false;
//	}
//	TopTools_ListOfShape aLSToTake;
//	TopTools_ListOfShape aLSToAvoid;
//	aLSToTake.Append(total.First());
//
//
//	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
//	BOPAlgo_BOP* new_bop1 = new BOPAlgo_BOP(aAllocator);
//	new_bop->AddTool(box);
//	new_bop->AddArgument(input_face);
//	new_bop->SetOperation(BOPAlgo_COMMON);
//	new_bop->Perform();
//	TopoDS_Shape tool = new_bop->Shape();
//
//	aLSToAvoid.Append(tool);
//	aCBuilder.AddToResult(aLSToTake, aLSToAvoid, 1);
//
//	aCBuilder.RemoveInternalBoundaries();
//	shape = aCBuilder.Shape();
//	BRepLProp_SLProps props(adps,2,Precision::Confusion());
//
//	//shape = new_bop->Shape();
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//
//	/*TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> Edges;
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//	gp_Vec vec1(0, 0, 10);
//	TopoDS_Shape box = BRepPrimAPI_MakePrism(ProfileFace1, vec1).Shape();*/
//
//	gp_Pnt pnt(0, 0, 0);
//	BRepPrimAPI_MakeSphere sp(pnt, 10);
//	sp.Build();
//	shape = sp.Shape();
//
//	TopExp_Explorer ex;
//	std::vector<TopoDS_Face> facelist;
//	for (ex.Init(shape, TopAbs_FACE); ex.More(); ex.Next())
//	{
//		TopoDS_Face xface = TopoDS::Face(ex.Current());
//		facelist.push_back(xface);
//	}
//	
//	//BRepMesh_IncrementalMesh bmesh(box, 1, false, 0.05, false);
//	//bmesh.Perform();
//	//shape = bmesh.Shape();
//	///*Standard_CString file = "D:\\Projects\\OpenCas\\BasicOCC\\test.stl";
//	//StlAPI_Writer stlwt;
//	//stlwt.Write(shape, file);*/
//
//	//Standard_Integer aNbNodes = 0;
//	//Standard_Integer aNbTriangles = 0;
//
//	//// calculate total number of the nodes and triangles
//	//for (TopExp_Explorer anExpSF(ProfileFace1, TopAbs_FACE); anExpSF.More(); anExpSF.Next())
//	//{
//	//	TopLoc_Location aLoc;
//	//	Handle(Poly_Triangulation) aTriangulation = BRep_Tool::Triangulation(TopoDS::Face(anExpSF.Current()), aLoc);
//	//	double df = aTriangulation->Deflection();
//	//	if (!aTriangulation.IsNull())
//	//	{
//	//		aNbNodes += aTriangulation->NbNodes();
//	//		aNbTriangles += aTriangulation->NbTriangles();
//	//	}
//	//}
//
//	shape = facelist[0];
//	
//	std::string stlFile = "D:\\Projects\\OpenCas\\BasicOCC\\test.stl";
//
//	/*StlAPI_Writer STLwriter;
//	STLwriter.Write(shape, stlFile.c_str());*/
//
//	OCCBasicTools::SaveStl(shape, stlFile);
//	TopExp_Explorer ex1;
//	std::vector<TopoDS_Face> facelist1;
//	for (ex1.Init(shape, TopAbs_FACE); ex1.More(); ex1.Next())
//	{
//		TopoDS_Face xface = TopoDS::Face(ex1.Current());
//		facelist1.push_back(xface);
//	}
//
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//
//	TopoDS_Wire Profile,Profile1;
//	std::vector<TopoDS_Edge> Edges,Edges1;
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 3, 0), gp_Pnt(3, 6, 0)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 6, 0), gp_Pnt(6, 6, 0)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(6, 6, 0), gp_Pnt(6, 3, 0)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(6, 3, 0), gp_Pnt(3, 3, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges1, Profile1);
//	TopoDS_Face ProfileFace2 = BRepBuilderAPI_MakeFace(Profile1);
//
//	BRepBuilderAPI_MakeFace mkface(ProfileFace1);
//	mkface.Add(Profile1);
//	mkface.Build();
//	shape = mkface.Shape();
//
//	/*BRepBuilderAPI_MakeFace mkface1(TopoDS::Face(shape));
//	mkface1.Add(Profile1);
//	mkface1.Build();
//	shape = mkface1.Shape();*/
//	BRepTools_ReShape re;
//	re.Remove(Profile1);
//	shape = re.Apply(shape);
//
//	TopExp_Explorer ex;
//	for (ex.Init(shape, TopAbs_FACE); ex.More(); ex.Next())
//	{
//		TopoDS_Face xface = TopoDS::Face(ex.Current());
//		shape = xface;
//	}
//
//	/*Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 3, 0), gp_Pnt(3, 13, 0)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 13, 0), gp_Pnt(13, 13, 0)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(13, 13, 0), gp_Pnt(13, 3, 0)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(13, 3, 0), gp_Pnt(3, 3, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges1, Profile1);
//	TopoDS_Face ProfileFace2 = BRepBuilderAPI_MakeFace(Profile1);*/
//
//	/*BRepAlgoAPI_Fuse fuse(ProfileFace1, ProfileFace2);
//	TopExp_Explorer ex(fuse.Shape(), TopAbs_FACE);
//	for (; ex.More(); ex.Next())
//	{
//		shape = ex.Current();
//		break;
//	}*/
//	return true;
//}


//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
//	gp_Elips test_elips(ax2,10,5);
//	TopoDS_Edge ellipse_edge = BRepBuilderAPI_MakeEdge(test_elips);
//	TopoDS_Wire ellipse_wwire = BRepBuilderAPI_MakeWire(ellipse_edge);
//	shape = BRepBuilderAPI_MakeFace(ellipse_wwire).Face();
//
//	TopoDS_Face face = TopoDS::Face(shape);
//	Handle(Geom_Surface) sfs = BRep_Tool::Surface(face);
//	shape = BRepBuilderAPI_MakeFace(sfs,Precision::Confusion()).Face();
//	BRepAdaptor_Surface adps(face);
//	double u1 = adps.FirstUParameter();
//	double u2 = adps.LastUParameter();
//	double v1 = adps.FirstVParameter();
//	double v2 = adps.LastVParameter();
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//
//	TopoDS_Wire Profile,Profile1;
//	std::vector<TopoDS_Edge> Edges,Edges1;
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//
//	gp_Vec vec1(0, 0, 10);
//	TopoDS_Shape box = BRepPrimAPI_MakePrism(ProfileFace1, vec1).Shape();
//
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 0, 0), gp_Pnt(3, 0, 10)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 0, 10), gp_Pnt(3, 10, 10)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 10, 10), gp_Pnt(3, 10, 0)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(3, 10, 0), gp_Pnt(3, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges1, Profile1);
//	TopoDS_Face ProfileFace2 = BRepBuilderAPI_MakeFace(Profile1);
//	gp_Vec vec2(1, 0, 0);
//	TopoDS_Shape box2 = BRepPrimAPI_MakePrism(ProfileFace2, vec2).Shape();
//
//	BOPAlgo_CellsBuilder cb;
//	cb.SetFuzzyValue(Precision::Confusion());
//	cb.SetRunParallel(true);
//	cb.Clear();
//	cb.AddArgument(box);
//	cb.AddArgument(box2);
//	cb.Perform();
//	TopTools_ListOfShape take, avoid;
//	
//	shape = cb.GetAllParts();
//	TopExp_Explorer ex, ex1;
//	std::vector<TopoDS_Shape> faces,faces1;
//	for (ex.Init(shape, TopAbs_SOLID); ex.More(); ex.Next())
//	{
//		TopoDS_Shape xface = ex.Current();
//		faces.push_back(xface);
//	}
//	take.Append(faces[0]);
//	take.Append(faces[2]);
//	avoid.Append(faces[1]);
//	cb.AddAllToResult();
//	cb.RemoveFromResult(avoid, take);
//	cb.RemoveInternalBoundaries();
//
//	shape = cb.Shape();
//
//	return true;
//}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	TopoDS_Wire Profile, Profile1;
//	std::vector<TopoDS_Edge> Edges, Edges1;
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
//	Edges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges, Profile);
//	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile);
//
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 5, -1), gp_Pnt(5, 6, -1)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 6, -1), gp_Pnt(5, 6, 1)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 6, 1), gp_Pnt(5, 5, 1)));
//	Edges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 5, 1), gp_Pnt(5, 5, -1)));
//	OCCBasicTools::GetOrderWireFromEdges(Edges1, Profile1);
//	TopoDS_Face ProfileFace2 = BRepBuilderAPI_MakeFace(Profile1);
//
//	BRepAlgoAPI_IntSS 
//	
//	return true;
//}

bool OCCTest::OffsetWire(TopoDS_Shape shape, double distance, TopoDS_Shape& offsetshape)
{
	BRepOffsetAPI_MakeOffset bmf(TopoDS::Wire(shape), GeomAbs_Intersection, Standard_True);
	bmf.Perform(distance);
	offsetshape = bmf.Shape();
	if (offsetshape.IsNull())
	{
		TopExp_Explorer exl;
		std::vector<gp_Pnt> Pntlist;
		for (exl.Init(shape, TopAbs_VERTEX); exl.More(); exl.Next())
		{
			TopoDS_Vertex xvtx = TopoDS::Vertex(exl.Current());
			gp_Pnt xpnt = BRep_Tool::Pnt(xvtx);
			Pntlist.push_back(xpnt);
		}
		if (Pntlist.size() != 2)
			return false;
		double length = Pntlist[0].Distance(Pntlist[1]);
		gp_Vec vec1(Pntlist[1].X() - Pntlist[0].X(), Pntlist[1].Y() - Pntlist[0].Y(), Pntlist[1].Z() - Pntlist[0].Z());
		gp_Vec vecZ(0, 0, 1);
		gp_Vec vec2 = length * vecZ;
		gp_Vec finalvec = vec1.Crossed(vec2);
		gp_Trsf tf;
		tf.SetTranslation(finalvec * distance);

		BRepBuilderAPI_Transform transform(tf);
		transform.Perform(shape);
		offsetshape = transform.ModifiedShape(shape);
	}

	//BRepOffsetAPI_MakeOffsetShape OffsetMaker2;
	//// Computes offset shape using simple algorithm.
	//OffsetMaker2.PerformBySimple(shape, distance);
	//if (OffsetMaker2.IsDone())
	//	offsetshape = OffsetMaker2.Shape();

	return true;
}

bool OCCTest::Connect(TopTools_ListOfShape shapelist, bool gensolid, TopoDS_Shape& connectshape)
{
	if (shapelist.Size() < 2)
		return false;
	BRepOffsetAPI_ThruSections bts(gensolid);
	for (auto iter : shapelist)
	{
		if (iter.ShapeType() == TopAbs_VERTEX)
			bts.AddVertex(TopoDS::Vertex(iter));
		else if (iter.ShapeType() == TopAbs_WIRE)
			bts.AddWire(TopoDS::Wire(iter));
		else
			continue;
	}
	bts.Build();
	if (bts.IsDone())
		connectshape = bts.Shape();
	return true;

	Handle(Geom_BSplineCurve) Spl;
	TopoDS_Edge ShellOutletEdge = BRepBuilderAPI_MakeEdge(Spl).Edge();
	TopoDS_Wire ShellOutletWire = BRepBuilderAPI_MakeWire(ShellOutletEdge).Wire();
}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	QString name;
//	std::string strPID;
//	void* entity = NULL;
//
//	std::string strFile = "C:\\Users\\DELL\\Documents\\WeChat Files\\wxid_tmwypojkb8f022\\FileStorage\\File\\2021-12\\dongye(1).geomTurbo";
//	ifstream ifile;
//	ifile.open(strFile, ifstream::in);
//	if (!ifile)
//		std::cerr << "open failed" << std::endl;
//	std::string line, word;
//	std::vector<std::vector<double>> points;
//	int count = 0;
//	int pant = 0;
//
//	TopoDS_Compound com;
//	BRep_Builder B;
//	B.MakeCompound(com);
//	std::vector<TopoDS_Shape> shapelist;
//	std::vector<TopoDS_Shape> connectlist;
//
//	int totalline = 0;
//	while (getline(ifile, line))
//	{
//		totalline++;
//		count++;
//		std::vector<string> text;
//		istringstream istrm(line);//创建字符串流
//		while (istrm >> word)     //逐单词读
//			text.push_back(word); //将单词都放入容器
//		std::vector<double> vecx;
//		
//		if (text.size() == 3)
//		{
//			double dTempValue = atof(text[0].c_str());
//			vecx.push_back(dTempValue);
//			dTempValue = atof(text[1].c_str());
//			vecx.push_back(dTempValue);
//			dTempValue = atof(text[2].c_str());
//			vecx.push_back(dTempValue);
//			points.push_back(vecx);
//		}
//		if (count == 108)
//		{
//			pant++;
//			TopoDS_Shape otp;
//			CreatePolyLine(points, otp);
//			TopAbs_ShapeEnum sptp = otp.ShapeType();
//			BRepAdaptor_CompCurve  gc(TopoDS::Wire(otp));
//			double fst = gc.FirstParameter();
//			double lst = gc.LastParameter();
//			shapelist.push_back(otp);
//			if (shapelist.size() == 2 && pant == 2)
//			{
//				BRepBuilderAPI_MakeWire bmw(TopoDS::Wire(shapelist[0]));
//				bmw.Add(TopoDS::Wire(shapelist[1]));
//				bool ifclose = bmw.Shape().Closed();
//				connectlist.push_back(bmw.Shape());
//
//				pant = 0;
//				shapelist.clear();
//			}
//			points.clear();
//			count = 0;
//		}
//	}
//	
//	TopTools_ListOfShape toplist;
//	TopoDS_Shape finalshape;
//
//	for (int i = 0; i < connectlist.size(); i++)
//	{
//		toplist.Append(connectlist[i]);
//		B.Add(com, connectlist[i]);
//		break;
//	}
//
//	//Connect(toplist, true, finalshape);
//	//B.Add(com, bmw.Shape());
//
//	/*TopoDS_Shape offsetwire;
//	OffsetWire(bmw.Shape(), -0.0002, offsetwire);
//	
//	B.Add(com, offsetwire);*/
//
//	shape = com;
//	return true;
//}

bool OCCTest::CreatePolyLine(std::vector<std::vector<double>>& points, TopoDS_Shape& outputshape)
{
	if (points.size() < 2)
	{
		return false;
	}

	TopoDS_Wire finalWire;
	BRep_Builder B;
	B.MakeWire(finalWire);

	for (int i = 0; i < points.size() - 1; ++i)
	{
		gp_Pnt p1(points[i][0], points[i][1], points[i][2]), p2(points[i + 1][0], points[i + 1][1], points[i + 1][2]);
		BRepBuilderAPI_MakeEdge edgeBuilder(p1, p2);
		edgeBuilder.Build();
		if (!edgeBuilder.IsDone())
		{
			return false;
		}

		B.Add(finalWire, edgeBuilder.Shape());
	}
	outputshape = finalWire;

	return true;
}

bool OCCTest::IsShapeGeomSame(const TopoDS_Shape shape1, const TopoDS_Shape shape2, const TopAbs_ShapeEnum type)
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

bool OCCTest::ShapeFindFilterSub(TopoDS_Shape& orishape, TopAbs_ShapeEnum targettype, TopoDS_Shape& filtershape)
{
	

	return true;
}

bool OCCTest::AutoFillGapFaces(TopoDS_Shape& orishape, double allowedarea, TopoDS_Shape& result)
{
	if (orishape.ShapeType() != TopAbs_SHELL)
		return false;
	//先找所有的被单个面拥有的边
	std::vector<TopoDS_Shape> alldupedges, neededges;
	TopExp_Explorer edgeex;
	for (edgeex.Init(orishape, TopAbs_EDGE); edgeex.More(); edgeex.Next())
	{
		alldupedges.push_back(edgeex.Current());
	}
	for (int i = 0; i < alldupedges.size(); i++)
	{
		int count = 0;
		TopoDS_Shape edge1 = alldupedges[i];
		for (int j = 0; j < alldupedges.size(); j++)
		{
			TopoDS_Shape edge2 = alldupedges[j];
			if(IsShapeGeomSame(edge1, edge2, TopAbs_EDGE))
				count++;
		}
		if (count == 1)
			neededges.push_back(edge1);
	}

	//再分别划分组成一个个封闭wire的edge
	TopTools_ListOfShape usedshapes;
	std::vector<std::vector<TopoDS_Shape>> grouped_edges;
	for (int i = 0; i < neededges.size(); i++)
	{
		TopTools_ListOfShape edgeset, listededges;
		TopoDS_Shape edge1 = neededges[i];
		std::vector<TopoDS_Shape> xedgegroup;
		if (usedshapes.Contains(edge1))
			continue;
		
		LoopFindAdjacentEdge(usedshapes, edge1, edgeset, neededges, listededges);

		for (auto iter : listededges)
			xedgegroup.push_back(iter);
		grouped_edges.push_back(xedgegroup);
	}

	//处理每一组独立的封闭wire
	TopTools_ListOfShape filledfaces;
	for (int i = 0; i < grouped_edges.size(); i++)
	{
		std::vector<TopoDS_Shape> xgroup = grouped_edges[i];
		BRepFill_Filling bff;
		for (int j = 0; j < xgroup.size(); j++)
		{
			bff.Add(TopoDS::Edge(xgroup[j]),GeomAbs_C0);
		}
		bff.Build();
		if (!bff.IsDone())
			continue;
		TopoDS_Face fillface = bff.Face();

		GProp_GProps gg;
		BRepGProp::SurfaceProperties(fillface, gg);
		if (gg.Mass() < allowedarea + 0.1)
			filledfaces.Append(bff.Face());
	}
	BRep_Builder b;
	b.MakeShell(TopoDS::Shell(result));
	for (auto iter : filledfaces)
		b.Add(result, iter);
	TopExp_Explorer faceex(orishape, TopAbs_FACE);
	for (; faceex.More(); faceex.Next())
	{
		b.Add(result, faceex.Current());
	}
	ShapeUpgrade_UnifySameDomain unify(result);
	unify.SetAngularTolerance(M_PI / 6);
	unify.Build();
	result = unify.Shape();
	return true;
}

bool OCCTest::LoopFindAdjacentEdge(TopTools_ListOfShape& usedshapes, TopoDS_Shape& startedge, 
	TopTools_ListOfShape& edgeset,
	std::vector<TopoDS_Shape>& neededges, TopTools_ListOfShape& xedgegroup)
{
	TopoDS_Shape edge1 = startedge;
	if (edgeset.Contains(edge1))
		return false;

	ShapeAnalysis_Edge sae1;
	TopoDS_Vertex v11 = sae1.FirstVertex(TopoDS::Edge(edge1));
	TopoDS_Vertex v12 = sae1.LastVertex(TopoDS::Edge(edge1));

	for (int j = 0; j < neededges.size(); j++)
	{
		TopoDS_Shape edge2 = neededges[j];
		if(edge2.IsSame(edge1))
			continue;
		if (usedshapes.Contains(edge2))
			continue;
		ShapeAnalysis_Edge sae2;
		TopoDS_Vertex v21 = sae2.FirstVertex(TopoDS::Edge(edge2));
		TopoDS_Vertex v22 = sae2.LastVertex(TopoDS::Edge(edge2));
		if (IsShapeGeomSame(v11, v21, TopAbs_VERTEX) || IsShapeGeomSame(v11, v22, TopAbs_VERTEX) || 
			IsShapeGeomSame(v12, v21, TopAbs_VERTEX) || IsShapeGeomSame(v12, v22, TopAbs_VERTEX))
		{
			if(!usedshapes.Contains(edge1))
				usedshapes.Append(edge1);
			if (!usedshapes.Contains(edge2))
				usedshapes.Append(edge2);
			if (!edgeset.Contains(edge1))
				edgeset.Append(edge1);
			if (!xedgegroup.Contains(edge1))
				xedgegroup.Append(edge1);
			if (!xedgegroup.Contains(edge2))
				xedgegroup.Append(edge2);
			if (!LoopFindAdjacentEdge(usedshapes, edge2, edgeset, neededges, xedgegroup))
				return false;
		}
	}
	return true;
}

bool OCCTest::shapeFilter(const TopoDS_Shape& origShape, TopAbs_ShapeEnum targetType, TopoDS_Shape& filterShape)
{
	if (origShape.NbChildren() == 1 && origShape.ShapeType() < targetType)
	{
		TopoDS_Iterator iter(origShape);
		return shapeFilter(iter.Value(), targetType, filterShape);
	}
	else if (origShape.ShapeType() == targetType)
	{
		filterShape = origShape;
		return true;
	}
	else
		return false;
}

bool OCCTest::Assembly(TopoDS_Shape toolshape, TopoDS_Shape blankshape, AssembleType type, 
	TopoDS_Shape& targetshape, NCollection_Mat4<double> matrix)
{
	TopoDS_Shape toolshapeowner;
	
	//固定对象是面
	if (blankshape.ShapeType() == TopAbs_SHELL)
	{
		TopoDS_Face blankface;
		TopoDS_Iterator itera(blankshape);
		for (; itera.More(); itera.Next())
		{
			if (itera.Value().ShapeType() == TopAbs_FACE)
			{
				blankface = TopoDS::Face(itera.Value());
				break;
			}
		}
		if (blankface.IsNull())
			return false;

		Handle(Geom_Surface) blankgeomsurface = BRep_Tool::Surface(blankface);
		BRepAdaptor_Surface bas(blankface);
		GeomAbs_SurfaceType blanksurfacetype = bas.GetType();
		//固定面是平面
		if (blanksurfacetype == GeomAbs_Plane)
		{
			//耦合对象是面
			if (toolshape.ShapeType() == TopAbs_SHELL)
			{
				AssemblyPlaneFaceFace(toolshape, bas,
					blankgeomsurface, targetshape, type, matrix, toolshapeowner);
			}
			//耦合对象是边
			else if (toolshape.ShapeType() == TopAbs_EDGE || toolshape.ShapeType() == TopAbs_WIRE)
			{
				AssemblyPlaneFaceEdge(toolshape, bas, blankgeomsurface, type, matrix, toolshapeowner, targetshape);
			}
			//耦合对象是点
			else if (toolshape.ShapeType() == TopAbs_VERTEX)
			{
				AssemblyPlaneFaceVertex(bas, toolshape, matrix, targetshape, toolshapeowner, blankgeomsurface, type);
			}
		}
		//固定面是圆柱面
		else if (blanksurfacetype == GeomAbs_Cylinder)
		{
			//耦合对象是面
			if (toolshape.ShapeType() == TopAbs_SHELL)
			{
				AssemblyCylinderFaceFace(toolshape, bas, targetshape, type, matrix, toolshapeowner);
			}
			//耦合对象是边
			else if (toolshape.ShapeType() == TopAbs_EDGE || toolshape.ShapeType() == TopAbs_WIRE)
			{
				AssemblyCylinderFaceEdge(toolshape, bas, type, matrix, toolshapeowner, targetshape);
			}
			//耦合对象是点
			else if (toolshape.ShapeType() == TopAbs_VERTEX)
			{
				AssemblyCylinderFaceVertex(bas, toolshape, matrix, targetshape, toolshapeowner, type);
			}
		}
		//固定面是球面
		else if (blanksurfacetype == GeomAbs_Sphere)
		{
			//耦合对象是面
			if (toolshape.ShapeType() == TopAbs_SHELL)
			{
				AssemblySphereFaceFace(toolshape, bas, targetshape, type, matrix, toolshapeowner);
			}
			//耦合对象是边
			else if (toolshape.ShapeType() == TopAbs_EDGE || toolshape.ShapeType() == TopAbs_WIRE)
			{
				AssemblySphereFaceEdge(toolshape, bas, type, matrix, toolshapeowner, targetshape);
			}
			//耦合对象是点
			else if (toolshape.ShapeType() == TopAbs_VERTEX)
			{
				gp_Sphere plane = bas.Sphere();
				gp_Ax3 blankposition = plane.Position();

				gp_Pnt blankbasepoint = blankposition.Location();
				gp_Dir blankbasedir = blankposition.Direction();

				gp_Pnt toolbasepoint = BRep_Tool::Pnt(TopoDS::Vertex(toolshape));

				//相合
				if (type == AssembleType::coincidence)
				{
					//空间变换
					gp_Trsf transf;
					transf.SetTranslation(toolbasepoint, blankbasepoint);

					transf.GetMat4(matrix);

					BRepBuilderAPI_Transform bat(transf);
					bat.Perform(toolshapeowner);
					if (!bat.IsDone())
						return false;
					targetshape = bat.Shape();
				}
			}
		}
	}
	//固定对象是线
	else if (blankshape.ShapeType() == TopAbs_EDGE || toolshape.ShapeType() == TopAbs_WIRE)
	{
		if (blankshape.ShapeType() == TopAbs_WIRE)
		{
			TopoDS_Iterator itera(blankshape);
			for (; itera.More(); itera.Next())
			{
				if (itera.Value().ShapeType() == TopAbs_EDGE)
				{
					blankshape = itera.Value();
					break;
				}
			}
		}
		if (blankshape.ShapeType() != TopAbs_EDGE)
			return false;
		TopoDS_Edge blankedge = TopoDS::Edge(blankshape);
		double first, last;
		Handle(Geom_Curve) blankgeomcurve = BRep_Tool::Curve(blankedge, first, last);
		BRepAdaptor_Curve bas(blankedge);
		GeomAbs_CurveType blankscurvetype = bas.GetType();
		//固定边是直边
		if (blankscurvetype == GeomAbs_Line)
		{
			//耦合对象是面
			if (toolshape.ShapeType() == TopAbs_SHELL)
			{
				AssemblyLineFace(toolshape, bas, targetshape, type, matrix, toolshapeowner);
			}
			//耦合对象是边
			else if (toolshape.ShapeType() == TopAbs_EDGE || toolshape.ShapeType() == TopAbs_WIRE)
			{
				AssemblyLineEdge(toolshape, bas, type, matrix, toolshapeowner, targetshape);
			}
			//耦合对象是点
			else if (toolshape.ShapeType() == TopAbs_VERTEX)
			{
				gp_Lin line = bas.Line();
				gp_Ax1 blankposition = line.Position();

				gp_Pnt blankbasepoint = blankposition.Location();
				gp_Dir blankbasedir = blankposition.Direction();

				Handle(Geom_Line) blankgeomline = new Geom_Line(blankbasepoint, blankbasedir);

				gp_Pnt toolpoint = BRep_Tool::Pnt(TopoDS::Vertex(toolshape));
				GeomAPI_ProjectPointOnCurve gppoc(toolpoint, blankgeomline);
				if (gppoc.NbPoints() < 1)
					return false;
				gp_Pnt toolprojpoint = gppoc.Point(1);
				//相合
				if (type == AssembleType::coincidence)
				{
					//空间变换
					gp_Trsf transf;
					transf.SetTranslation(toolpoint, toolprojpoint);

					transf.GetMat4(matrix);

					BRepBuilderAPI_Transform bat(transf);
					bat.Perform(toolshapeowner);
					if (!bat.IsDone())
						return false;
					targetshape = bat.Shape();
				}
			}
		}
		//固定边是平面线
		else
		{
			//耦合对象是面
			if (toolshape.ShapeType() == TopAbs_SHELL)
			{
				AssemblyPlanarLineFace(toolshape, blankedge, targetshape, type, matrix, toolshapeowner);
			}
			//耦合对象是边
			else if (toolshape.ShapeType() == TopAbs_EDGE || toolshape.ShapeType() == TopAbs_WIRE)
			{
				AssemblyPlanarLineEdge(toolshape, blankedge, type, matrix, toolshapeowner, targetshape);
			}
			//耦合对象是点
			else if (toolshape.ShapeType() == TopAbs_VERTEX)
			{
				double first, last;
				Handle(Geom_Curve) blankgeomcurve = BRep_Tool::Curve(blankedge, first, last);
				gp_Pnt blankbasepoint;
				gp_Dir blankbasedir;
				if (!IsEdgePlanar(blankgeomcurve, first, last, blankbasepoint, blankbasedir))
					return false;
				GC_MakePlane blankgcmp(blankgeomcurve->Value(first), blankgeomcurve->Value(last), blankbasepoint);
				Handle(Geom_Plane) blankgeplane = blankgcmp.Value();

				gp_Pnt toolpoint = BRep_Tool::Pnt(TopoDS::Vertex(toolshape));
				//投影点到blank所在平面
				GeomAPI_ProjectPointOnSurf gppos(toolpoint, blankgeplane);
				if (!gppos.IsDone() && gppos.NbPoints() < 1)
					return false;
				gp_Pnt toolprojpoint = gppos.Point(1);
				//相合
				if (type == AssembleType::coincidence)
				{
					//空间变换
					gp_Trsf transf;
					transf.SetTranslation(toolpoint, toolprojpoint);

					transf.GetMat4(matrix);

					BRepBuilderAPI_Transform bat(transf);
					bat.Perform(toolshapeowner);
					if (!bat.IsDone())
						return false;
					targetshape = bat.Shape();
				}
			}
		}
	}
	else if (blankshape.ShapeType() == TopAbs_VERTEX)
	{
		//耦合对象是面
		if (toolshape.ShapeType() == TopAbs_SHELL)
		{
			AssemblyVtxFace(toolshape, blankshape, targetshape, type, matrix, toolshapeowner);
		}
		//耦合对象是边
		else if (toolshape.ShapeType() == TopAbs_EDGE || toolshape.ShapeType() == TopAbs_WIRE)
		{
			AssemblyVtxEdge(toolshape, blankshape, type, matrix, toolshapeowner, targetshape);
		}
		//耦合对象是点
		else if (toolshape.ShapeType() == TopAbs_VERTEX)
		{
			gp_Pnt blankpoint = BRep_Tool::Pnt(TopoDS::Vertex(blankshape));
			gp_Pnt toolpoint = BRep_Tool::Pnt(TopoDS::Vertex(toolshape));
			//相合
			if (type == AssembleType::coincidence)
			{
				//空间变换
				gp_Trsf transf;
				transf.SetTranslation(toolpoint, blankpoint);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			}
		}
	}
}

bool OCCTest::AssemblyVtxEdge(TopoDS_Shape toolshape, TopoDS_Shape blankshape,
	AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape)

{
	gp_Pnt blankpoint = BRep_Tool::Pnt(TopoDS::Vertex(blankshape));

	if (toolshape.ShapeType() != TopAbs_EDGE)
		return false;
	TopoDS_Edge tooledge = TopoDS::Edge(toolshape);
	BRepAdaptor_Curve bastool(tooledge);
	double first, last;
	Handle(Geom_Curve) bascurve = BRep_Tool::Curve(tooledge, first, last);
	gp_Pnt toolcurvept1 = bascurve->Value(first);
	gp_Pnt toolcurvept2 = bascurve->Value(last);

	//耦合对象是直线
	if (bastool.GetType() == GeomAbs_Line)
	{
		gp_Lin line = bastool.Line();
		gp_Pnt toolbasepoint = line.Location();
		gp_Dir toolbasedir = line.Direction();

		Handle(Geom_Line) toolgeomline = new Geom_Line(toolbasepoint, toolbasedir);

		//投影blank点到tool所在线
		GeomAPI_ProjectPointOnCurve gppoc(blankpoint, toolgeomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//点与平面线相合、对齐、同向的操作是一样的
		//if (type == AssembleType::coincidence)
		//{
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankpoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合对象是平面线
	else
	{
		gp_Pnt toolbasepoint;
		gp_Dir toolbasedir;
		if (IsEdgePlanar(bascurve, first, last, toolbasepoint, toolbasedir))
		{
			GC_MakePlane blankgcmp(bascurve->Value(first), bascurve->Value(last), toolbasepoint);
			Handle(Geom_Plane) toolgeplane = blankgcmp.Value();
			//投影tool平面点到blank线所在平面
			GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, toolgeplane);
			if (!gppos.IsDone() && gppos.NbPoints() < 1)
				return false;
			gp_Pnt toolprojpoint = gppos.Point(1);

			//点与平面线相合、对齐、同向的操作是一样的
			//if (type == AssembleType::coincidence)
			//{
				//空间变换
				gp_Trsf transf;
				transf.SetTranslation(toolprojpoint, blankpoint);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			//}
		}
	}
}

bool OCCTest::AssemblyVtxFace(TopoDS_Shape toolshape, TopoDS_Shape blankshape, TopoDS_Shape& targetshape, AssembleType type
	, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner)
{
	gp_Pnt blankpoint = BRep_Tool::Pnt(TopoDS::Vertex(blankshape));

	TopoDS_Face toolface;
	TopoDS_Iterator iteratool(toolshape);
	for (; iteratool.More(); iteratool.Next())
	{
		if (iteratool.Value().ShapeType() == TopAbs_FACE)
		{
			toolface = TopoDS::Face(iteratool.Value());
			break;
		}
	}
	if (toolface.IsNull())
		return false;
	BRepAdaptor_Surface bastool(toolface);
	GeomAbs_SurfaceType toolsurfacetype = bastool.GetType();
	//耦合面是平面
	if (toolsurfacetype == GeomAbs_Plane)
	{
		gp_Pln toolplane = bastool.Plane();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Surface) toolgeomsurface = BRep_Tool::Surface(toolface);

		//投影blank点到tool面上
		GeomAPI_ProjectPointOnSurf gppos(blankpoint, toolgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合
		//if (type == AssembleType::coincidence)
		//{
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankpoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合面是圆柱面
	else if (toolsurfacetype == GeomAbs_Cylinder)
	{
		gp_Cylinder toolplane = bastool.Cylinder();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Line) toolgeomline = new Geom_Line(toolbasepoint, toolbasedir);

		//投影blank点到tool圆柱面中轴线上
		GeomAPI_ProjectPointOnCurve gppoc(blankpoint, toolgeomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合
		//if (type == AssembleType::coincidence)
		//{
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankpoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合面是圆锥面
	else if (toolsurfacetype == GeomAbs_Cone)
	{
		gp_Cone toolplane = bastool.Cone();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Line) toolgeomline = new Geom_Line(toolbasepoint, toolbasedir);

		//投影blank点到tool圆锥面中轴线上
		GeomAPI_ProjectPointOnCurve gppoc(blankpoint, toolgeomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合
		//if (type == AssembleType::coincidence)
		//{
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankpoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合面是球面
	else if (toolsurfacetype == GeomAbs_Sphere)
	{
		gp_Sphere toolplane = bastool.Sphere();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//相合
		//if (type == AssembleType::coincidence)
		//{
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolbasepoint, blankpoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
}

bool OCCTest::AssemblyPlanarLineEdge(TopoDS_Shape toolshape, TopoDS_Edge blankedge,
	AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape)

{
	double first, last;
	Handle(Geom_Curve) blankgeomcurve = BRep_Tool::Curve(blankedge, first, last);
	gp_Pnt blankbasepoint;
	gp_Dir blankbasedir;
	if (!IsEdgePlanar(blankgeomcurve, first, last, blankbasepoint, blankbasedir))
		return false;
	GC_MakePlane blankgcmp(blankgeomcurve->Value(first), blankgeomcurve->Value(last), blankbasepoint);
	Handle(Geom_Plane) blankgeplane = blankgcmp.Value();

	if (toolshape.ShapeType() != TopAbs_EDGE)
		return false;
	TopoDS_Edge tooledge = TopoDS::Edge(toolshape);
	BRepAdaptor_Curve bastool(tooledge);
	double first, last;
	Handle(Geom_Curve) bascurve = BRep_Tool::Curve(tooledge, first, last);
	gp_Pnt toolcurvept1 = bascurve->Value(first);
	gp_Pnt toolcurvept2 = bascurve->Value(last);

	//耦合对象是直线
	if (bastool.GetType() == GeomAbs_Line)
	{
		gp_Lin line = bastool.Line();
		gp_Pnt toolbasepoint = line.Location();
		gp_Dir toolbasedir = line.Direction();

		//投影点到blank所在平面
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeplane);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//1相合
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合对象是平面线
	else
	{
		gp_Pnt toolbasepoint;
		gp_Dir toolbasedir;
		if (IsEdgePlanar(bascurve, first, last, toolbasepoint, toolbasedir))
		{
			//投影tool平面点到blank线所在平面
			GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeplane);
			if (!gppos.IsDone() && gppos.NbPoints() < 1)
				return false;
			gp_Pnt toolprojpoint = gppos.Point(1);

			//相合
			if (type == AssembleType::coincidence)
			{
				gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
				gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
				//空间变换
				gp_Trsf transf;
				transf.SetTransformation(toolorigin, toolterminal);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			}
		}
	}
}

bool OCCTest::AssemblyPlanarLineFace(TopoDS_Shape toolshape, TopoDS_Edge blankedge, TopoDS_Shape& targetshape,
	AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner)
{
	double first, last;
	Handle(Geom_Curve) blankgeomcurve = BRep_Tool::Curve(blankedge, first, last);
	gp_Pnt blankbasepoint;
	gp_Dir blankbasedir;
	if (!IsEdgePlanar(blankgeomcurve, first, last, blankbasepoint, blankbasedir))
		return false;
	GC_MakePlane blankgcmp(blankgeomcurve->Value(first), blankgeomcurve->Value(last), blankbasepoint);
	Handle(Geom_Plane) blankgeplane = blankgcmp.Value();

	TopoDS_Face toolface;
	TopoDS_Iterator iteratool(toolshape);
	for (; iteratool.More(); iteratool.Next())
	{
		if (iteratool.Value().ShapeType() == TopAbs_FACE)
		{
			toolface = TopoDS::Face(iteratool.Value());
			break;
		}
	}
	if (toolface.IsNull())
		return false;
	BRepAdaptor_Surface bastool(toolface);
	GeomAbs_SurfaceType toolsurfacetype = bastool.GetType();
	//耦合面是平面
	if (toolsurfacetype == GeomAbs_Plane)
	{
		gp_Pln toolplane = bastool.Plane();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Surface) toolgeomsurface = BRep_Tool::Surface(toolface);

		//投影tool平面点到blank曲线所在平面上
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeplane);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		////相合
		//if (type == AssembleType::coincidence)
		//{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合面是圆柱面
	else if (toolsurfacetype == GeomAbs_Cylinder)
	{
		gp_Cylinder toolplane = bastool.Cylinder();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影tool中轴线点到blank曲线所在平面上
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeplane);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//相合
		if (type == AssembleType::coincidence)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是圆锥面
	else if (toolsurfacetype == GeomAbs_Cone)
	{
		gp_Cone toolplane = bastool.Cone();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影tool中轴线点到blank曲线所在平面上
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeplane);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//相合
		if (type == AssembleType::coincidence)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else if (type == AssembleType::orient)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是球面
	else if (toolsurfacetype == GeomAbs_Sphere)
	{
		gp_Sphere toolplane = bastool.Sphere();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影tool中轴线点到blank曲线所在平面上
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeplane);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//2相合
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
}

bool OCCTest::AssemblyLineEdge(TopoDS_Shape toolshape, BRepAdaptor_Curve bas,
	AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape)

{
	gp_Lin line = bas.Line();
	gp_Ax1 blankposition = line.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	Handle(Geom_Line) blankgeomline = new Geom_Line(blankbasepoint, blankbasedir);

	if (toolshape.ShapeType() != TopAbs_EDGE)
		return false;
	TopoDS_Edge tooledge = TopoDS::Edge(toolshape);
	BRepAdaptor_Curve bastool(tooledge);
	double first, last;
	Handle(Geom_Curve) bascurve = BRep_Tool::Curve(tooledge, first, last);
	gp_Pnt toolcurvept1 = bascurve->Value(first);
	gp_Pnt toolcurvept2 = bascurve->Value(last);

	//耦合对象是直线
	if (bastool.GetType() == GeomAbs_Line)
	{
		gp_Lin line = bastool.Line();
		gp_Pnt toolbasepoint = line.Location();
		gp_Dir toolbasedir = line.Direction();

		//投影点到tool线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, blankgeomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合
		if (type == AssembleType::coincidence)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		//对齐和同向
		else
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolbasepoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合对象是平面线
	else
	{
		gp_Pnt toolbasepoint;
		gp_Dir toolbasedir;
		if (IsEdgePlanar(bascurve, first, last, toolbasepoint, toolbasedir))
		{
			GC_MakePlane gcmp(toolcurvept1, toolcurvept2, toolbasepoint);
			Handle(Geom_Plane) geplane = gcmp.Value();
			//投影点到tool平面所在平面
			GeomAPI_ProjectPointOnSurf gppos(blankbasepoint, geplane);
			if (!gppos.IsDone() && gppos.NbPoints() < 1)
				return false;
			gp_Pnt toolprojpoint = gppos.Point(1);

			gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
				toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

			//相合和对齐
			if (type == AssembleType::coincidence || type == AssembleType::align)
			{
				gp_Ax3 toolorigin(toolprojpoint, toolbasedir);
				gp_Ax3 toolterminal(blankbasepoint, toolnewdir);
				//空间变换
				gp_Trsf transf;
				transf.SetTransformation(toolorigin, toolterminal);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			}
			else
			{
				gp_Ax3 toolorigin(toolprojpoint, toolbasedir);
				gp_Ax3 toolterminal(blankbasepoint, blankbasedir);
				//空间变换
				gp_Trsf transf;
				transf.SetTransformation(toolorigin, toolterminal);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			}
		}
	}
}

bool OCCTest::AssemblyLineFace(TopoDS_Shape toolshape, BRepAdaptor_Curve bas
	, TopoDS_Shape& targetshape, AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner)
{
	gp_Lin line = bas.Line();
	gp_Ax1 blankposition = line.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	Handle(Geom_Line) blankgeomline = new Geom_Line(blankbasepoint, blankbasedir);

	TopoDS_Face toolface;
	TopoDS_Iterator iteratool(toolshape);
	for (; iteratool.More(); iteratool.Next())
	{
		if (iteratool.Value().ShapeType() == TopAbs_FACE)
		{
			toolface = TopoDS::Face(iteratool.Value());
			break;
		}
	}
	if (toolface.IsNull())
		return false;
	BRepAdaptor_Surface bastool(toolface);
	GeomAbs_SurfaceType toolsurfacetype = bastool.GetType();
	//耦合面是平面
	if (toolsurfacetype == GeomAbs_Plane)
	{
		gp_Pln toolplane = bastool.Plane();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Surface) toolgeomsurface = BRep_Tool::Surface(toolface);

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//投影blank点到tool面
		GeomAPI_ProjectPointOnSurf gppos(blankbasepoint, toolgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合和对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是圆柱面
	else if (toolsurfacetype == GeomAbs_Cylinder)
	{
		gp_Cylinder toolplane = bastool.Cylinder();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Line) geomline = new Geom_Line(toolbasepoint, toolbasedir);

		//投影点到tool点到blank线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, blankgeomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合和对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是圆锥面
	else if (toolsurfacetype == GeomAbs_Cone)
	{
		gp_Cone toolplane = bastool.Cone();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Line) geomline = new Geom_Line(toolbasepoint, toolbasedir);

		//投影点到tool点到blank线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, blankgeomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//3相合
		//if (type == AssembleType::coincidence)
		//{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合面是球面
	else if (toolsurfacetype == GeomAbs_Sphere)
	{
		gp_Sphere toolplane = bastool.Sphere();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影点到tool点到blank线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, blankgeomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//3相合
		//if (type == AssembleType::coincidence)
		//{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);
			
			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
}

bool OCCTest::AssemblySphereFaceEdge(TopoDS_Shape toolshape, BRepAdaptor_Surface bas,
	AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape)
{
	gp_Sphere plane = bas.Sphere();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	if (toolshape.ShapeType() != TopAbs_EDGE)
		return false;
	TopoDS_Edge tooledge = TopoDS::Edge(toolshape);
	BRepAdaptor_Curve bastool(tooledge);
	double first, last;
	Handle(Geom_Curve) bascurve = BRep_Tool::Curve(tooledge, first, last);
	gp_Pnt toolcurvept1 = bascurve->Value(first);
	gp_Pnt toolcurvept2 = bascurve->Value(last);

	//耦合对象是直线
	if (bastool.GetType() == GeomAbs_Line)
	{
		gp_Lin line = bastool.Line();
		gp_Pnt toolbasepoint = line.Location();
		gp_Dir toolbasedir = line.Direction();

		//投影点到tool线
		GeomAPI_ProjectPointOnCurve gppoc(blankbasepoint, bascurve);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合、对齐、同向一样
		//if (type == AssembleType::coincidence)
		//{
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankbasepoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合对象是平面线
	else
	{
		gp_Pnt toolbasepoint;
		gp_Dir toolbasedir;
		if (IsEdgePlanar(bascurve, first, last, toolbasepoint, toolbasedir))
		{
			GC_MakePlane gcmp(toolcurvept1, toolcurvept2, toolbasepoint);
			Handle(Geom_Plane) geplane = gcmp.Value();
			//投影点到tool平面所在平面
			GeomAPI_ProjectPointOnSurf gppos(blankbasepoint, geplane);
			if (!gppos.IsDone() && gppos.NbPoints() < 1)
				return false;
			gp_Pnt toolprojpoint = gppos.Point(1);

			//相合、对齐、同向一样
			//if (type == AssembleType::coincidence)
			//{
				//空间变换
				gp_Trsf transf;
				transf.SetTranslation(toolprojpoint, blankbasepoint);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			//}
		}
	}
}

bool OCCTest::AssemblySphereFaceFace(TopoDS_Shape toolshape, BRepAdaptor_Surface bas
	, TopoDS_Shape& targetshape, AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner)
{
	gp_Sphere plane = bas.Sphere();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	TopoDS_Face toolface;
	TopoDS_Iterator iteratool(toolshape);
	for (; iteratool.More(); iteratool.Next())
	{
		if (iteratool.Value().ShapeType() == TopAbs_FACE)
		{
			toolface = TopoDS::Face(iteratool.Value());
			break;
		}
	}
	if (toolface.IsNull())
		return false;
	BRepAdaptor_Surface bastool(toolface);
	GeomAbs_SurfaceType toolsurfacetype = bastool.GetType();
	//耦合面是平面
	if (toolsurfacetype == GeomAbs_Plane)
	{
		gp_Pln toolplane = bastool.Plane();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Surface) toolgeomsurface = BRep_Tool::Surface(toolface);

		//投影blank球心点到tool面
		GeomAPI_ProjectPointOnSurf gppos(blankbasepoint, toolgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankbasepoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是圆柱面
	else if (toolsurfacetype == GeomAbs_Cylinder)
	{
		gp_Cylinder toolplane = bastool.Cylinder();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Line) geomline = new Geom_Line(toolbasepoint, toolbasedir);

		//投影点到blank面的中轴线
		GeomAPI_ProjectPointOnCurve gppoc(blankbasepoint, geomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合和对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankbasepoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是圆锥面
	else if (toolsurfacetype == GeomAbs_Cone)
	{
		gp_Cone toolplane = bastool.Cone();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		Handle(Geom_Line) geomline = new Geom_Line(toolbasepoint, toolbasedir);

		//投影点到blank面的中轴线
		GeomAPI_ProjectPointOnCurve gppoc(blankbasepoint, geomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//对齐和同向一样
		if (type == AssembleType::orient || type == AssembleType::align)
		{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTranslation(toolprojpoint, blankbasepoint);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是球面
	else if (toolsurfacetype == GeomAbs_Sphere)
	{
		gp_Sphere toolplane = bastool.Sphere();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//相合、对齐、同向都一样
		//if (type == AssembleType::coincidence)
		//{
			//空间变换
			gp_Trsf transf;

			transf.GetMat4(matrix);

			transf.SetTranslation(toolbasepoint, blankbasepoint);
			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
}

bool OCCTest::AssemblyCylinderFaceVertex(BRepAdaptor_Surface bas, TopoDS_Shape toolshape, NCollection_Mat4<double> matrix,
	TopoDS_Shape& targetshape, TopoDS_Shape toolshapeowner, AssembleType type)
{
	gp_Cylinder plane = bas.Cylinder();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	Handle(Geom_Line) geomline = new Geom_Line(blankbasepoint, blankbasedir);

	gp_Pnt toolbasepoint = BRep_Tool::Pnt(TopoDS::Vertex(toolshape));
	//投影点到blank面的中轴线
	GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, geomline);
	if (gppoc.NbPoints() < 1)
		return false;
	gp_Pnt toolprojpoint = gppoc.Point(1);

	//相合和对齐一样
	if (type == AssembleType::coincidence || type == AssembleType::align)
	{
		gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
		//空间变换
		gp_Trsf transf;
		transf.SetTransformation(blankposition, toolterminal);

		transf.GetMat4(matrix);

		BRepBuilderAPI_Transform bat(transf);
		bat.Perform(toolshapeowner);
		if (!bat.IsDone())
			return false;
		targetshape = bat.Shape();
	}
}

bool OCCTest::AssemblyCylinderFaceEdge(TopoDS_Shape toolshape, BRepAdaptor_Surface bas,
	AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape)
{
	gp_Cylinder plane = bas.Cylinder();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	Handle(Geom_Line) geomline = new Geom_Line(blankbasepoint, blankbasedir);

	if (toolshape.ShapeType() == TopAbs_WIRE)
	{
		TopoDS_Iterator itera(toolshape);
		for (; itera.More(); itera.Next())
		{
			if (itera.Value().ShapeType() == TopAbs_EDGE)
			{
				toolshape = itera.Value();
				break;
			}
		}
	}
	if (toolshape.ShapeType() != TopAbs_EDGE)
		return false;
	TopoDS_Edge tooledge = TopoDS::Edge(toolshape);
	BRepAdaptor_Curve bastool(tooledge);
	double first, last;
	Handle(Geom_Curve) bascurve = BRep_Tool::Curve(tooledge, first, last);
	//耦合对象是直线
	if (bastool.GetType() == GeomAbs_Line)
	{
		gp_Lin line = bastool.Line();

		gp_Pnt toolbasepoint = line.Location();
		gp_Dir toolbasedir = line.Direction();
		gp_Ax3 toolorigin(toolbasepoint, toolbasedir);

		//投影点到blank面的中轴线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, geomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合和对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{	
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合对象是平面线
	else if (bastool.GetType() != GeomAbs_Line)
	{
		gp_Pnt toolbasepoint;
		gp_Dir toolbasedir;
		if (IsEdgePlanar(bascurve, first, last, toolbasepoint, toolbasedir))
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			//投影点到blank面的中轴线
			GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, geomline);
			if (gppoc.NbPoints() < 1)
				return false;
			gp_Pnt toolprojpoint = gppoc.Point(1);

			gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
				toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

			//相合
			if (type == AssembleType::coincidence)
			{
				gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
				//空间变换
				gp_Trsf transf;
				transf.SetTransformation(toolorigin, toolterminal);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			}
			else
			{
				gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
				//空间变换
				gp_Trsf transf;
				transf.SetTransformation(toolorigin, toolterminal);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			}
		}
	}
}

bool OCCTest::AssemblyCylinderFaceFace(TopoDS_Shape toolshape, BRepAdaptor_Surface bas
	, TopoDS_Shape& targetshape, AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner)
{
	gp_Cylinder plane = bas.Cylinder();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	Handle(Geom_Line) geomline = new Geom_Line(blankbasepoint, blankbasedir);

	TopoDS_Face toolface;
	TopoDS_Iterator iteratool(toolshape);
	for (; iteratool.More(); iteratool.Next())
	{
		if (iteratool.Value().ShapeType() == TopAbs_FACE)
		{
			toolface = TopoDS::Face(iteratool.Value());
			break;
		}
	}
	if (toolface.IsNull())
		return false;
	BRepAdaptor_Surface bastool(toolface);
	GeomAbs_SurfaceType toolsurfacetype = bastool.GetType();
	//耦合面是平面
	if (toolsurfacetype == GeomAbs_Plane)
	{
		gp_Pln toolplane = bastool.Plane();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//投影点到blank面的中轴线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, geomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合
		if (type == AssembleType::coincidence)
		{
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else
		{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是圆柱面
	else if (toolsurfacetype == GeomAbs_Cylinder)
	{
		gp_Cylinder toolplane = bastool.Cylinder();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影点到blank面的中轴线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, geomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合、对齐、同向一样
		//if (type == AssembleType::coincidence || type == AssembleType::orient)
		//{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);
			
			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合面是圆锥面
	else if (toolsurfacetype == GeomAbs_Cone)
	{
		gp_Cone toolplane = bastool.Cone();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影点到blank面的中轴线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, geomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//对齐和同向一样
		if (type == AssembleType::align || type == AssembleType::orient)
		{
			//可以不存在相合
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);
			
			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是球面
	else if (toolsurfacetype == GeomAbs_Sphere)
	{
		gp_Sphere toolplane = bastool.Sphere();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影点到blank面的中轴线
		GeomAPI_ProjectPointOnCurve gppoc(toolbasepoint, geomline);
		if (gppoc.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppoc.Point(1);

		//相合和对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);
			
			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
}

bool OCCTest::AssemblyPlaneFaceVertex(BRepAdaptor_Surface bas, TopoDS_Shape toolshape, NCollection_Mat4<double> matrix,
	TopoDS_Shape& targetshape, TopoDS_Shape toolshapeowner, Handle(Geom_Surface) blankgeomsurface, AssembleType type)
{
	gp_Pln plane = bas.Plane();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	gp_Pnt toolbasepoint = BRep_Tool::Pnt(TopoDS::Vertex(toolshape));
	//投影点到blank面
	GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeomsurface);
	if (!gppos.IsDone() && gppos.NbPoints() < 1)
		return false;
	gp_Pnt toolprojpoint = gppos.Point(1);

	//相合或者对齐
	if (type == AssembleType::coincidence || type == AssembleType::align)
	{
		gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
		//空间变换
		gp_Trsf transf;
		transf.SetTransformation(blankposition, toolterminal);

		transf.GetMat4(matrix);

		BRepBuilderAPI_Transform bat(transf);
		bat.Perform(toolshapeowner);
		if (!bat.IsDone())
			return false;
		targetshape = bat.Shape();
	}
}

bool OCCTest::AssemblyPlaneFaceFace(TopoDS_Shape toolshape, BRepAdaptor_Surface bas,
	 Handle(Geom_Surface) blankgeomsurface, TopoDS_Shape& targetshape, AssembleType type,
	NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner)
{
	gp_Pln plane = bas.Plane();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	TopoDS_Face toolface;
	TopoDS_Iterator iteratool(toolshape);
	for (; iteratool.More(); iteratool.Next())
	{
		if (iteratool.Value().ShapeType() == TopAbs_FACE)
		{
			toolface = TopoDS::Face(iteratool.Value());
			break;
		}
	}
	if (toolface.IsNull())
		return false;
	BRepAdaptor_Surface bastool(toolface);
	GeomAbs_SurfaceType toolsurfacetype = bastool.GetType();
	//耦合面是平面
	if (toolsurfacetype == GeomAbs_Plane)
	{
		gp_Pln toolplane = bastool.Plane();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();
		//投影点到blank面
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合、对齐、同向都一样
		//if (type == AssembleType::coincidence)
		//{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		//}
	}
	//耦合面是圆柱面
	else if (toolsurfacetype == GeomAbs_Cylinder)
	{
		gp_Cylinder toolplane = bastool.Cylinder();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//投影点到blank面
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合
		if (type == AssembleType::coincidence)
		{
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);
			
			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else
		{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是圆锥面
	else if (toolsurfacetype == GeomAbs_Cone)
	{
		gp_Cone toolplane = bastool.Cone();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//投影点到blank面
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合
		if (type == AssembleType::coincidence)
		{
			//可以不存在相合
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else if (type == AssembleType::orient)
		{
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合面是球面
	else if (toolsurfacetype == GeomAbs_Sphere)
	{
		gp_Sphere toolplane = bastool.Sphere();
		gp_Ax3 toolposition = toolplane.Position();

		gp_Pnt toolbasepoint = toolposition.Location();
		gp_Dir toolbasedir = toolposition.Direction();

		//投影点到blank面
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合和对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolterminal(toolprojpoint, toolbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolposition, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
}

bool OCCTest::AssemblyPlaneFaceEdge(TopoDS_Shape toolshape, BRepAdaptor_Surface bas, Handle(Geom_Surface) blankgeomsurface,
	AssembleType type, NCollection_Mat4<double> matrix, TopoDS_Shape toolshapeowner, TopoDS_Shape& targetshape)
{
	gp_Pln plane = bas.Plane();
	gp_Ax3 blankposition = plane.Position();

	gp_Pnt blankbasepoint = blankposition.Location();
	gp_Dir blankbasedir = blankposition.Direction();

	if (toolshape.ShapeType() == TopAbs_WIRE)
	{
		TopoDS_Iterator itera(toolshape);
		for (; itera.More(); itera.Next())
		{
			if (itera.Value().ShapeType() == TopAbs_EDGE)
			{
				toolshape = itera.Value();
				break;
			}
		}
	}
	if (toolshape.ShapeType() != TopAbs_EDGE)
		return false;
	TopoDS_Edge tooledge = TopoDS::Edge(toolshape);
	BRepAdaptor_Curve bastool(tooledge);
	double first, last;
	Handle(Geom_Curve) bascurve = BRep_Tool::Curve(tooledge, first, last);
	//耦合对象是直线
	if (bastool.GetType() == GeomAbs_Line)
	{
		gp_Lin line = bastool.Line();
		gp_Pnt toolbasepoint = line.Location();
		gp_Dir toolbasedir = line.Direction();

		gp_Dir toolnewdir(toolbasedir.X() - blankbasedir.X(),
			toolbasedir.Y() - blankbasedir.Y(), toolbasedir.Z() - blankbasedir.Z());

		//投影点到blank面
		GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeomsurface);
		if (!gppos.IsDone() && gppos.NbPoints() < 1)
			return false;
		gp_Pnt toolprojpoint = gppos.Point(1);

		//相合和对齐一样
		if (type == AssembleType::coincidence || type == AssembleType::align)
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, toolnewdir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);
			
			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
		else
		{
			gp_Ax3 toolorigin(toolbasepoint, toolbasedir);
			gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
			//空间变换
			gp_Trsf transf;
			transf.SetTransformation(toolorigin, toolterminal);

			transf.GetMat4(matrix);

			BRepBuilderAPI_Transform bat(transf);
			bat.Perform(toolshapeowner);
			if (!bat.IsDone())
				return false;
			targetshape = bat.Shape();
		}
	}
	//耦合对象是平面线
	else if (bastool.GetType() != GeomAbs_Line)
	{
		gp_Pnt toolbasepoint;
		gp_Dir toolbasedir;
		if (IsEdgePlanar(bascurve, first, last, toolbasepoint, toolbasedir))
		{
			//投影点到blank面
			GeomAPI_ProjectPointOnSurf gppos(toolbasepoint, blankgeomsurface);
			if (!gppos.IsDone() && gppos.NbPoints() < 1)
				return false;
			gp_Pnt toolprojpoint = gppos.Point(1);

			//相合、对齐、同向的情况都一样
			//if (type == AssembleType::coincidence)
			//{
				gp_Ax3 toolterminal(toolprojpoint, blankbasedir);
				//空间变换
				gp_Trsf transf;
				transf.SetTransformation(toolterminal);

				transf.GetMat4(matrix);

				BRepBuilderAPI_Transform bat(transf);
				bat.Perform(toolshapeowner);
				if (!bat.IsDone())
					return false;
				targetshape = bat.Shape();
			//}
		}
	}
}

bool OCCTest::IsEdgePlanar(Handle(Geom_Curve) curve, double first, double last, gp_Pnt& centerpoint, gp_Dir& normaldir)
{
	if (!curve)
		return false;
	gp_Pnt pt1 = curve->Value(first);
	gp_Pnt pt2 = curve->Value(first + (last - first) / 5);
	gp_Pnt pt3 = curve->Value(first + 2 * (last - first) / 5);
	gp_Pnt pt4 = curve->Value(first + 3 * (last - first) / 5);
	gp_Pnt pt5 = curve->Value(first + 4 * (last - first) / 5);
	gp_Pnt pt6 = curve->Value(first + 5 * (last - first) / 5);

	//造两个面
	GC_MakePlane gcmp1(pt1, pt2, pt3);
	GC_MakePlane gcmp2(pt4, pt5, pt6);

	Handle(Geom_Plane) pln1 = gcmp1.Value();
	Handle(Geom_Plane) pln2 = gcmp2.Value();
	if (!pln1 || !pln2)
		return false;

	gp_Pln p1 = pln1->Pln();
	gp_Pln p2 = pln2->Pln();

	bool overlap = false;
	if (p1.Distance(p2) < Precision::Confusion())
		overlap = true;
	if (!overlap)
		return false;
	gp_Ax1 ax1 = p1.Axis();
	centerpoint.SetX((pt1.X() + pt2.X() + pt3.X() + pt4.X() + pt5.X() + pt6.X()) / 6);
	centerpoint.SetY((pt1.Y() + pt2.Y() + pt3.Y() + pt4.Y() + pt5.Y() + pt6.Y()) / 6);
	centerpoint.SetY((pt1.Z() + pt2.Z() + pt3.Z() + pt4.Z() + pt5.Z() + pt6.Z()) / 6);
	normaldir = ax1.Direction();
	return true;
}

bool OCCTest::Transform(Transforminfo transinfo, NCollection_Mat4<double> inputmatrix , NCollection_Mat4<double>& outputmatrix)
{
	NCollection_Mat4<double> operatematrix;
	gp_Trsf trsfprocess;
	/*gp_Trsf trsfori, trsfprocess, trsffinal;
	double ori_a11 = inputmatrix.GetValue(1, 1);
	double ori_a12 = inputmatrix.GetValue(1, 2);
	double ori_a13 = inputmatrix.GetValue(1, 3);
	double ori_a14 = inputmatrix.GetValue(1, 4);
	double ori_a21 = inputmatrix.GetValue(2, 1);
	double ori_a22 = inputmatrix.GetValue(2, 2);
	double ori_a23 = inputmatrix.GetValue(2, 3);
	double ori_a24 = inputmatrix.GetValue(2, 4);
	double ori_a31 = inputmatrix.GetValue(3, 1);
	double ori_a32 = inputmatrix.GetValue(3, 2);
	double ori_a33 = inputmatrix.GetValue(3, 3);
	double ori_a34 = inputmatrix.GetValue(3, 4);
	trsfori.SetValues(ori_a11, ori_a12, ori_a13, ori_a14, ori_a21, ori_a22, ori_a23, ori_a24, ori_a31, ori_a32, ori_a33, ori_a34);*/
	//平移
	if (transinfo.istranslate)
	{
		//==========================================================
		// T1 = 1  0  0  0
		//      0  1  0  0
		//      0  0  1  0
		//      tx ty tz 1
		//==========================================================
		gp_Vec vec;
		vec.SetX(transinfo.tanslatedistance * transinfo.transformirection[0]);
		vec.SetY(transinfo.tanslatedistance * transinfo.transformirection[1]);
		vec.SetZ(transinfo.tanslatedistance * transinfo.transformirection[2]);
		trsfprocess.SetTranslation(vec);
		trsfprocess.GetMat4(operatematrix);
	}
	//旋转
	else if (transinfo.isrotate)
	{
		gp_Ax1 ax1(gp_Pnt(transinfo.transformbase[0], transinfo.transformbase[1], transinfo.transformbase[2]),
			gp_Dir(transinfo.transformirection[0], transinfo.transformirection[1], transinfo.transformirection[2]));
		trsfprocess.SetRotation(ax1, transinfo.rotateangleinrad);
		trsfprocess.GetMat4(operatematrix);
	}
	//缩放
	else if (transinfo.isscale)
	{
		//==========================================================
		// T1 = Sx 0  0  0
		//      0  Sy 0  0
		//      0  0  Sz 0
		//      0  0  0  1
		//==========================================================
		trsfprocess.SetScale(gp_Pnt(transinfo.transformbase[0], transinfo.transformbase[1], transinfo.transformbase[2]), transinfo.scalevalue);
		trsfprocess.GetMat4(operatematrix);
	}
	//镜像
	else if(transinfo.ismirror)
	{
		gp_Ax2 ax2(gp_Pnt(transinfo.transformbase[0], transinfo.transformbase[1], transinfo.transformbase[2]),
			gp_Dir(transinfo.transformirection[0], transinfo.transformirection[1], transinfo.transformirection[2]));
		trsfprocess.SetMirror(ax2);
		trsfprocess.GetMat4(operatematrix);
	}
	outputmatrix = inputmatrix * operatematrix;;
	return true;
}

bool OCCTest::Transform(Transforminfo transinfo, TopoDS_Shape& operateshape)
{
	gp_Trsf trsfprocess;
	//平移
	if (transinfo.istranslate)
	{
		gp_Vec vec;
		vec.SetX(transinfo.tanslatedistance * transinfo.transformirection[0]);
		vec.SetY(transinfo.tanslatedistance * transinfo.transformirection[1]);
		vec.SetZ(transinfo.tanslatedistance * transinfo.transformirection[2]);
		trsfprocess.SetTranslation(vec);
	}
	//旋转
	else if (transinfo.isrotate)
	{
		gp_Ax1 ax1(gp_Pnt(transinfo.transformbase[0], transinfo.transformbase[1], transinfo.transformbase[2]),
			gp_Dir(transinfo.transformirection[0], transinfo.transformirection[1], transinfo.transformirection[2]));
		trsfprocess.SetRotation(ax1, transinfo.rotateangleinrad);
	}
	//缩放
	else if (transinfo.isscale)
	{
		trsfprocess.SetScale(gp_Pnt(transinfo.transformbase[0], transinfo.transformbase[1], transinfo.transformbase[2]), transinfo.scalevalue);
	}
	//镜像
	else if (transinfo.ismirror)
	{
		gp_Ax2 ax2(gp_Pnt(transinfo.transformbase[0], transinfo.transformbase[1], transinfo.transformbase[2]),
			gp_Dir(transinfo.transformirection[0], transinfo.transformirection[1], transinfo.transformirection[2]));
		trsfprocess.SetMirror(ax2);
	}
	BRepBuilderAPI_Transform transform(trsfprocess);
	transform.Perform(operateshape);
	operateshape = transform.Shape();

	return true;
}

bool OCCTest::FaceSplitBody(TopoDS_Shape inputbody, std::vector<TopoDS_Face> faces, TopoDS_Compound& outputshape)
{
	if (inputbody.ShapeType() > TopAbs_SHELL)
		return false;
	//组装输出compound
	BRep_Builder B;
	B.MakeCompound(outputshape);
	if (inputbody.ShapeType() == TopAbs_COMPOUND)
	{
		TopoDS_Iterator iter(inputbody);
		for (; iter.More(); iter.Next())
		{
			if (iter.Value().ShapeType() == TopAbs_SOLID)
			{
				TopoDS_Shape splitedsolid;
				FaceSplitSolidShell(iter.Value(), faces, splitedsolid);
				B.Add(outputshape, splitedsolid);
			}
			else if (iter.Value().ShapeType() == TopAbs_SHELL)
			{
				TopoDS_Shape splitedshell;
				FaceSplitSolidShell(iter.Value(), faces, splitedshell);
				B.Add(outputshape, splitedshell);
			}
			else
				B.Add(outputshape, iter.Value());
		}
	}
	else
	{
		FaceSplitSolidShell(inputbody, faces, outputshape);
	}
	return true;
}

bool OCCTest::FaceSplitSolidShell(TopoDS_Shape inputbody, std::vector<TopoDS_Face> faces, TopoDS_Shape& outputshape)
{
	if (inputbody.ShapeType() != TopAbs_SOLID || inputbody.ShapeType() != TopAbs_SHELL)
		return false;

	BOPAlgo_CellsBuilder aCBuilder;

	aCBuilder.AddArgument(inputbody);
	for (int i = 0; i < faces.size(); i++)
	{
		aCBuilder.AddArgument(faces[i]);
	}

	aCBuilder.Perform();
	if (aCBuilder.HasErrors())
	{ // check error status
		return false;
	}

	TopTools_ListOfShape aLSToTake;
	TopTools_ListOfShape aLSToAvoid;

	aLSToTake.Append(inputbody);
	for (int i = 0; i < faces.size(); i++)
	{
		aLSToAvoid.Append(faces[i]);
	}
	aCBuilder.AddToResult(aLSToTake, aLSToAvoid);
	outputshape = aCBuilder.Shape();

	return true;
}

bool OCCTest::CalculateSpecificPoints(TopoDS_Shape inputbody, std::vector<gp_Pnt> points)
{
	bool generalpoints = true;

	std::vector<gp_Pnt> prepoints;
	std::vector<TopoDS_Edge> edges;
	TopExp_Explorer edgeex;
	for (edgeex.Init(inputbody, TopAbs_EDGE); edgeex.More(); edgeex.Next())
	{
		edges.push_back(TopoDS::Edge(edgeex.Current()));
	}
	for (int i = 0; i < edges.size(); i++)
	{
		TopoDS_Edge curedge = edges[i];
		BRepAdaptor_Curve curvex(curedge);
		double ufst = curvex.FirstParameter();
		double ulst = curvex.LastParameter();
		double umid = (ufst + ulst) / 2;
		prepoints.push_back(curvex.Value(ufst));
		prepoints.push_back(curvex.Value(ulst));
		prepoints.push_back(curvex.Value(umid));
		if (!generalpoints)
		{
			double u14 = (ufst + ulst) / 4;
			double u34 = 3 * (ufst + ulst) / 4;
			prepoints.push_back(curvex.Value(u14));
			prepoints.push_back(curvex.Value(u34));
			if (curvex.GetType() == GeomAbs_Circle)
			{
				gp_Circ circ = curvex.Circle();
				prepoints.push_back(circ.Location());
			}
			else if (curvex.GetType() == GeomAbs_Ellipse)
			{
				gp_Elips elips = curvex.Ellipse();
				prepoints.push_back(elips.Location());
				prepoints.push_back(elips.Focus1());
				prepoints.push_back(elips.Focus2());
			}
			else if (curvex.GetType() == GeomAbs_Hyperbola)
			{
				gp_Hypr Hypr = curvex.Hyperbola();
				prepoints.push_back(Hypr.Location());
				prepoints.push_back(Hypr.Focus1());
				prepoints.push_back(Hypr.Focus2());
			}
			else if (curvex.GetType() == GeomAbs_Parabola)
			{
				gp_Parab Parab = curvex.Parabola();
				prepoints.push_back(Parab.Location());
				prepoints.push_back(Parab.Focus());
			}
		}
	}

	for (int i = 0; i < prepoints.size(); i++)
	{
		bool ifadd = true;
		gp_Pnt pt1 = prepoints[i];
		for (int j = 0; j < points.size(); j++)
		{
			gp_Pnt pt2 = points[j];
			if (pt1.X() == pt2.X() && pt1.Y() == pt2.Y() && pt1.Z() == pt2.Z())
			{
				ifadd = false;
				break;
			}
		}
		if (ifadd)
			points.push_back(pt1);
	}
	return true;
}

bool OCCTest::CompondSolidTest(TopoDS_Shape shape1, TopoDS_Shape shape2, TopoDS_Shape& res)
{
	TopoDS_CompSolid coms;
	BRep_Builder b;
	b.MakeCompSolid(coms);
	b.Add(shape1, coms);
	b.Add(shape2, coms);
	res = BRepBuilderAPI_MakeSolid(coms).Solid();
	return true;
}

bool OCCTest::MakeSolidTest(TopoDS_Solid shape1, TopoDS_Shell shape2, TopoDS_Shape& res)
{
	BRepBuilderAPI_MakeSolid mksold(shape1);
	mksold.Add(shape2);
	res = mksold.Solid();
	return true;
}

bool OCCTest::MakeDraft(TopoDS_Shape& shape)
{
	TopoDS_Wire Profile;
	std::vector<TopoDS_Edge> anEdges;
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(5, 0, 0)));
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(5, 5, 0)));
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 5, 0), gp_Pnt(0, 5, 0)));
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 5, 0), gp_Pnt(0, 0, 0)));
	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);

	//建立棱柱
	gp_Vec vec(0, 0, 5);
	BRepPrimAPI_MakePrism* make_prism = new BRepPrimAPI_MakePrism(ProfileFace, vec);
	TopoDS_Shape shape1 = make_prism->Shape();

	TopoDS_Wire Profile1;
	std::vector<TopoDS_Edge> anEdges1;
	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(10, 0, 0)));
	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 5, 0)));
	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 5, 0), gp_Pnt(5, 5, 0)));
	anEdges1.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 5, 0), gp_Pnt(5, 0, 0)));
	OCCBasicTools::GetOrderWireFromEdges(anEdges1, Profile1);
	TopoDS_Face ProfileFace1 = BRepBuilderAPI_MakeFace(Profile1);

	//建立棱柱
	BRepPrimAPI_MakePrism* make_prism1 = new BRepPrimAPI_MakePrism(ProfileFace1, vec);
	TopoDS_Shape shape2 = make_prism1->Shape();

	CompondSolidTest(shape1, shape2, shape);

	TopoDS_Wire Profile2;
	std::vector<TopoDS_Edge> anEdges2;
	anEdges2.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
	anEdges2.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 5, 0)));
	anEdges2.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 5, 0), gp_Pnt(0, 5, 0)));
	anEdges2.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 5, 0), gp_Pnt(0, 0, 0)));
	OCCBasicTools::GetOrderWireFromEdges(anEdges2, Profile2);
	TopoDS_Face ProfileFace2 = BRepBuilderAPI_MakeFace(Profile2);
	//建立棱柱
	BRepPrimAPI_MakePrism* make_prism2 = new BRepPrimAPI_MakePrism(ProfileFace2, vec);
	TopoDS_Shape shape3 = make_prism2->Shape();
	TopExp_Explorer exsold;
	TopoDS_Solid solid1;
	for (exsold.Init(shape3, TopAbs_SOLID); exsold.More(); exsold.Next())
	{
		solid1 = TopoDS::Solid(exsold.Current());
	}

	TopoDS_Wire Profile3;
	std::vector<TopoDS_Edge> anEdges3;
	anEdges3.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(5, 5, 0)));
	anEdges3.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 5, 0), gp_Pnt(5, 5, 5)));
	anEdges3.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 5, 5), gp_Pnt(5, 0, 5)));
	anEdges3.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 5), gp_Pnt(5, 0, 0)));
	OCCBasicTools::GetOrderWireFromEdges(anEdges3, Profile3);
	TopoDS_Face ProfileFace3 = BRepBuilderAPI_MakeFace(Profile3);
	BRep_Builder b;
	TopoDS_Shell shell1;
	b.MakeShell(shell1);
	b.Add(shell1, ProfileFace3);
	MakeSolidTest(solid1, shell1, shape);

	return true;
}

//bool OCCTest::MakeDraft(TopoDS_Shape& shape)
//{
//	//testting code
//	//gp_Dir dir1(0, 0, 1);
//	//gp_Dir dir2(0, 1, 0);
//	//gp_Dir dir3(1, 0, 0);
//	//gp_Dir dir4(-1, 0, 0);
//	//gp_Dir dir5(0, -1, 0);
//
//	//gp_Dir cross1 = dir1.Crossed(dir2);
//	//gp_Dir cross2 = dir2.Crossed(dir1);
//	//gp_Dir cross3 = dir1.Crossed(dir3);
//	//gp_Dir cross4 = dir3.Crossed(dir1);
//
//	//gp_Dir cross5 = dir1.Crossed(dir4);
//	//gp_Dir cross6 = dir4.Crossed(dir1);
//	//gp_Dir cross7 = dir1.Crossed(dir5);
//	//gp_Dir cross8 = dir5.Crossed(dir1);
//
//	////绘制椭圆
//	//gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
//	//gp_Elips test_elips(ax2,10,5);
//	//TopoDS_Wire my_wire;
//	//TopoDS_Face myface;
//	////模型，边数，线框
//	//ConvertEllipse2Polygon(test_elips, 7, my_wire);
//
//	//TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(my_wire);
//	//shape = ProfileFace;
//
//
//	TopoDS_Wire Profile;
//	std::vector<TopoDS_Edge> anEdges;
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(5, 0, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(5, 0, 0), gp_Pnt(10, 10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 20, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 20, 0), gp_Pnt(-10, -10, 0)));
//	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(-10, -10, 0), gp_Pnt(0, 0, 0)));
//	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
//	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
//
//	//建立棱柱
//	gp_Vec vec(10, 10, 20);
//	BRepPrimAPI_MakePrism* make_prism = new BRepPrimAPI_MakePrism(ProfileFace, vec);
//
//	TopoDS_Shape first_shape = make_prism->FirstShape();
//	TopoDS_Shape first_face = TopoDS::Face(first_shape);
//	TopoDS_Shape last_shape = make_prism->LastShape();
//
//	BRepSweep_Prism prism = make_prism->Prism();
//	//shape = prism.Shape();
//
//	gp_Dir Direc(0., 0., 1.);                             //中性面的方向，似乎不影响
//	gp_Dir Direc1(-10., -10., -20.);                      //（关键）
//	Standard_Real Angle = -5. * PI / 180.;                  //代表拔模方向上与（0，0，1）法向的夹角                   （关键）
//	gp_Pln Neutral(gp_Pnt(0, 0, 20), Direc);                 //选择的这个面代表棱柱的这个底面在拔模过程中面积不会发生变化   （关键）
//	BRepOffsetAPI_DraftAngle theDraft_test(shape);
//	BRepOffsetAPI_DraftAngle theDraft(shape);
//
//	//找棱柱的侧面（用于拔模的面）
//	TopTools_ListOfShape ListOfFace_preselect;             //初选所有面
//	TopTools_ListOfShape ListOfFace_finalselect;           //真正可以用的面
//	TopExp_Explorer Ex;
//	for (Ex.Init(shape, TopAbs_FACE); Ex.More(); Ex.Next())
//	{
//		ListOfFace_preselect.Append(Ex.Value());
//	}
//
//	TopTools_ListIteratorOfListOfShape itl;
//	//这一段for筛出能拔模的面
//	for (itl.Initialize(ListOfFace_preselect); itl.More(); itl.Next())
//	{
//		TopoDS_Face todo_face = TopoDS::Face(itl.Value());                              //theDraft.Remove(todo_face)这个方法有bug，别用
//		theDraft_test.Add(todo_face, Direc1, Angle, Neutral);
//		if (theDraft_test.AddDone())
//		{
//			ListOfFace_finalselect.Append(todo_face);
//		}
//		else
//		{
//			break;
//		}
//	}
//	if (ListOfFace_finalselect.Size() != ListOfFace_preselect.Size())
//	{
//		for (itl.Initialize(ListOfFace_finalselect); itl.More(); itl.Next())
//		{
//			TopoDS_Face todo_face = TopoDS::Face(itl.Value());
//			theDraft.Add(todo_face, Direc1, Angle, Neutral);
//		}
//	}
//
//	theDraft.Build();
//	shape = theDraft.Shape();
//
//	//倒角
//	bool if_filleted = false;
//	//if_filleted = fillet(shape);
//
//	//OffSet(shape, -1, if_filleted);
//
//	return true;
//}

bool OCCTest::FindTypeLevelFatherLabel(TDF_Label& inputlabel, TopAbs_ShapeEnum type, TopoDS_Shape& currentshape)
{
	TopoDS_Shape testshape = getLabelShape(inputlabel);
	TopAbs_ShapeEnum currenttype = testshape.ShapeType();
	if (currenttype > type)
	{
		inputlabel = inputlabel.Father();
		FindTypeLevelFatherLabel(inputlabel, type, currentshape);
		//手动终止递归
		if (!currentshape.IsNull())
			return true;
		currenttype = testshape.ShapeType();
	}
	else if (currenttype == type)
	{
		currentshape = getLabelShape(inputlabel);
		return true;
	}	
	return false;
}

//从label找shape
TopoDS_Shape OCCTest::getLabelShape(TDF_Label label)
{
	TopoDS_Shape emptyshape;
	Handle(TNaming_NamedShape) namedshape;
	if(!label.FindAttribute(TNaming_NamedShape::GetID(), namedshape))
		return emptyshape;
	return namedshape->Get();
}

//从shape找label
TDF_Label OCCTest::getShapeLabel(TDF_Label& rootlabel, TopoDS_Shape shape)
{
	Handle(TNaming_NamedShape) namedshape;
	//当前label与要找的shape的label一致
	if (!rootlabel.FindAttribute(TNaming_NamedShape::GetID(), namedshape))
	{
		TopoDS_Shape compareshape = namedshape->Get();
		if (compareshape.IsSame(shape))
		{
			return rootlabel;
		}
	}

	//没找到，继续往rootlabel的子节点寻找
	TDF_ChildIterator childiter(rootlabel);
	for (; childiter.More(); childiter.Next())
	{
		TDF_Label childlabel = childiter.Value();
		getShapeLabel(childlabel, shape);
	}
}

bool OCCTest::TestRemoveHolesWithinAreaAndDetectFaces(TopoDS_Shape muttershape, 
	TopoDS_Shape& output, TopTools_ListOfShape& facelist, double inputarea)
{
	ShapeUpgrade_RemoveInternalWires sri(muttershape);
	sri.RemoveFaceMode() = true;
	sri.MinArea() = inputarea;
	sri.Perform();
	ShapeExtend_Status ses;
	if (sri.Status(ses) && ses <= ShapeExtend_DONE)
	{
		output = sri.GetResult();
		TopTools_SequenceOfShape faces = sri.RemovedFaces();
		for (auto iter : faces)
		{
			facelist.Append(iter);
		}
		return true;
	}
	return false;
}

bool OCCTest::DetectHoleFacesAndRemove(TopoDS_Shape shape, TopTools_ListOfShape& facelist,
	double inputradius)
{
	TopExp_Explorer faceex;
	//当前模型所有的面
	std::vector<TopoDS_Face> facelists;
	//发现的小于半径值的圆线框
	std::vector<TopoDS_Wire> wirelists;
	for (faceex.Init(shape, TopAbs_FACE); faceex.More(); faceex.Next())
		facelists.push_back(TopoDS::Face(faceex.Current()));
	//找到内部线框
	for (int i = 0; i < facelists.size(); i++)
	{
		TopTools_ListOfShape innerwires;
		ShapeAnalysis_FreeBounds shapeanalyze(facelists[i]);
		TopoDS_Compound wires = shapeanalyze.GetClosedWires();

		TopoDS_Shape outcontour;
		double biggestarea = 0;
		//找到最大wire,即外部线框
		for (TopExp_Explorer iter(wires, TopAbs_WIRE); iter.More(); iter.Next())
		{
			Bnd_Box box;
			BRepBndLib::Add(iter.Current(), box);
			if (biggestarea < box.SquareExtent())
			{
				biggestarea = box.SquareExtent();
				outcontour = iter.Current();
			}
		}

		//再次遍历wires，如果包围盒小于外框，则是内部线
		for (TopExp_Explorer iter(wires, TopAbs_WIRE); iter.More(); iter.Next())
		{
			Bnd_Box box;
			BRepBndLib::Add(iter.Current(), box);
			if (biggestarea > box.SquareExtent())
			{
				innerwires.Append(iter.Current());
			}
		}

		//寻找内部线框面积小于指定面积的
		for (auto wireiter : innerwires)
		{
			GProp_GProps ggx;
			BRepGProp::LinearProperties(wireiter, ggx);
			double areax = ggx.Mass();
			if (areax <= inputradius * 2 * M_PI)
			{
				wirelists.push_back(TopoDS::Wire(wireiter));
			}
		}
	}

	//处理内部线框，找内部线框对应的面
	std::vector<TopoDS_Edge> edgelist;
	TopTools_ListOfShape face_listofshape;
	//找一下内部线的最大半径
	std::vector<double> radiuses;
	for (int i = 0; i < wirelists.size(); i++)
	{
		TopoDS_Wire wireshape = wirelists[i];
		TopExp_Explorer thisex;
		TopoDS_Edge edgesp;
		bool needcontinue = false;
		for (thisex.Init(wireshape, TopAbs_EDGE); thisex.More(); thisex.Next())
		{
			edgesp = TopoDS::Edge(thisex.Current());
			BRepAdaptor_Curve xcurve(edgesp);
			Handle(Geom_Curve) xxcurve = xcurve.Curve().Curve();
			Handle(Geom_Circle) xcire = Handle(Geom_Circle)::DownCast(xxcurve);
			if (xcire)
			{
				gp_Circ xcircle = xcire->Circ();
				double radius = xcircle.Radius();
				radiuses.push_back(radius);
			}
			else
				needcontinue = true;
		}
		//跳过中间干涉边
		if(needcontinue)
			continue;

		for (int j = 0; j < facelists.size(); j++)
		{
			TopoDS_Face xface = facelists[j];
			Handle(Geom_Surface) xsurface = BRep_Tool::Surface(xface);
			if (!xsurface)
				continue;
			if(xsurface->IsKind(STANDARD_TYPE(Geom_Plane)))
				continue;
			TopExp_Explorer wireex;

			bool ifcontinue = true;
			for (wireex.Init(xface, TopAbs_EDGE); wireex.More(); wireex.Next())
			{
				if(IsShapeGeomSame(edgesp, wireex.Current(),TopAbs_EDGE))
					ifcontinue = false;
			}
			if(ifcontinue)
				continue;
			if (!facelist.Contains(xface))
				facelist.Append(xface);

			//找face的对面边
			for (wireex.Init(xface, TopAbs_EDGE); wireex.More(); wireex.Next())
			{
				TopoDS_Shape compareshape = wireex.Current();
				bool ifint = Does2EdgeIntersect(edgesp, TopoDS::Edge(wireex.Current()));
				bool ifrepeat = false;
				for (int k = 0; k < wirelists.size(); k++)
				{
					TopoDS_Wire wireshapexx = wirelists[k];
					//内部线转为edge
					TopExp_Explorer thisexx;
					TopoDS_Edge edgespx;
					for (thisexx.Init(wireshapexx, TopAbs_EDGE); thisexx.More(); thisexx.Next())
					{
						edgespx = TopoDS::Edge(thisexx.Current());
					}
					if (IsShapeGeomSame(edgespx, compareshape, TopAbs_EDGE))
					{
						ifrepeat = true;
						break;
					}
				}
				if (!ifint && !ifrepeat)
					edgelist.push_back(TopoDS::Edge(wireex.Current()));
			}
		}
	}
	std::sort(radiuses.begin(), radiuses.end());
	//找另一个面
	for (int i = 0; i < facelists.size(); i++)
	{
		TopExp_Explorer wireex;
		TopoDS_Face xface = facelists[i];
		bool ifcontinue = true;
		for (wireex.Init(xface, TopAbs_EDGE); wireex.More(); wireex.Next())
		{
			TopoDS_Edge xire = TopoDS::Edge(wireex.Current());
			for (int j = 0; j < edgelist.size(); j++)
			{
				TopoDS_Edge xxire = edgelist[j];
				if (IsShapeGeomSame(xire, xxire, TopAbs_EDGE))
					ifcontinue = false;
			}
		}
		if (ifcontinue)
			continue;
		if (!facelist.Contains(xface))
		{
			//需要判断一下内外性，防止最外轮廓面被包括
			GProp_GProps gg;
			BRepGProp::SurfaceProperties(xface, gg);
			double comparea = M_PI * pow(radiuses[radiuses.size() - 1], 2);
			double surfarea = gg.Mass();
			if (surfarea <= comparea + 0.1)
				facelist.Append(xface);
		}
	}

	TopTools_ListOfShape finalfacelist;
	//处理特殊情况，内曲面分离
	for (auto iter : facelist)
	{
		TopoDS_Shape xs = iter;
		Handle(Geom_Surface) xsurface = BRep_Tool::Surface(TopoDS::Face(xs));
		if (xsurface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface)))
		{
			Handle(Geom_CylindricalSurface) xsurfcy = Handle(Geom_CylindricalSurface)
				::DownCast(xsurface);
			gp_Cylinder xcylinder = xsurfcy->Cylinder();
			gp_Ax1 ax1 = xcylinder.Axis();
			double r1 = xcylinder.Radius();
			for (int i = 0; i < facelists.size(); i++)
			{
				TopoDS_Face xface = facelists[i];
				if (facelist.Contains(xface))
					continue;
				Handle(Geom_Surface) xxsurface = BRep_Tool::Surface(xface);
				if (xxsurface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface)))
				{
					Handle(Geom_CylindricalSurface) xxsurfcy = Handle(Geom_CylindricalSurface)
						::DownCast(xxsurface);
					gp_Cylinder xxcylinder = xxsurfcy->Cylinder();
					gp_Ax1 xax1 = xxcylinder.Axis();
					double xr1 = xxcylinder.Radius();
					if (ax1.IsCoaxial(xax1, 0.00001, 0.00001) && r1 == xr1)
						finalfacelist.Append(xface);
				}
			}
		}
	}
	for (auto iter : finalfacelist)
	{
		if (!facelist.Contains(iter))
			facelist.Append(iter);
	}
	return true;
}

bool OCCTest::Does2EdgeIntersect(TopoDS_Edge eg1, TopoDS_Edge eg2)
{
	BRepAdaptor_Curve c1(eg1);
	BRepAdaptor_Curve c2(eg2);

	Extrema_ExtCC ecc(c1, c2);
	ecc.Perform();
	int num = ecc.NbExt();
	if (num == 1)
		return false;
	return true;
}

void OCCTest::InterceptString(string stringtocut)
{
	std::vector<string> array;
	int index = 0;
	while (stringtocut.find("@") != -1)
	{
		index = stringtocut.find("@");
		string xstr = stringtocut.substr(0, index);
		array.push_back(xstr);
		stringtocut = stringtocut.substr(index + 1, stringtocut.length() - index);
	}
	array.push_back(stringtocut);
	int MB_OK = 0x00000000L;
	MessageBoxA(NULL, "abc", "abc", MB_OK);
}


bool OCCTest::StitchFaces(std::vector<TopoDS_Shape> inputfaces, double gap,
	std::vector<TopoDS_Shape>& outresults)
{
	if (inputfaces.size() < 1)
		return false;
	std::vector<TopoDS_Shape> thisentities, nextentities;
	TopoDS_Shape argshape = inputfaces[0];
	for (int i = 1; i < inputfaces.size(); i++)
	{
		TopoDS_Shape toolshape = inputfaces[i];
		double distance = BRepExtrema_DistShapeShape(argshape, toolshape).Value();
		if (distance <= gap + 0.1)
			thisentities.push_back(toolshape);
		else
			nextentities.push_back(toolshape);
	}
	BRepBuilderAPI_Sewing sewbuilder;
	sewbuilder.SetTolerance(gap);
	sewbuilder.Load(argshape);
	for (int i = 0; i < thisentities.size(); i++)
	{
		sewbuilder.Add(thisentities[i]);
	}
	sewbuilder.Perform();
	if (sewbuilder.IsModified(argshape))
		argshape = sewbuilder.SewedShape();
	outresults.push_back(argshape);
	StitchFaces(nextentities, gap, outresults);
	return true;
}

bool OCCTest::XDStitchFaces(string facestobestitched)
{
	try
	{
		/*TDocStd_Document* curDoc = TheOCCApp.getCurrentModel()->getDoc();
		Handle(TDF_Label) data = curDoc->GetData();
		TDF_Label rootlabel = data->Root();*/

		TDF_Label rootlabel = TDocStd_Document::Main();

	}
	catch (...)
	{
		return true;
	}
}
