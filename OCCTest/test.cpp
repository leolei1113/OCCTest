#include "test.h"

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <AIS_Shape.hxx>
#include "StlAPI_Writer.hxx"
#include "BRepAlgoAPI_Section.hxx"

#include "BRepMesh_IncrementalMesh.hxx"
#include "BRepBuilderAPI_Copy.hxx"
#include "BRep_Tool.hxx"
#include "BRepTools_ReShape.hxx"
#include "occView.h"

#include "Standard_CString.hxx"
#include "RWStl.hxx"
//#include "StlMesh_Mesh.hxx"
//#include "StlMesh_MeshExplorer.hxx"
#include "StlAPI_Writer.hxx"
#include "QPolarChart"
#include "QValueAxis"
#include "QScatterSeries"
#include "OSD_Environment.hxx"
#include "qlayout.h"


#include <STEPControl_Controller.hxx>
#include <STEPControl_Writer.hxx>
#include <AIS_InteractiveContext.hxx>

#include "OCCTest.h"

void CreateLinearDim(gp_Pnt first_pnt, gp_Pnt second_pnt, gp_Pnt txt_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ);
void Annotation_Lead_(gp_Pnt first_pnt, gp_Pnt txt_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ);
void Annotation_Angle_(gp_Pnt center_pnt, gp_Pnt first_pnt, gp_Pnt second_pnt, gp_Pnt txt_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ);
void CreateDiameterDim(gp_Pnt first_pnt, gp_Pnt second_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ);
void CreateRadiusDim(gp_Pnt first_pnt, gp_Pnt second_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ);
void CreateDim(TopoDS_Face showface, gp_Pnt position, TopoDS_Edge showedge, OccView* occ);

#define color Quantity_NOC_RED

TopoDS_Shape Test()
{
	TopoDS_Shape shape;

    OCCTest::MakeDraft(shape);

	return shape;
}

test::test(QWidget *parent)
    : QMainWindow(parent)
{
    //建立UI界面
    //ui.setupUi(this);

    //这个路径是干吗用的，运行完没有这个文件
    QString exeDir = QApplication::applicationDirPath();
    exeDir += "/Shaders";

    //Add occ environment setting
    TCollection_AsciiString envSDValue = TCollection_AsciiString((Standard_CString)(exeDir.toStdString().c_str()));
    OSD_Environment aSDR(TCollection_AsciiString("CSF_ShadersDirectory"), envSDValue);
    aSDR.Build();

    OccView* occ = new OccView(this);

    QVBoxLayout *layout = new QVBoxLayout(ui.centralWidget);
    layout->setMargin(0);
    layout->addWidget(occ);
//     ui.occ->setBackgroundRole(QPalette::Dark);

    /************************************************************************/
    /* occ                                                                     */
    /************************************************************************/


	TopoDS_Shape shape = Test();
    Handle(AIS_Shape) anAisBox = new AIS_Shape(shape);

    anAisBox->SetColor(Quantity_Color(/*tYZData.m_OutletColor[0], tYZData.m_OutletColor[1], tYZData.m_OutletColor[2]*/12 / 255.0, 254 / 255.0, 28 / 255.0, Quantity_TOC_RGB));
    anAisBox->SetDisplayMode(AIS_Shaded);

    //anAisBox->SetColor(Quantity_NOC_AZURE);

    occ->getContext()->Display(anAisBox, Standard_True);
    occ->getContext()->SetColor(anAisBox, color,true);

    //========================文字标注
	/*TopoDS_Wire Profile;
	std::vector<TopoDS_Edge> anEdges;
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)));
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)));
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0)));
	anEdges.push_back(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 10, 0), gp_Pnt(0, 0, 0)));
	OCCBasicTools::GetOrderWireFromEdges(anEdges, Profile);
	TopoDS_Face ProfileFace = BRepBuilderAPI_MakeFace(Profile);
	gp_Pnt position(12, 5, 0);
	TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(gp_Pnt(10, 0, 0), gp_Pnt(10, 10, 0)).Edge();
	CreateDim(ProfileFace, position, edge, occ);*/

    occ->fitAll();
}

test::~test()
{

}

void CreateDim(TopoDS_Face showface, gp_Pnt position, TopoDS_Edge showedge, OccView* occ)
{
    Handle(AIS_LengthDimension) ais_dim = new AIS_LengthDimension(showface, showedge);
    ais_dim->SetTextPosition(position);
    Handle(AIS_Shape) anAisBox1 = new AIS_Shape(showface);

	occ->getContext()->Display(ais_dim, Standard_True);
	occ->getContext()->SetColor(ais_dim, color, true);

	occ->getContext()->Display(anAisBox1, Standard_True);
	occ->getContext()->SetColor(anAisBox1, color, true);

	occ->fitAll();
}

//void CreateLinearDim(gp_Pnt first_pnt, gp_Pnt second_pnt, gp_Pnt txt_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ)
//{
//    //文字高度
//    double height_txt = 7;
//
//    double first_pt_x = first_pnt.X();
//    double first_pt_y = first_pnt.Y();
//    double first_pt_z = first_pnt.Z();
//
//    double second_pt_x = second_pnt.X();
//    double second_pt_y = second_pnt.Y();
//    double second_pt_z = second_pnt.Z();
//
//    double txt_pt_x = txt_pnt.X();
//    double txt_pt_y = txt_pnt.Y();
//    double txt_pt_z = txt_pnt.Z();
//
//    gp_Pnt imp_fst_pnt(first_pt_x, txt_pt_y - 1, first_pt_z);
//    gp_Pnt imp_sec_pnt(second_pt_x, txt_pt_y - 1, second_pt_z);
//
//    //三条标注线
//    TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(first_pnt, imp_fst_pnt).Edge();
//    TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(second_pnt, imp_sec_pnt).Edge();
//
//    TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(gp_Pnt(first_pt_x, txt_pt_y, first_pt_z), gp_Pnt(second_pt_x, txt_pt_y, second_pt_z)).Edge();
//
//    //建造箭头arrow
//    double angle = std::atan2(second_pt_y - first_pt_y, second_pt_x - first_pt_x);
//
//    double length = 1.0;
//
//    double add1 = sin(angle + M_PI / 3) * length;
//    double add2 = sin(angle + M_PI - M_PI / 3) * length;
//    double add3 = cos(angle + M_PI / 3) * length;
//    double add4 = cos(angle + M_PI - M_PI / 3) * length;
//
//    gp_Pnt arrowpt1(first_pt_x + sin(angle + M_PI / 3) * length, txt_pt_y + cos(angle + M_PI / 3) * length, 0);
//    gp_Pnt arrowpt2(first_pt_x + sin(angle + M_PI - M_PI / 3) * length, txt_pt_y + cos(angle + M_PI - M_PI / 3) * length, 0);
//    TopoDS_Edge edge4_1 = BRepBuilderAPI_MakeEdge(gp_Pnt(first_pt_x, txt_pt_y, first_pt_z), arrowpt1).Edge();
//    TopoDS_Edge edge4_2 = BRepBuilderAPI_MakeEdge(gp_Pnt(first_pt_x, txt_pt_y, first_pt_z), arrowpt2).Edge();
//    TopoDS_Edge edge4_3 = BRepBuilderAPI_MakeEdge(arrowpt1, arrowpt2).Edge();
//    TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge4_1, edge4_2, edge4_3).Wire();
//    TopoDS_Face face1 = BRepBuilderAPI_MakeFace(wire1).Face();
//
//    gp_Pnt arrowpt3(second_pt_x - sin(angle + M_PI / 3) * length, txt_pt_y + cos(angle + M_PI / 3) * length, 0);
//    gp_Pnt arrowpt4(second_pt_x - sin(angle + M_PI - M_PI / 3) * length, txt_pt_y + cos(angle + M_PI - M_PI / 3) * length, 0);
//    TopoDS_Edge edge5_1 = BRepBuilderAPI_MakeEdge(gp_Pnt(second_pt_x, txt_pt_y, second_pt_z), arrowpt3).Edge();
//    TopoDS_Edge edge5_2 = BRepBuilderAPI_MakeEdge(gp_Pnt(second_pt_x, txt_pt_y, second_pt_z), arrowpt4).Edge();
//    TopoDS_Edge edge5_3 = BRepBuilderAPI_MakeEdge(arrowpt3, arrowpt4).Edge();
//    TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge5_1, edge5_2, edge5_3).Wire();
//    TopoDS_Face face2 = BRepBuilderAPI_MakeFace(wire2).Face();
//
//    Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
//    BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
//    new_bop->AddArgument(edge1);
//    new_bop->AddTool(edge2);
//    new_bop->AddTool(edge3);
//    //new_bop->AddTool(face1);
//    //new_bop->AddTool(face2);
//    BOPAlgo_BOP* new_bop1 = new BOPAlgo_BOP(aAllocator);
//    new_bop1->AddArgument(face1);
//    new_bop1->AddTool(face2);
//
//    new_bop->SetOperation(BOPAlgo_FUSE);
//    new_bop->Perform();
//    TopoDS_Shape shape = new_bop->Shape();
//    Handle(AIS_Shape) anAisBox = new AIS_Shape(shape);
//
//    new_bop1->SetOperation(BOPAlgo_FUSE);
//    new_bop1->Perform();
//    TopoDS_Shape shape1 = new_bop1->Shape();
//    Handle(AIS_Shape) anAisBox1 = new AIS_Shape(shape1);
//
//    occ->getContext()->Display(anAisBox, Standard_True);
//    occ->getContext()->SetColor(anAisBox, color, true);
//
//    occ->getContext()->Display(anAisBox1, Standard_True);
//    occ->getContext()->SetColor(anAisBox1, color, true);
//
//    Standard_CString str = value;
//    TCollection_ExtendedString tostr(str);
//    Handle(AIS_TextLabel) aLabel = new AIS_TextLabel();
//    aLabel->SetText(tostr);
//    aLabel->SetColor(color);
//    aLabel->SetPosition(txt_pnt);
//    aLabel->SetFont(txt_type);
//    occ->getContext()->Display(aLabel, Standard_False);
//
//    occ->fitAll();
//}

void CreateLinearDim(gp_Pnt first_pnt, gp_Pnt second_pnt, gp_Pnt txt_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ)
{
	//文字高度
	double height_txt = 7;

	double first_pt_x = first_pnt.X();
	double first_pt_y = first_pnt.Y();
	double first_pt_z = first_pnt.Z();

	double second_pt_x = second_pnt.X();
	double second_pt_y = second_pnt.Y();
	double second_pt_z = second_pnt.Z();

	double txt_pt_x = txt_pnt.X();
	double txt_pt_y = txt_pnt.Y();
	double txt_pt_z = txt_pnt.Z();

	gp_Pnt imp_fst_pnt(first_pt_x, txt_pt_y - 1, first_pt_z);
	gp_Pnt imp_sec_pnt(second_pt_x, txt_pt_y - 1, second_pt_z);

	//三条标注线
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(first_pnt, imp_fst_pnt).Edge();
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(second_pnt, imp_sec_pnt).Edge();

	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(gp_Pnt(first_pt_x, txt_pt_y, first_pt_z), gp_Pnt(second_pt_x, txt_pt_y, second_pt_z)).Edge();

	//建造箭头arrow
	double angle = std::atan2(second_pt_y - first_pt_y, second_pt_x - first_pt_x);

	double length = 1.0;

	double add1 = sin(angle + M_PI / 3) * length;
	double add2 = sin(angle + M_PI - M_PI / 3) * length;
	double add3 = cos(angle + M_PI / 3) * length;
	double add4 = cos(angle + M_PI - M_PI / 3) * length;

	gp_Pnt arrowpt1(first_pt_x + sin(angle + M_PI / 3) * length, txt_pt_y + cos(angle + M_PI / 3) * length, 0);
	gp_Pnt arrowpt2(first_pt_x + sin(angle + M_PI - M_PI / 3) * length, txt_pt_y + cos(angle + M_PI - M_PI / 3) * length, 0);
	TopoDS_Edge edge4_1 = BRepBuilderAPI_MakeEdge(gp_Pnt(first_pt_x, txt_pt_y, first_pt_z), arrowpt1).Edge();
	TopoDS_Edge edge4_2 = BRepBuilderAPI_MakeEdge(gp_Pnt(first_pt_x, txt_pt_y, first_pt_z), arrowpt2).Edge();
	TopoDS_Edge edge4_3 = BRepBuilderAPI_MakeEdge(arrowpt1, arrowpt2).Edge();
	TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge4_1, edge4_2, edge4_3).Wire();
	TopoDS_Face face1 = BRepBuilderAPI_MakeFace(wire1).Face();

	gp_Pnt arrowpt3(second_pt_x - sin(angle + M_PI / 3) * length, txt_pt_y + cos(angle + M_PI / 3) * length, 0);
	gp_Pnt arrowpt4(second_pt_x - sin(angle + M_PI - M_PI / 3) * length, txt_pt_y + cos(angle + M_PI - M_PI / 3) * length, 0);
	TopoDS_Edge edge5_1 = BRepBuilderAPI_MakeEdge(gp_Pnt(second_pt_x, txt_pt_y, second_pt_z), arrowpt3).Edge();
	TopoDS_Edge edge5_2 = BRepBuilderAPI_MakeEdge(gp_Pnt(second_pt_x, txt_pt_y, second_pt_z), arrowpt4).Edge();
	TopoDS_Edge edge5_3 = BRepBuilderAPI_MakeEdge(arrowpt3, arrowpt4).Edge();
	TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge5_1, edge5_2, edge5_3).Wire();
	TopoDS_Face face2 = BRepBuilderAPI_MakeFace(wire2).Face();

	Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
	BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
	new_bop->AddArgument(edge1);
	new_bop->AddTool(edge2);
	new_bop->AddTool(edge3);
	//new_bop->AddTool(face1);
	//new_bop->AddTool(face2);
	BOPAlgo_BOP* new_bop1 = new BOPAlgo_BOP(aAllocator);
	new_bop1->AddArgument(face1);
	new_bop1->AddTool(face2);

	new_bop->SetOperation(BOPAlgo_FUSE);
	new_bop->Perform();
	TopoDS_Shape shape = new_bop->Shape();
	Handle(AIS_Shape) anAisBox = new AIS_Shape(shape);

	new_bop1->SetOperation(BOPAlgo_FUSE);
	new_bop1->Perform();
	TopoDS_Shape shape1 = new_bop1->Shape();
	Handle(AIS_Shape) anAisBox1 = new AIS_Shape(shape1);

	occ->getContext()->Display(anAisBox, Standard_True);
	occ->getContext()->SetColor(anAisBox, color, true);

	occ->getContext()->Display(anAisBox1, Standard_True);
	occ->getContext()->SetColor(anAisBox1, color, true);

	Standard_CString str = value;
	TCollection_ExtendedString tostr(str);
	Handle(AIS_TextLabel) aLabel = new AIS_TextLabel();
	aLabel->SetText(tostr);
	aLabel->SetColor(color);
	aLabel->SetPosition(txt_pnt);
	aLabel->SetFont(txt_type);
	occ->getContext()->Display(aLabel, Standard_False);

	occ->fitAll();
}

void Annotation_Lead_(gp_Pnt first_pnt, gp_Pnt txt_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ)
{
    //文字高度
    double height_txt = 7;

    double first_pt_x = first_pnt.X();
    double first_pt_y = first_pnt.Y();
    double first_pt_z = first_pnt.Z();

    double txt_pt_x = txt_pnt.X();
    double txt_pt_y = txt_pnt.Y();
    double txt_pt_z = txt_pnt.Z();

    gp_Pnt imp_fst_pnt(txt_pt_x + 1, first_pt_y, first_pt_z);

    //标注线
    TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(first_pnt, imp_fst_pnt).Edge();
    TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(gp_Pnt(txt_pt_x, txt_pt_y - 1, txt_pt_z), gp_Pnt(txt_pt_x, first_pt_y, first_pt_z)).Edge();

    //箭头
    double angle = std::atan2(txt_pt_y - first_pt_y, 0);
    double length = 1.0;
    gp_Pnt arrowpt1(txt_pt_x - sin(angle + M_PI / 3) * length, first_pt_y - cos(angle + M_PI / 3) * length, 0);
    gp_Pnt arrowpt2(txt_pt_x - sin(angle + M_PI - M_PI / 3) * length, first_pt_y - cos(angle + M_PI - M_PI / 3) * length, 0);
    TopoDS_Edge edge2_1 = BRepBuilderAPI_MakeEdge(gp_Pnt(txt_pt_x, first_pt_y, first_pt_z), arrowpt1).Edge();
    TopoDS_Edge edge2_2 = BRepBuilderAPI_MakeEdge(gp_Pnt(txt_pt_x, first_pt_y, first_pt_z), arrowpt2).Edge();
    TopoDS_Edge edge2_3 = BRepBuilderAPI_MakeEdge(arrowpt1, arrowpt2).Edge();
    TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge2_1, edge2_2, edge2_3).Wire();
    TopoDS_Face face1 = BRepBuilderAPI_MakeFace(wire1).Face();
    Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
    BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
    new_bop->AddArgument(edge1);
    //new_bop->AddTool(face1);
    new_bop->AddTool(edge3);

    new_bop->SetOperation(BOPAlgo_FUSE);
    new_bop->Perform();
    TopoDS_Shape shape = new_bop->Shape();

    Handle(AIS_Shape) anAisBox = new AIS_Shape(shape);
    Handle(AIS_Shape) anAisBox1 = new AIS_Shape(face1);

    occ->getContext()->Display(anAisBox, Standard_True);
    occ->getContext()->SetColor(anAisBox, color, true);

    occ->getContext()->Display(anAisBox1, Standard_True);
    occ->getContext()->SetColor(anAisBox1, color, true);

    Standard_CString str = value;
    TCollection_ExtendedString tostr(str);
    Handle(AIS_TextLabel) aLabel = new AIS_TextLabel();
    aLabel->SetText(tostr);
    aLabel->SetColor(color);
    double angle1 = 90.0 / 180.0 * M_PI;
    aLabel->SetAngle(angle1);
    aLabel->SetPosition(txt_pnt);
    aLabel->SetFont(txt_type);
    occ->getContext()->Display(aLabel, Standard_False);

    occ->fitAll();
}

void Annotation_Angle_(gp_Pnt center_pnt, gp_Pnt first_pnt, gp_Pnt second_pnt, gp_Pnt txt_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ)
{
    double first_pt_x = first_pnt.X();
    double first_pt_y = first_pnt.Y();
    double first_pt_z = first_pnt.Z();

    double second_pt_x = second_pnt.X();
    double second_pt_y = second_pnt.Y();
    double second_pt_z = second_pnt.Z();

    double center_pt_x = center_pnt.X();
    double center_pt_y = center_pnt.Y();
    double center_pt_z = center_pnt.Z();

    double txt_pt_x = txt_pnt.X();
    double txt_pt_y = txt_pnt.Y();
    double txt_pt_z = txt_pnt.Z();

    gp_Vec vec1(center_pt_x - first_pt_x, center_pt_y - first_pt_y, center_pt_z - first_pt_z);
    gp_Vec vec2(center_pt_x - second_pt_x, center_pt_y - second_pt_y, center_pt_z - second_pt_z);

    double assisi_pt_y = (first_pt_x - second_pt_x + vec1.Y() * first_pt_y / vec1.X() - vec2.Y() * second_pt_y / vec2.X()) / (vec1.Y() / vec1.X() - vec2.Y() / vec2.X());
    double assisi_pt_x = vec1.Y() * (-assisi_pt_y + first_pt_y) / vec1.X() + first_pt_x;
    gp_Pnt assisi_pt(assisi_pt_x, assisi_pt_y, 0);

    //创建圆弧
    double radium = center_pnt.Distance(first_pnt);
    gp_Ax2 ax(center_pnt, gp_Dir(0, 0, 1));
    gp_Circ circ(ax, radium);
    Handle(Geom_TrimmedCurve) arc = GC_MakeArcOfCircle(circ, first_pnt, second_pnt,true).Value();
    TopoDS_Edge aEdge1 = BRepBuilderAPI_MakeEdge(arc).Edge();

    TopoDS_Edge aEdge2 = BRepBuilderAPI_MakeEdge(center_pnt, first_pnt).Edge();
    TopoDS_Edge aEdge3 = BRepBuilderAPI_MakeEdge(center_pnt, second_pnt).Edge();

    //建造箭头arrow
    double angle = std::atan2(first_pt_y - assisi_pt_y, first_pt_x - assisi_pt_x);
    double length = 1.0;
    gp_Pnt arrowpt1(first_pt_x + sin(angle + M_PI / 3) * length, first_pt_y + cos(angle + M_PI / 3) * length, 0);
    gp_Pnt arrowpt2(first_pt_x + sin(angle + M_PI - M_PI / 3) * length, first_pt_y + cos(angle + M_PI - M_PI / 3) * length, 0);
    TopoDS_Edge edge4_1 = BRepBuilderAPI_MakeEdge(first_pnt, arrowpt1).Edge();
    TopoDS_Edge edge4_2 = BRepBuilderAPI_MakeEdge(first_pnt, arrowpt2).Edge();
    TopoDS_Edge edge4_3 = BRepBuilderAPI_MakeEdge(arrowpt1, arrowpt2).Edge();
    TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge4_1, edge4_2, edge4_3).Wire();
    TopoDS_Face face1 = BRepBuilderAPI_MakeFace(wire1).Face();

    angle = std::atan2(second_pt_y - assisi_pt_y, second_pt_x - assisi_pt_x);
    gp_Pnt arrowpt3(second_pt_x - sin(angle + M_PI / 3) * length, second_pt_y + cos(angle + M_PI / 3) * length, 0);
    gp_Pnt arrowpt4(second_pt_x - sin(angle + M_PI - M_PI / 3) * length, second_pt_y + cos(angle + M_PI - M_PI / 3) * length, 0);
    TopoDS_Edge edge5_1 = BRepBuilderAPI_MakeEdge(second_pnt, arrowpt3).Edge();
    TopoDS_Edge edge5_2 = BRepBuilderAPI_MakeEdge(second_pnt, arrowpt4).Edge();
    TopoDS_Edge edge5_3 = BRepBuilderAPI_MakeEdge(arrowpt3, arrowpt4).Edge();
    TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge5_1, edge5_2, edge5_3).Wire();
    TopoDS_Face face2 = BRepBuilderAPI_MakeFace(wire2).Face();

    Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();
    BOPAlgo_BOP* new_bop = new BOPAlgo_BOP(aAllocator);
    new_bop->AddArgument(aEdge1);
    new_bop->AddTool(aEdge2);
    new_bop->AddTool(aEdge3);
    /*new_bop->AddTool(face1);
    new_bop->AddTool(face2);*/
    BOPAlgo_BOP* new_bop1 = new BOPAlgo_BOP(aAllocator);
    new_bop1->AddArgument(face1);
    new_bop1->AddTool(face2);

    new_bop->SetOperation(BOPAlgo_FUSE);
    new_bop->Perform();
    TopoDS_Shape shape = new_bop->Shape();

    new_bop1->SetOperation(BOPAlgo_FUSE);
    new_bop1->Perform();
    TopoDS_Shape shape1 = new_bop1->Shape();
    Handle(AIS_Shape) anAisBox1 = new AIS_Shape(shape1);

    Handle(AIS_Shape) anAisBox = new AIS_Shape(shape);

    occ->getContext()->Display(anAisBox, Standard_True);
    occ->getContext()->SetColor(anAisBox, color, true);

    occ->getContext()->Display(anAisBox1, Standard_True);
    occ->getContext()->SetColor(anAisBox1, color, true);


    Standard_CString str = value;
    TCollection_ExtendedString tostr(str);
    Handle(AIS_TextLabel) aLabel = new AIS_TextLabel();
    aLabel->SetText(tostr);
    aLabel->SetColor(color);
    aLabel->SetAngle(angle);
    aLabel->SetPosition(txt_pnt);
    aLabel->SetFont(txt_type);
    occ->getContext()->Display(aLabel, Standard_False);

    occ->fitAll();
}

void CreateDiameterDim(gp_Pnt first_pnt, gp_Pnt second_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ)
{
    //文字高度
    double height_txt = 7;

    double first_pt_x = first_pnt.X();
    double first_pt_y = first_pnt.Y();
    double first_pt_z = first_pnt.Z();

    double second_pt_x = second_pnt.X();
    double second_pt_y = second_pnt.Y();
    double second_pt_z = second_pnt.Z();

    //标注线
    TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(first_pnt, second_pnt).Edge();

    //建造箭头arrow
    double angle = std::atan2(second_pt_y - first_pt_y, second_pt_x - first_pt_x);
    double length = 1.0;

    gp_Pnt arrowpt1(first_pt_x + sin(angle + M_PI / 3) * length, first_pt_y - cos(angle + M_PI / 3) * length, 0);
    gp_Pnt arrowpt2(first_pt_x + sin(angle + M_PI - M_PI / 3) * length, first_pt_y - cos(angle + M_PI - M_PI / 3) * length, 0);
    TopoDS_Edge edge4_1 = BRepBuilderAPI_MakeEdge(first_pnt, arrowpt1).Edge();
    TopoDS_Edge edge4_2 = BRepBuilderAPI_MakeEdge(first_pnt, arrowpt2).Edge();
    TopoDS_Edge edge4_3 = BRepBuilderAPI_MakeEdge(arrowpt1, arrowpt2).Edge();
    TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge4_1, edge4_2, edge4_3).Wire();
    TopoDS_Face face1 = BRepBuilderAPI_MakeFace(wire1).Face();

    gp_Pnt arrowpt3(second_pt_x - sin(angle + M_PI / 3) * length, second_pt_y + cos(angle + M_PI / 3) * length, 0);
    gp_Pnt arrowpt4(second_pt_x - sin(angle + M_PI - M_PI / 3) * length, second_pt_y + cos(angle + M_PI - M_PI / 3) * length, 0);
    TopoDS_Edge edge5_1 = BRepBuilderAPI_MakeEdge(second_pnt, arrowpt3).Edge();
    TopoDS_Edge edge5_2 = BRepBuilderAPI_MakeEdge(second_pnt, arrowpt4).Edge();
    TopoDS_Edge edge5_3 = BRepBuilderAPI_MakeEdge(arrowpt3, arrowpt4).Edge();
    TopoDS_Wire wire2 = BRepBuilderAPI_MakeWire(edge5_1, edge5_2, edge5_3).Wire();
    TopoDS_Face face2 = BRepBuilderAPI_MakeFace(wire2).Face();

    Handle(NCollection_BaseAllocator) aAllocator = NCollection_BaseAllocator::CommonBaseAllocator();

    BOPAlgo_BOP* new_bop1 = new BOPAlgo_BOP(aAllocator);
    new_bop1->AddArgument(face1);
    new_bop1->AddTool(face2);

    TopoDS_Shape shape = edge1;
    Handle(AIS_Shape) anAisBox = new AIS_Shape(shape);

    new_bop1->SetOperation(BOPAlgo_FUSE);
    new_bop1->Perform();
    TopoDS_Shape shape1 = new_bop1->Shape();
    Handle(AIS_Shape) anAisBox1 = new AIS_Shape(shape1);

    occ->getContext()->Display(anAisBox, Standard_True);
    occ->getContext()->SetColor(anAisBox, color, true);     

    occ->getContext()->Display(anAisBox1, Standard_True);
    occ->getContext()->SetColor(anAisBox1, color, true);

    gp_Pnt txt_pnt((first_pt_x + second_pt_x) / 2, (first_pt_y + second_pt_y) / 2, (first_pt_z + second_pt_z) / 2);

    Standard_CString str = 'Φ' + value;

    TCollection_ExtendedString tostr(str);
    Handle(AIS_TextLabel) aLabel = new AIS_TextLabel();
    aLabel->SetText(tostr);
    aLabel->SetColor(color);
    aLabel->SetPosition(txt_pnt);
    aLabel->SetFont(txt_type);
    aLabel->SetAngle(angle);
    occ->getContext()->Display(aLabel, Standard_False);

    occ->fitAll();
}

void CreateRadiusDim(gp_Pnt first_pnt, gp_Pnt second_pnt, Standard_CString value, Standard_CString txt_type, OccView* occ)
{
	//文字高度
	double height_txt = 7;

	double first_pt_x = first_pnt.X();
	double first_pt_y = first_pnt.Y();
	double first_pt_z = first_pnt.Z();

	double second_pt_x = second_pnt.X();
	double second_pt_y = second_pnt.Y();
	double second_pt_z = second_pnt.Z();
	//标注线
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(first_pnt, second_pnt).Edge();
	//建造箭头arrow
	double angle = std::atan2(second_pt_y - first_pt_y, second_pt_x - first_pt_x);
	double length = 1.0;
	gp_Pnt arrowpt1(first_pt_x + sin(angle + M_PI / 3) * length, first_pt_y - cos(angle + M_PI / 3) * length, 0);
	gp_Pnt arrowpt2(first_pt_x + sin(angle + M_PI - M_PI / 3) * length, first_pt_y - cos(angle + M_PI - M_PI / 3) * length, 0);
	TopoDS_Edge edge4_1 = BRepBuilderAPI_MakeEdge(first_pnt, arrowpt1).Edge();
	TopoDS_Edge edge4_2 = BRepBuilderAPI_MakeEdge(first_pnt, arrowpt2).Edge();
	TopoDS_Edge edge4_3 = BRepBuilderAPI_MakeEdge(arrowpt1, arrowpt2).Edge();
	TopoDS_Wire wire1 = BRepBuilderAPI_MakeWire(edge4_1, edge4_2, edge4_3).Wire();
	TopoDS_Face face1 = BRepBuilderAPI_MakeFace(wire1).Face();

    Handle(AIS_Shape) anAisBox = new AIS_Shape(edge1);
	Handle(AIS_Shape) anAisBox1 = new AIS_Shape(face1);

	occ->getContext()->Display(anAisBox, Standard_True);
	occ->getContext()->SetColor(anAisBox, color, true);

	occ->getContext()->Display(anAisBox1, Standard_True);
	occ->getContext()->SetColor(anAisBox1, color, true);

	gp_Pnt txt_pnt((first_pt_x + second_pt_x) / 2, (first_pt_y + second_pt_y) / 2, (first_pt_z + second_pt_z) / 2);

	Standard_CString str = 'R' + value;

	TCollection_ExtendedString tostr(str);
	Handle(AIS_TextLabel) aLabel = new AIS_TextLabel();
	aLabel->SetText(tostr);
	aLabel->SetColor(color);
	aLabel->SetPosition(txt_pnt);
	aLabel->SetFont(txt_type);
	//aLabel->SetAngle(angle);
	occ->getContext()->Display(aLabel, Standard_False);

	occ->fitAll();
}