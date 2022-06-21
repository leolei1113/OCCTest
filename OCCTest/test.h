#ifndef TEST_H
#define TEST_H

#include <QMainWindow>
//#include "ui_test.h"
#include "TopoDS_Shape.hxx"
#include "QTextStream"
#include "QChartView"
#include "QStringList"
#include "qvector.h"
#include <AIS_TextLabel.hxx>
#include <AIS_Trihedron.hxx>
#include <Geom_Axis2Placement.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <gp_Circ.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <AIS_Dimension.hxx>
#include <AIS_LengthDimension.hxx>
// class TopoDS_Shape;
// class Standard_Real;

class test : public QMainWindow
{
    Q_OBJECT

public:
    test(QWidget *parent = 0);
    
    ~test();

private:
   // Ui::testClass ui;

};

#endif // TEST_H
