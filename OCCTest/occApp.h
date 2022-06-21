#pragma once
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
#include <gp_Elips.hxx>

#include<Geom_Plane.hxx>
#include <gp_Pln.hxx>
#include <NCollection_Array1.hxx>
#include<NCollection/NCollection_Vector.hxx>
#include<NCollection/NCollection_List.hxx>

#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>

#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include "IGESControl_Reader.hxx"

#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>

#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <GC_MakeCircle.hxx>


/*
class Sketch {
	gp_Pnt position;							//草图位置
	gp_Mat mtl;									//坐标变换矩阵				
	
	//NCollection_List<Handle(Geom_Curve)> edgeTmp;

	std::vector<Handle(Geom_Curve)> edgeTmp;

	NCollection_Vector<TopoDS_Edge> wires;

	TopoDS_Face face;
private:
	inline TopoDS_Wire createWire();
public:
	Sketch();
	Sketch(gp_Ax2 coor);
	Sketch(gp_Pnt position, gp_Vec direction);
	Sketch(gp_Pnt position, gp_Vec dx, gp_Vec dy, gp_Vec dz);
	~Sketch();

	inline void setPosi(gp_Pnt posi);
	inline void setCoor(gp_Ax1 vx, gp_Ax1 vy, gp_Ax1 vz);
	inline void setCoor(gp_Ax2 coordinate);
	inline gp_Pnt getLocation();
	gp_Vec getDirection();

	TopoDS_Face* getFace();

	Handle(Geom_Curve) drawLine(gp_Pnt2d p1, gp_Pnt2d p2);
	Handle(Geom_Curve) drawCircle(gp_Pnt2d center, Standard_Real radius);
	Handle(Geom_Curve) drawArc(gp_Pnt2d pfirst, gp_Pnt2d pOn, gp_Pnt2d pEnd);
	
	void drawRectangle(gp_Pnt2d center, Standard_Real length, Standard_Real width = 0);

	
	//	将缓冲中的edge进行镜像
		//center:		镜像中心
	//	isDelete:	是否删除原来边
	
	void mirror(gp_Ax1 center, Standard_Boolean isDelete = false);

	
	//	旋转edge
	//	center:	旋转中心
	//	angle:	旋转角度
	
	//void rotate(gp_Pnt2d center, Standard_Real angle, Standard_Boolean isDelete);

	
	//	将edge连接成wire并返回
	
	//TopoDS_Wire getWire();

	
	// 清空草图
	
	//inline void clear();

	
	//保存缓存中edges
	
	//void flush();

	
	//获取全局坐标
	
	//inline gp_Pnt getGlobalCoor(gp_Pnt2d& p1);
	
};
*/

/*
class OccApp {
	TopoDS_Shape myBody;			//Shape文件

	TopoDS_Compound CompoundShape;
	BRep_Builder aBuilder;
	
	TopoDS_Shape tmp;				//临时Shape文件
	
	TopoDS_Shape* face;				

	Sketch sketch;					//草图
	TopoDS_Face xoy, yoz, zoy;		//坐标平面
	TopoDS_Edge ox, oy, oz;			//坐标轴

	NCollection_List<TopoDS_Shape> shapeTmp;

public:
	TopoDS_Shape getShape();
	OccApp();
	void setFace(TopoDS_Face& tf);

	TopoDS_Shape extrusion(Standard_Real len);

	Sketch* createSketch(gp_Pln pln = gp_Pln(gp_Pnt(0,0,0), gp_Vec(0,0,1)));
	TopoDS_Shape stretching(gp_Vec dir1, Standard_Real len1, gp_Vec dir2 = gp_Vec(1, 0, 0), Standard_Real len2 = 0);
	TopoDS_Shape stretchingTo(gp_Pnt p1);
	TopoDS_Face* makeFace();
	TopoDS_Wire makeWire();
	TopoDS_Shape stretchingCut(gp_Vec dir1, Standard_Real len1, gp_Vec dir2 = gp_Vec(1, 0, 0), Standard_Real len2 = 0);
	TopoDS_Shape revolve(gp_Ax1 axis, Standard_Real angle1, Standard_Real angle2 = 0);
	TopoDS_Shape makeRevolve(gp_Ax1 axis, Standard_Real angle1, Standard_Real angle2 = 0);
	TopoDS_Shape revolveCut(gp_Ax1 axis, Standard_Real angle1, Standard_Real angle2 = 0);
	TopoDS_Shape mirrow(gp_Ax2 centerAxis);
	TopoDS_Shape mirrow(TopoDS_Shape ts, gp_Ax2 centerAxis);
	
	TopoDS_Edge drawLine(gp_Pnt p1, gp_Pnt p2);
	TopoDS_Edge drawCircle(gp_Pnt pcenter, Standard_Real r, gp_Vec dir);

	void SaveIges(const std::string& igesFile);
	inline TopoDS_Shape stretch(gp_Vec dir1, Standard_Real len1);

private:
	
	inline TopoDS_Shape revol(gp_Ax1 axis, Standard_Real angle1);

	void setVissibleCoor(Standard_Integer i);
};
*/

class Sketch {
	gp_Pnt location;
	gp_Vec normal;
	gp_Mat mtl;
	NCollection_Vector<TopoDS_Edge> edgTmp;
	NCollection_List<Handle(Geom_Curve)> geomTmp;
	gp_Ax2 coordinate;
	
	TopoDS_Wire wire;
	TopoDS_Face myFace;

public:
	Sketch();
	~Sketch();
	Sketch(gp_Ax2 coor);

	Sketch(gp_Pnt posi, gp_Vec direction);
	Sketch(gp_Pnt posi, gp_Vec dx, gp_Vec dy, gp_Vec dz);

	inline gp_Pnt getLocation();
	inline gp_Vec getNormal();
	
	inline void setLocation(gp_Pnt& posi) {
		location = posi;
	}

	inline void setCoordinate(const gp_Ax2& coordinate);
	inline void setNormal(gp_Vec&);

	void setCoor(const gp_Ax1& ax, const gp_Ax1& ay, const gp_Ax1& az);
	
	void clear();

	TopoDS_Face* getFace();

	TopoDS_Wire* getWire();

	Handle(Geom_Curve) drawLine(const gp_Pnt2d& p1, const gp_Pnt2d& p2);

	Handle(Geom_Curve) drawCircle(const gp_Pnt2d& pCenter, const Standard_Real& radius);

	Handle(Geom_Curve) drawArc(const gp_Pnt2d& pfirst, const gp_Pnt2d& pOn, const gp_Pnt2d& pEnd);

	void drawRectangle(const gp_Pnt2d& center, const Standard_Real& length, const Standard_Real width = 0);

	void drawRegularPolygon(const gp_Pnt2d& center, gp_Pnt2d pStart, const Standard_Integer& edgNum);

	void drawRectangle(const gp_Pnt2d& p1, const gp_Pnt2d& p2);

	void mirror(const gp_Ax1& center, const Standard_Boolean& isDelete);

	void rotate(const gp_Pnt2d& center, const Standard_Real& angle, const Standard_Boolean& isDelete);
	
	void flush();

	inline gp_Pnt getGlobalCoor(const gp_Pnt2d& p1)
	{
		gp_Pnt p3d1;
		gp_XYZ tmp(p1.X(), p1.Y(), 0);
		p3d1.SetXYZ(mtl * tmp + location.XYZ());

		return p3d1;
	}

private:
	inline void setMtl(const gp_Ax2& coor);

	inline void createWire();

};

/*
	OccApp：建模帮助类，完成拉伸，旋转，拉伸切除，旋转切除，线性阵列，面阵列，体阵列，阵列切除等操作
*/
class OccApp {
	
	TopoDS_Shape* face;				//保存由草图生成的平面，或者wire
	Sketch sketch;					//草图
	const gp_Ax2 xoy = gp::XOY();	//xoy面
public:

	TopoDS_Shape myShape;			//shape
	TopoDS_Shape shapeTmp;			//临时shape

	OccApp();

	TopoDS_Shape makeBall(const gp_Pnt& center, const Standard_Real& radius = 1);

	TopoDS_Shape extrusion(gp_Vec dir1, const Standard_Real len1, gp_Vec dir2, const Standard_Real len2);	
	
	TopoDS_Shape extrusion(gp_Vec& dir, const Standard_Real& len);
	
	TopoDS_Shape extrusion(const Standard_Real& len1, const Standard_Real& len2);
	
	TopoDS_Shape extrusion(gp_Pnt& p1);

	TopoDS_Shape extrusion(const Standard_Real& len);

	TopoDS_Shape extrusionCut(gp_Vec& dir, const Standard_Real& len);

	TopoDS_Shape extrusionCut(gp_Vec& dir1, Standard_Real& len1, gp_Vec& dir2, Standard_Real& len2);

	TopoDS_Shape extrusionCut(const Standard_Real& len);

	TopoDS_Shape revolveExtrusion(const gp_Ax1& axis, const Standard_Real& angle1, const Standard_Real& angle2 = 0);

	TopoDS_Shape revolveCut(const gp_Ax1& axis, const Standard_Real& angle1, const Standard_Real& angle2 = 0);

	TopoDS_Shape linearPattern(gp_Vec& dir, const Standard_Real& gapDis, const Standard_Integer& num);

	TopoDS_Shape linearPattern(gp_Vec& dir, const Standard_Integer& num, const Standard_Real& distance);

	TopoDS_Shape facePattern(gp_Vec& dir1, const Standard_Real& gapDis1, const Standard_Integer& num1,
		gp_Vec& dir2, const Standard_Real& gapDis2, const Standard_Integer& num2);

	TopoDS_Shape volumePattern(gp_Vec& dir1, const Standard_Real& gapDis1, const Standard_Integer& num1,
		gp_Vec& dir2, const Standard_Real& gapDis2, const Standard_Integer& num2,
		gp_Vec& dir3, const Standard_Real& gapDis3, const Standard_Integer& num3);

	TopoDS_Shape linearPatternCut(gp_Vec& dir, const Standard_Real& gapDis, const Standard_Integer& num);

	TopoDS_Shape facePatternCut(gp_Vec& dir1, const Standard_Real& gapDis1, const Standard_Integer& num1,
		gp_Vec& dir2, const Standard_Real& gapDis2, const Standard_Integer& num2);

	TopoDS_Shape circularPattern(const gp_Ax1& centerAx, const Standard_Real& gapRadian, const Standard_Integer& num);

	TopoDS_Shape circularPatternCut(const gp_Ax1& centerAx, const Standard_Real& gapRadian, const Standard_Integer& num);

	TopoDS_Shape offset();

	TopoDS_Shape mirror(const gp_Ax1 &ax);

	TopoDS_Shape mirror(const gp_Ax2& ax);

	TopoDS_Shape mirrorMyself(const gp_Ax2& ax);

	TopoDS_Shape getShape();
	
	void flush1();

	void flush2();

	TopoDS_Face* makeFace();

	TopoDS_Wire* makeWire();

	Sketch* createSketch(const gp_Ax2& coor =  gp::XOY());

	TopoDS_Shape makeBoolCut(TopoDS_Shape& ts);

	TopoDS_Shape makeBoolUnion(TopoDS_Shape& ts);

	TopoDS_Shape makeBoolCoomon(TopoDS_Shape& ts);

	TopoDS_Shape makeCompound(TopoDS_Shape& ts);

	TopoDS_Shape boolCut(TopoDS_Shape ts);

	TopoDS_Shape boolUnion(TopoDS_Shape ts);
	
	TopoDS_Shape boolCommon(TopoDS_Shape ts);

	static TopoDS_Shape boolCut(TopoDS_Shape& ts1, TopoDS_Shape& ts2);

	static TopoDS_Shape boolUnion(TopoDS_Shape& ts1, TopoDS_Shape& ts2);

	static TopoDS_Shape boolCommon(TopoDS_Shape& ts1, TopoDS_Shape& ts2);

	static TopoDS_Shape Compound(TopoDS_Shape& ts1, TopoDS_Shape& ts2);

private:
	inline void boolCut();
	inline void boolUnion();
};
