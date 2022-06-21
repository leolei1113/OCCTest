#include "TopoDS_Shape.hxx"
#include "occApp.h"
#include<vector>
using std::vector;

class AAHX {
	struct Data
	{
	public:
		Standard_Real Rc = 62.5;		//中心圆半径
		Standard_Real Theta = 90.0;		//进出口中心包角
		Standard_Real Rt = 5.32;		//集流管内径
		Standard_Real DeltaT = 0.3;		//集流管壁厚
		Standard_Real ht = 2.73;		//集流管缺口高度
		Standard_Integer NR = 3;		//径向排数
		Standard_Integer NL = 49;		//轴向排数
		Standard_Real dr = 3.00;		//径向间距
		Standard_Real dl = 3.00;		//轴向间距
		Standard_Real Ri = 1;			//管子内径
		Standard_Real deltai = 0.3;		//管子壁厚
		Standard_Real L = 150.3;		//换热器轴向长度
		Standard_Real R1 = 75;			//外涵道内径
		Standard_Real R2 = 50;			//外涵道外径
		Standard_Real thetaOut;			//外涵道包角
		Standard_Real Lout;				//外涵道长度

		Standard_Real Theta1;
		Standard_Real thetaOut1;
		Standard_Real sl;
		Standard_Real sr;

		Data() {
			calculateParameter();
		}

		Data(Standard_Real Rc, Standard_Real Theata, Standard_Real Rt,
			Standard_Real deltaT, Standard_Real ht, Standard_Real dr,
			Standard_Real dl, Standard_Real Ri, Standard_Real deltai,
			Standard_Real L, Standard_Real R1, Standard_Real R2,
			Standard_Real thetaOut, Standard_Real Lout) {

			setParameter(Rc, Theata, Rt, DeltaT, ht, dr, dl, Ri, deltai, L, R1, R2, thetaOut, Lout);

			calculateParameter();
		}

		inline void setParameter(Standard_Real Rc, Standard_Real Theata, Standard_Real Rt,
			Standard_Real deltaT, Standard_Real ht, Standard_Real dr,
			Standard_Real dl, Standard_Real Ri, Standard_Real deltai,
			Standard_Real L, Standard_Real R1, Standard_Real R2,
			Standard_Real thetaOut, Standard_Real Lout);

		inline void calculateParameter();
	} data;


public:
	//绘图数据
	gp_Pln plane;
	gp_Ax1 ax = gp::OY();

	Standard_Real angle1, angle2;					//角度

	gp_Pnt2d po1, po2, po3, pi1, pi2, pi3;			//左立柱横截面轮廓坐标
	gp_Pnt2d pom1, pom2, pom3, pim1, pim2, pim3;	//右立柱横截面轮廓坐标
	gp_Pnt2d pob1, pob2, pob3, pob4, pob5, pob6;	//外包围盒横截面坐标

	gp_Pnt p3d;										//换热管草图平面原点坐标
	gp_Vec v3d, v3d1;

	Standard_Real theta;							//立柱上的内平面与外平面夹角

	TopoDS_Shape aahxShape;							//整体shape
	TopoDS_Shape boxShape;							//外包围盒shape
	TopoDS_Shape shapeEntity;						//换热器shape
	TopoDS_Compound shapeInside, shapeOutside;		//内外域shape
	TopoDS_Builder inBuilder, outBuilder;			//内外域builder
	TopoDS_Shape shapeSeal1, shapeSeal2;


public:
	/*
		完成对坐标点参数计算
	*/
	AAHX();
	
	TopoDS_Shape createTest();

	/*
		生成外包围盒Shape
	*/
	TopoDS_Shape createBox();

	/*
		生成整体Shape
	*/
	TopoDS_Shape createAAHX();

	/*
		生成换热器shape
	*/
	TopoDS_Shape createShape();

	void saveIges(const std::string& igesFile, TopoDS_Shape shape);
	void saveStep(const std::string& stepFile, TopoDS_Shape shape);
	void saveStl(const std::string& stlFile, TopoDS_Shape shape);

	/*
		将整体shape分成内域shape和外域shape
	*/
	void splitShpe();


};

