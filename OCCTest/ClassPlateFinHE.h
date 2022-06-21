#pragma once
#include "TopoDS_Shape.hxx"

//板翅式换热器，冷端热端排布方式，从最底层开始排布
enum PFHELayerType
{
	CHCHCH = 0,			//cold + hot + cold + ...
	HCHCHC,
	CHHCHH,
	HCCHCC
};

struct FinData
{
	int mN = 0;				//	翅片层数
	double mH = 0;				//	翅片高
	double mT = 0;				//	翅片厚度
	double ms = 0;				//	翅片间距
	double mx = 0;				//	内距
	double my = 0;				//	内高
	double mJ = 0;				//	节距

	double mBF = 0;			//	封条宽度
	double mH1 = 0;			//	封条尺寸1
	double mL1 = 0;			//	封条尺寸2
	double mA = 0;				//	封条型面角度

	double mD = 0;				//	进口直径
	double mAH = 0;			//	进口高度
	double mAL = 0;			//	进口宽度
	double mLD = 0;			//	导流板区域长度
	int mND = 0;				//	导流片个数
	double mTD = 0;			//	导流片厚度

};
struct PFHEData
{
	double mLe = 0;				//	单元的有效长度
	double mBe = 0;				//	单元的有效宽度
	int mNL = 0;				//	总层数
	double mTgb = 0;			//	隔板
	double mNgb = 0;			//	隔板个数
	double mTs = 0;				//	外壳厚度
	
	FinData	CFinData;		//  冷端翅板数据
	FinData	HFinData;		//  热端翅板数据

};
class ClassPlateFinHE
{
public:
	ClassPlateFinHE();
	ClassPlateFinHE(PFHEData& tPFHEData, PFHELayerType tLayerType, const std::string& ExportPath);
	~ClassPlateFinHE();

public:
	PFHEData mPFHEData;
	std::string mExportPath;
	PFHELayerType mLayerType;

	TopoDS_Shape GenerateHeatExchanger();
	TopoDS_Shape Test();
};

