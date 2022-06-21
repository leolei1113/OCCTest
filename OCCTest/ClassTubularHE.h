#pragma once
#include "TopoDS_Shape.hxx"

struct TSHEData
{
	double mDtin;	//进口管内径
	double mTtin;	//进口管壁厚
	double mAtin;	//进口管与轴线夹角
	double mHtin;	//进口管长度
	double mDtout;	//出口管内径
	double mTtout;	//出口管壁厚
	double mAtout;	//出口管与轴线夹角
	double mHtout;	//出口管长度
	double mdt;		//管子内径
	double mTt;		//管子壁厚
	int	   mNt;		//管子个数
	double mLt;		//管子间距（三角形排布）
	double mLtin;	//内管进口段延伸长度
	double mLtout;	//内管出口段延伸长度
	double mRt;		//U型管转弯半径
	double mTE;		//分层隔板厚度
	double mlE;		//第一层管子间距

	double mDsin;	//进口管内径
	double mTsin;	//进口管壁厚
	double mBsin;	//进口管中心与端壁距离
	double mHsin;	//进口管长度
	double mDsout;	//出口管内径
	double mTsout;	//出口管壁厚
	double mBsout;	//出口管中心与端壁距离
	double mHsout;	//出口管长度
	double mTzlb;	//弓形折流板厚度
	double mhqk;	//弓形折流板缺口高度
	double mLzlb;	//弓形折流板间距
	double mTgb;	//管板厚度
	double mLs;		//壳体长度
	double mTs;		//壳体壁厚
	double mRs;		//球型封头内径
	double mDs;		//壳体内径
	double mDzlb;	//折流板直径
	double mDkk;		//开孔直径

	int mNzlb;		//折流板数
	int mNtmax;		//最大管数
	int mNlayer;	//管子层数

};


class ClassTubularHE
{
public:
	ClassTubularHE();
	ClassTubularHE(int tTurbineSide, TSHEData tTSHEData, const std::string& ExportPath);
	~ClassTubularHE();

public:
	int mTurbineSide; //管程
	TSHEData mTSHEData;
	std::string mExportPath;

	TopoDS_Shape GenerateHeatExchanger();
};

;