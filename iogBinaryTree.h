//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Sun Gan
//   Date   : 2022/04/19
//
//=============================================================================
#ifndef _IOGBINARYTREE_H_
#define _IOGBINARYTREE_H_

#include<iostream>
#include"base.h"

struct stuPoint
{
public:
	double dCrd[3];
	double dTol = MATH_ZERO_D6;

	stuPoint(const double dC[3], const double dT)
	{
		dCrd[0] = dC[0];
		dCrd[1] = dC[1];
		dCrd[2] = dC[2];
		dTol = dT;
	}

	stuPoint(const double dC[3])
	{
		dCrd[0] = dC[0];
		dCrd[1] = dC[1];
		dCrd[2] = dC[2];
	}

	stuPoint(const stuPoint & pnt)
	{
		dCrd[0] = pnt.dCrd[0];
		dCrd[1] = pnt.dCrd[1];
		dCrd[2] = pnt.dCrd[2];
		dTol = pnt.dTol;
	}

	stuPoint & operator=(const stuPoint & v)
	{
		dCrd[0] = v.dCrd[0];
		dCrd[1] = v.dCrd[1];
		dCrd[2] = v.dCrd[2];
		dTol = v.dTol;
		return * this;
	}

	bool operator==(stuPoint & v)
	{
		return((fabs(dCrd[0] - v.dCrd[0]) <= dTol) && (fabs(dCrd[1] - v.dCrd[1]) <= dTol) && (fabs(dCrd[2] - v.dCrd[2]) <= dTol));
	}

	bool operator!=(stuPoint & v)
	{
		return((fabs(dCrd[0] - v.dCrd[0]) > dTol) || (fabs(dCrd[1] - v.dCrd[1]) > dTol) || (fabs(dCrd[2] - v.dCrd[2]) > dTol));
	}

	bool operator<(stuPoint & v)
	{
		return ((v.dCrd[0] - dCrd[0] > dTol) || (fabs(v.dCrd[0] - dCrd[0]) <= dTol && v.dCrd[1] - dCrd[1] > dTol) || (fabs(v.dCrd[0] - dCrd[0]) <= dTol && fabs(v.dCrd[1] - dCrd[1]) <= dTol && v.dCrd[2] - dCrd[2] > dTol));
		
	}

	bool operator>(stuPoint & v)
	{
		return((dCrd[0] - v.dCrd[0] > dTol) || (fabs(dCrd[0] - v.dCrd[0]) <= dTol && dCrd[1] - v.dCrd[1] > dTol) || (fabs(dCrd[0] - v.dCrd[0]) <= dTol && fabs(dCrd[1] - v.dCrd[1]) <= dTol && dCrd[2] - v.dCrd[2] > dTol));
	}

	double GetTol()const
	{
		return dTol;
	}

	void SetTolerance(double tol)
	{
		dTol = tol;
	}

	friend std::ostream & operator<<(std::ostream & os,const stuPoint & pnt)
	{
		os << pnt.dCrd[0] << "," << pnt.dCrd[1] << "," << pnt.dCrd[2] << std::endl;
		return os;
	}
};

struct iogBinaryNode
{
public:
	stuPoint Point;
	int nHeight;
	long long llVert = 0;
	iogBinaryNode * pLeftTree;
	iogBinaryNode * pRightTree;

	iogBinaryNode(const stuPoint & p) :Point(p), nHeight(0), pLeftTree(nullptr), pRightTree(nullptr) {};
	iogBinaryNode(const stuPoint & p, iogBinaryNode * pLeft, iogBinaryNode * pRight) :Point(p), nHeight(0), pLeftTree(pLeft), pRightTree(pRight) {};

	int GetHeight()const { return nHeight; }
	stuPoint GetPoint()const { return Point; }

	friend std::ostream & operator<<(std::ostream & os,const iogBinaryNode & pnt)
	{
		os << pnt << "Left Tree: " << pnt.pLeftTree << std::endl << "Right Tree: " << pnt.pRightTree << std::endl << "Height; " << pnt.nHeight << std::endl;
		return os;
	}
};

class iogBinaryTree
{
public:
	iogBinaryTree();
	~iogBinaryTree();

	iogBinaryNode * Search(stuPoint pnt) const;
	iogBinaryNode * GetRoot()const;

	stuPoint * Minimum() const;
	stuPoint * Maximum() const;

	int Max(int a, int b) const;
	int TreeHeight() const;

	long long ReGetSameIdx(stuPoint pnt)const;

	void Purge();
	void PrintTree() const;
	void PreOrder() const;
	void InOrder() const;
	void PostOrder() const;
	void Insert(stuPoint pnt);

	bool Contain(stuPoint pnt) const;
	bool IsEmpty() const;

	long long m_llPointNum = 0;

private:
	iogBinaryNode * MaxNode(iogBinaryNode * root) const;
	iogBinaryNode * MinNode(iogBinaryNode * root) const;
	iogBinaryNode * Search(iogBinaryNode * root, stuPoint pnt)const;
	iogBinaryNode * GetRootNode()const;

	int Height(iogBinaryNode *root)const;
	int GetTreeHeight(iogBinaryNode *root)const;

	bool Contain(iogBinaryNode * root, stuPoint pnt) const;

	void PrintTree(iogBinaryNode * root, stuPoint pnt, int nDir) const;
	void Purge(iogBinaryNode * root);
	void PreOrder(iogBinaryNode * root) const;
	void InOrder(iogBinaryNode * root) const;
	void PostOrder(iogBinaryNode * root) const;
	void Insert(iogBinaryNode * &root, stuPoint pnt);

	iogBinaryNode * m_pRootNode;

};

#endif // !_IOGBINARYTREE_H_

