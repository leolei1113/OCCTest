#pragma once
#include <iostream>

struct Point
{
public:
	double x, y, z;
	double dTol = 1e-12;

	Point(const double x1, const double y1, const double z1, const double dT)
	{
		x = x1;
		y = y1;
		z = z1;
		dTol = dT;
	}

	Point(const double x1, const double y1, const double z1)
	{
		x = x1;
		y = y1;
		z = z1;
	}

	Point(const Point& p1)
	{
		x = p1.x;
		y = p1.y;
		z = p1.z;
	}

	Point& operator=(const Point& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		dTol = v.dTol;
		return *this;
	}

	bool operator==(Point v)
	{
		return((fabs(x - v.x) <= dTol) && (fabs(y - v.y) <= dTol) && (fabs(z - v.z) <= dTol));
	}

	bool operator!=(Point v)
	{
		return((fabs(x - v.x) > dTol) || (fabs(y - v.y) > dTol) || (fabs(z - v.z) > dTol));
	}

	bool operator<(Point v)
	{
		return (
			(v.x - x > dTol) ||
			((v.x - x <= dTol) && v.y - y > dTol) ||
			((v.y - y <= dTol) && v.z - z > dTol)
			);
	}

	bool operator>(Point v)
	{
		return (
			(v.x - x < dTol) ||
			((v.x - x >= dTol) && v.y - y < dTol) ||
			((v.y - y >= dTol) && v.z - z < dTol)
			);
	}
};

struct binaryNode
{
public:
	Point pt;
	int nHeight = 0;
	long long llvert = 0;
	binaryNode* pLeftTree, * pRightTree;

	binaryNode(Point& p): pt(p){};
	binaryNode(Point& p, binaryNode* pLeftTree, binaryNode* pRightTree) : pt(p) {};

	int GetHeight() { return nHeight; }
	Point GetPoint() { return pt };
};
class pointBinaryTree
{
public:
	pointBinaryTree();
	~pointBinaryTree();

	binaryNode* Search(Point pt);
	binaryNode* GetRoot();

	int TreeHeight();

	long long ReGetSameIdx(Point pt);

	void Insert(Point pt);
	void Insert(binaryNode* root, Point pt);

	binaryNode* Search(binaryNode* root, Point p);

	binaryNode* m_pRoot;

	void Purge(binaryNode* root);
	long long m_llPointNum = 0;
};

