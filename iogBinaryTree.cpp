//=============================================================================
//
//   Copyright (c) China Automotive Innovation Corporation.
//
//   Author : Sun Gan
//   Date   : 2022/04/19
//
//=============================================================================
#include "iogBinaryTree.h"
iogBinaryTree::iogBinaryTree() : m_pRootNode(nullptr)
{

}

iogBinaryTree::~iogBinaryTree()
{
	Purge(m_pRootNode);
}
//////////////////////////////////////////////////////////////////////////////
//
int iogBinaryTree::Max(int a, int b)const
{
	return a > b ? a : b;
}

int iogBinaryTree::Height(iogBinaryNode * root)const
{
	if (root == nullptr)
	{
		return 0;
	}
	return root->nHeight;
}

int iogBinaryTree::GetTreeHeight(iogBinaryNode * root)const
{
	if (root == nullptr)
	{
		return -1;
	}
	else
	{
		return root->nHeight = Max(GetTreeHeight(root->pLeftTree), GetTreeHeight(root->pRightTree)) + 1;
	}
}

int iogBinaryTree::TreeHeight()const
{
	return GetTreeHeight(m_pRootNode);
}
//////////////////////////////////////////////////////////////////////////////
//
long long iogBinaryTree::ReGetSameIdx(stuPoint pnt)const
{
	long long llSameIdx;
	iogBinaryNode * pSearch = Search(pnt);
	if (pSearch == nullptr)
	{
		llSameIdx = -1;
	}
	else
	{
		llSameIdx = pSearch->llVert;
	}
	return llSameIdx;
}
//////////////////////////////////////////////////////////////////////////////
//
bool iogBinaryTree::Contain(iogBinaryNode * root, stuPoint pnt) const
{
	iogBinaryNode * pRes = Search(root, pnt);
	if (pRes == nullptr)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool iogBinaryTree::Contain(stuPoint pnt)const
{
	return Contain(m_pRootNode, pnt);
}

bool iogBinaryTree::IsEmpty()const
{
	return (m_llPointNum == 0);
}
//////////////////////////////////////////////////////////////////////////////
//
iogBinaryNode * iogBinaryTree::MaxNode(iogBinaryNode * root)const
{
	if (root == nullptr)
	{
		return nullptr;
	}

	while (root->pRightTree != nullptr)
	{
		root = root->pRightTree;
	}
	return root;
}

iogBinaryNode * iogBinaryTree::MinNode(iogBinaryNode * root)const
{
	if (root == nullptr)
	{
		return nullptr;
	}

	while (root->pLeftTree != nullptr)
	{
		root = root->pLeftTree;
	}
	return root;
}

iogBinaryNode * iogBinaryTree::GetRootNode()const
{
	return m_pRootNode;
}

iogBinaryNode * iogBinaryTree::Search(iogBinaryNode * root, stuPoint pnt)const
{
	while (root != nullptr && root->Point != pnt)
	{
		if (root->Point > pnt)
		{
			root = root->pLeftTree;
		}
		else if (root->Point < pnt)
		{
			root = root->pRightTree;
		}
	}
	return root;
}

void iogBinaryTree::Insert(iogBinaryNode * &root, stuPoint pnt)
{
	iogBinaryNode * pNewNode = new iogBinaryNode(pnt, nullptr, nullptr);
	if (!pNewNode)
	{
		return;
	}

	++m_llPointNum;

	if (root == nullptr)
	{
		root = pNewNode;
		root->llVert = m_llPointNum;
		return;
	}

	iogBinaryNode * pRec = root;
	iogBinaryNode * pParent = nullptr;

	while (pRec != nullptr)
	{
		pParent = pRec;

		if (pnt < pRec->Point)
		{
			pRec = pRec->pLeftTree;
		}
		else
		{
			pRec = pRec->pRightTree;
		}
	}

	if (pnt < pParent->Point)
	{
		pParent->pLeftTree = pNewNode;
	}
	else
	{
		pParent->pRightTree = pNewNode;
	}

	pNewNode->llVert = m_llPointNum;

	return;
}

iogBinaryNode * iogBinaryTree::Search(stuPoint pnt) const
{
	return Search(m_pRootNode, pnt);
}

iogBinaryNode * iogBinaryTree::GetRoot()const
{
	return m_pRootNode;
}

//////////////////////////////////////////////////////////////////////////////
//
stuPoint * iogBinaryTree::Minimum() const
{
	iogBinaryNode * pRes = MinNode(m_pRootNode);
	if (pRes != nullptr)
	{
		return &pRes->Point;
	}
	return nullptr;
}

stuPoint * iogBinaryTree::Maximum() const
{
	iogBinaryNode * pRes = MaxNode(m_pRootNode);
	if (pRes != nullptr)
	{
		return &pRes->Point;
	}
	return nullptr;
}
//////////////////////////////////////////////////////////////////////////////
//
void iogBinaryTree::Purge()
{
	Purge(m_pRootNode);
}

void iogBinaryTree::Insert(stuPoint pnt)
{
	Insert(m_pRootNode, pnt);
}

void iogBinaryTree::Purge(iogBinaryNode * root)
{
	if (root == nullptr)
	{
		return;
	}
	if (root->pLeftTree != nullptr)
	{
		Purge(root->pLeftTree);
	}
	if (root->pRightTree != nullptr)
	{
		Purge(root->pRightTree);
	}
	delete root;
}

void iogBinaryTree::PrintTree(iogBinaryNode * root, stuPoint pnt, int nDir) const
{

}

void iogBinaryTree::PreOrder(iogBinaryNode * root) const
{
	if (root != nullptr)
	{
		std::cout << root->Point << " ";
		PreOrder(root->pLeftTree);
		PreOrder(root->pRightTree);
	}
}

void iogBinaryTree::InOrder(iogBinaryNode * root) const
{
	if (root != nullptr)
	{
		PreOrder(root->pLeftTree);
		std::cout << root->Point << " ";
		PreOrder(root->pRightTree);
	}
}

void iogBinaryTree::PostOrder(iogBinaryNode * root) const
{
	if (root != nullptr)
	{
		PreOrder(root->pLeftTree);
		PreOrder(root->pRightTree);
		std::cout << root->Point << " ";
	}
}

void iogBinaryTree::PrintTree() const
{
	if (m_pRootNode != nullptr)
	{
		PrintTree(m_pRootNode, m_pRootNode->Point, 0);
	}
}

void iogBinaryTree::PreOrder() const
{
	PreOrder(m_pRootNode);
}

void iogBinaryTree::InOrder() const
{
	InOrder(m_pRootNode);
}

void iogBinaryTree::PostOrder() const
{
	PostOrder(m_pRootNode);
}