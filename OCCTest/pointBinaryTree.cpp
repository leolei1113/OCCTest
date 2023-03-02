#include "pointBinaryTree.h"

pointBinaryTree::pointBinaryTree() :m_pRoot(nullptr)
{

}

pointBinaryTree::~pointBinaryTree()
{
	Purge(m_pRoot);
}

void pointBinaryTree::Purge(binaryNode* root)
{
	if (!root)
		return;
	if (root->pLeftTree)
	{
		Purge(root->pLeftTree);
	}
	if (root->pRightTree)
	{
		Purge(root->pRightTree);
	}
	delete root;
}

void pointBinaryTree::Insert(Point pt)
{
	Insert(m_pRoot, pt);
}

void pointBinaryTree::Insert(binaryNode* root, Point p)
{
	binaryNode* newNode = new binaryNode(p, nullptr, nullptr);
	m_llPointNum++;

	if (!root)
	{
		root = newNode;
		root->llvert = m_llPointNum;
		return;
	}

	binaryNode* pCurrent = root, *pParent = nullptr;

	while(!pCurrent)
	{
		pParent = pCurrent;
		if (p < pCurrent->pt)
		{
			pCurrent = pCurrent->pLeftTree;
		}
		else
		{
			pCurrent = pCurrent->pRightTree;
		}
	}

	if (p < pParent->pt)
	{
		pParent->pLeftTree = newNode;
	}
	else
	{
		pParent->pRightTree = newNode;
	}
	newNode->llvert = m_llPointNum;
	return;
}

binaryNode* pointBinaryTree::Search(binaryNode* root, Point p)
{
	while (root && root->pt != p)
	{
		if (root->pt > p)
		{
			root = root->pLeftTree;
		}
		else if (root->pt < p)
		{
			root = root->pRightTree;
		}
	}
	return root;
}