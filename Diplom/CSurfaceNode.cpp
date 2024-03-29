#include "CSurfaceNode.h"

extern const double EqualPointsEps;

CSurfaceNode::CSurfaceNode() noexcept
	: CPoint()
{
}

CSurfaceNode::CSurfaceNode(const double& _x, const double& _y, const double& _z) noexcept
	:CPoint(_x, _y, _z)
{
}

CSurfaceNode::CSurfaceNode(const CPoint& p) noexcept
	:CPoint(p)
{
}

CSurfaceNode::CSurfaceNode(const CPoint* p)
	:CPoint(p)
{
}

CSurfaceNode::CSurfaceNode(const CSurfaceNode& other) noexcept
	:CPoint(other.x, other.y, other.z)
{
}

CSurfaceNode::CSurfaceNode(const CSurfaceNode* other)
	:CSurfaceNode(*other)
{
}

bool CSurfaceNode::operator==(const CSurfaceNode& other) const noexcept
{
	return sqrt(pow(other.x - this->x, 2) + pow(other.y - this->y, 2) + pow(other.z - this->z, 2)) < EqualPointsEps; //-16 максимум
}
