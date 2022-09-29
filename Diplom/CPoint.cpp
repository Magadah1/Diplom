#include "CPoint.h"

CPoint::CPoint() noexcept
	: x(0.), y(0.), z(0.)
{
}

CPoint::CPoint(const double& _x, const double& _y, const double& _z) noexcept
	:x(_x), y(_y), z(_z)
{
}

CPoint::CPoint(const CPoint& other) noexcept
	:x(other.x), y(other.y), z(other.z)
{
}

CPoint::CPoint(const CPoint* other)
	:CPoint(*other)
{
}

CPoint& CPoint::operator=(const CPoint& other) noexcept
{
	if (&other == this)
		return *this;

	x = other.x;
	y = other.y;
	z = other.z;

	return *this;
}
