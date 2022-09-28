#include "CVector.h"

CVector::CVector() noexcept
	:CPoint()
{
}

CVector::CVector(const double& _x, const double& _y, const double& _z) noexcept
	: CPoint(_x, _y, _z)
{
}

CVector::CVector(const double& _xS, const double& _yS, const double& _zS, const double& _xE, const double& _yE, const double& _zE) noexcept
	: CPoint(_xE - _xS, _yE - _yS, _zE - _zS)
{
}

CVector::CVector(const CPoint& p) noexcept
	:CPoint(p)
{
}

CVector::CVector(const CPoint* p)
	:CVector(*p)
{
}

CVector::CVector(const CPoint& start, const CPoint& end) noexcept
	: CPoint(end.X() - start.X(), end.Y() - start.Y(), end.Z() - start.Z())
{
}

CVector::CVector(const CPoint* start, const CPoint* end)
	: CPoint(end->X() - start->X(), end->Y() - start->Y(), end->Z() - start->Z())
{
}

CVector::CVector(const CVector& other) noexcept
	:CPoint(other.x, other.y, other.z)
{
}

CVector::CVector(const CVector* other)
	: CVector(*other)
{
}