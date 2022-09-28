#include "CSideEquation.h"

CSideEquation::CSideEquation() noexcept
	: a(0.), b(0.), c(0.), d(0.)
{
}

CSideEquation::CSideEquation(const CPoint& p1, const CPoint& p2, const CPoint& p3) noexcept
{
	double x1 = p2.X() - p1.X();
	double y1 = p2.Y() - p1.Y();
	double z1 = p2.Z() - p1.Z();
	double x2 = p3.X() - p1.X();
	double y2 = p3.Y() - p1.Y();
	double z2 = p3.Z() - p1.Z();

	a = y1 * z2 - z1 * y2;
	b = x2 * z1 - x1 * z2;
	c = x1 * y2 - x2 * y1;

	d = -(p1.X() * a + p1.Y() * b + p1.Z() * c);
}

CSideEquation::CSideEquation(const CPoint* p1, const CPoint* p2, const CPoint* p3)
	: CSideEquation(*p1, *p2, *p3)
{
}

CSideEquation::CSideEquation(const CPoint& p, const CVector& v1, const CVector& v2) noexcept
	:CSideEquation(p, v1.toCPoint(), v2.toCPoint())
{
}

CSideEquation::CSideEquation(const CPoint* p, const CVector* v1, const CVector* v2)
	: CSideEquation(*p, *v1, *v2)
{
}

CSideEquation::CSideEquation(const CSideEquation& other) noexcept
	: a(other.a), b(other.b), c(other.c), d(other.d)
{
}

CSideEquation::CSideEquation(const CSideEquation* other)
	:CSideEquation(*other)
{
}

bool CSideEquation::getIntersectionPoint(const CPoint& p1, const CPoint& p2, CPoint* result) const noexcept
{
	CVector r1(p1);
	CVector s1 = CVector(p1).createVector(p2);
	CVector n2 = getNormal();

	double mult = s1 * n2;

	if (abs(mult) < 1e-6) // прямая параллельна плоскости
		return false;

	if (result)
		*result = (r1 - s1 * (r1 * n2 + d) / mult).toCPoint();

	return true;
}

bool CSideEquation::getIntersectionPoint(const CPoint* p1, const CPoint* p2, CPoint* result) const
{
	return getIntersectionPoint(*p1, *p2, result);
}

bool CSideEquation::getIntersectionPoint(const CPoint& p, const CVector& v, CPoint* result) const noexcept
{
	return getIntersectionPoint(p, (v + p).toCPoint(), result);
}

bool CSideEquation::getIntersectionPoint(const CPoint* p, const CVector* v, CPoint* result) const
{
	return getIntersectionPoint(*p, *v, result);
}
