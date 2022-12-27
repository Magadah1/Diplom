#include "CSideEquation.h"

extern const double PlaneEps;

CSideEquation::CSideEquation() noexcept
	: a(0.), b(0.), c(0.), d(0.)
{
}

CSideEquation::CSideEquation(const double& _a, const double& _b, const double& _c, const double& _d) noexcept
	:a(_a), b(_b), c(_c), d(_d)
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

CSideEquation::CSideEquation(const CVector& n) noexcept
{
	setNormal(n);
}

CSideEquation::CSideEquation(const CVector* n)
{
	setNormal(n);
}

CSideEquation::CSideEquation(const CVector& n, const CPoint& p) noexcept
{
	setNormal(n, p);
}

CSideEquation::CSideEquation(const CVector* n, const CPoint* p)
{
	setNormal(n, p);
}

void CSideEquation::setNormal(const CVector& n) noexcept
{
	a = n.X();
	b = n.Y();
	c = n.Z();
	d = 0.;
}

void CSideEquation::setNormal(const CVector* n)
{
	setNormal(*n);
}

void CSideEquation::setNormal(const CVector& n, const CPoint& p) noexcept
{
	setNormal(n);
	recalcD(p);
}

void CSideEquation::setNormal(const CVector* n, const CPoint* p)
{
	setNormal(*n, *p);
}

int CSideEquation::getNormedValue(const CPoint& p) const noexcept
{
	const double value = getValue(p);

	return abs(value) > PlaneEps ? (value > 0 ? 1 : (value < 0 ? -1 : 0)) : 0; //-13 максимум
}

bool CSideEquation::getIntersectionPoint(const CPoint& p1, const CPoint& p2, CPoint* const result) const noexcept
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

bool CSideEquation::getIntersectionPoint(const CPoint* p1, const CPoint* p2, CPoint* const result) const
{
	return getIntersectionPoint(*p1, *p2, result);
}

bool CSideEquation::getIntersectionPoint(const CPoint& p, const CVector& v, CPoint* const result) const noexcept
{
	return getIntersectionPoint(p, (v + p).likeCPoint(), result);
}

bool CSideEquation::getIntersectionPoint(const CPoint* p, const CVector* v, CPoint* const result) const
{
	return getIntersectionPoint(*p, *v, result);
}

bool CSideEquation::getIntersectionPointOnLine(const CPoint& p1, const CPoint& p2, CPoint* const result) const noexcept
{
	/*CPoint res;

	if (!getIntersectionPoint(p1, p2, &res))
		return false;*/

	// Сравнение по длинам составных отрезков
	/*if (abs(CVector(p1).createVector(res).Length() + CVector(res).createVector(p2).Length() - CVector(p1).createVector(p2).Length()) < 1e-6)
	{
		if (result)
			*result = res;
		return true;
	}

	return false;*/

	double valueP1 = getNormedValue(p1); 
	double valueP2 = getNormedValue(p2); 

	// значения valueP1[2] равны 1,0 или -1 если точка находится по направлению нормали, на плоскости или против направления нормали соответсвенно
	
	// более долгое стравнение (от 1 до 3 проверок)
	//if (
	//	valueP1 == -valueP2 // если знаки разные, то точки по разную сторону от плоскости => точка пересечения внутри отрезка
	//	||
	//	!valueP1 
	//	||					// одна из точек на плоскости, она и будет точкой пересечения.
	//	!valueP2
	//	)
	//{
	//	*result = res;
	//	return true;
	//}
	//return false;

	// более быстрое сравнение (от 1 до 2 проверок)
	if (
		valueP1 == valueP2 && valueP1 // точки одного знака и не на плоскости
		)
	{
		return false;
	}
	/*if (result)
		*result = res;
	return true;*/
	return getIntersectionPoint(p1, p2, result);
}

bool CSideEquation::getIntersectionPointOnLine(const CPoint* p1, const CPoint* p2, CPoint* const result) const
{
	return getIntersectionPointOnLine(*p1, *p2, result);
}

bool CSideEquation::getIntersectionPointOnLine(const CPoint& p, const CVector& v, CPoint* const result) const noexcept
{
	return getIntersectionPointOnLine(p, (v + p).likeCPoint(), result);
}

bool CSideEquation::getIntersectionPointOnLine(const CPoint* p, const CVector* v, CPoint* const result) const
{
	return getIntersectionPointOnLine(*p, *v, result);
}
