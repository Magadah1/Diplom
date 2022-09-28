#pragma once
#include "CVector.h"

class CSideEquation
{
public:
	CSideEquation() noexcept;
	CSideEquation(const CPoint& p1, const CPoint& p2, const CPoint& p3) noexcept;
	CSideEquation(const CPoint* p1, const CPoint* p2, const CPoint* p3);
	CSideEquation(const CPoint& p, const CVector& v1, const CVector& v2) noexcept;
	CSideEquation(const CPoint* p, const CVector* v1, const CVector* v2);
	CSideEquation(const CSideEquation& other) noexcept;
	CSideEquation(const CSideEquation* other);

	inline CVector getNormal() const noexcept
	{
		return CVector(a, b, c);
	}
	inline double getValue(const CPoint& p) const noexcept
	{
		return a * p.X() + b * p.Y() + c * p.Z() + d;
	}
	inline double getValue(const CPoint* p) const
	{
		return getValue(*p);
	}
	inline double recalcD(const CPoint& p) noexcept
	{
		d = -(a * p.X() + b * p.Y() + c * p.Z());
		return d;
	}
	inline double recalcD(const CPoint* p)
	{
		return recalcD(*p);
	}
	
	// Возвращает true - если есть пересечение прямой и плоскости. В result записывается сама точка пересечения, если это необходимо. 
	// Если пересечения нет - false и result не трогается.
	bool getIntersectionPoint(const CPoint& p1, const CPoint& p2, CPoint* result = nullptr) const noexcept;
	bool getIntersectionPoint(const CPoint* p1, const CPoint* p2, CPoint* result = nullptr) const;
	bool getIntersectionPoint(const CPoint& p, const CVector& v, CPoint* result = nullptr) const noexcept;
	bool getIntersectionPoint(const CPoint* p, const CVector* v, CPoint* result = nullptr) const;
	
	double a, b, c, d;
};

