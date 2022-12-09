#pragma once
#include "CVector.h"

class CSideEquation
{
public:
	CSideEquation() noexcept;
	CSideEquation(const double& _a, const double& _b, const double& _c, const double& _d) noexcept;
	CSideEquation(const CPoint& p1, const CPoint& p2, const CPoint& p3) noexcept;
	CSideEquation(const CPoint* p1, const CPoint* p2, const CPoint* p3);
	CSideEquation(const CPoint& p, const CVector& v1, const CVector& v2) noexcept;
	CSideEquation(const CPoint* p, const CVector* v1, const CVector* v2);
	CSideEquation(const CSideEquation& other) noexcept;
	CSideEquation(const CSideEquation* other);
	CSideEquation(const CVector& n) noexcept;
	CSideEquation(const CVector* n);
	CSideEquation(const CVector& n, const CPoint& p) noexcept;
	CSideEquation(const CVector* n, const CPoint* p);

	// Версии с одним параметром обнуляют коэффициет d, а с двуми - вычисляют его путём подстановки второго параметра.
	void setNormal(const CVector& n) noexcept;
	void setNormal(const CVector* n);
	void setNormal(const CVector& n, const CPoint& p) noexcept;
	void setNormal(const CVector* n, const CPoint* p);

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
	inline int getNormedValue(const CPoint& p) const noexcept
	{
		const double value = getValue(p);

		return abs(value) > 1e-4 ? (value > 0 ? 1 : (value < 0 ? -1 : 0)): 0;
	}
	inline double getNormedValue(const CPoint* p) const
	{
		return getNormedValue(*p);
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
	bool getIntersectionPoint(const CPoint& p1, const CPoint& p2, CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPoint(const CPoint* p1, const CPoint* p2, CPoint* const result = nullptr) const;
	bool getIntersectionPoint(const CPoint& p, const CVector& v,  CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPoint(const CPoint* p, const CVector* v,  CPoint* const result = nullptr) const;
	
	// Возращает true - если есть пересечение прямой и плоскости, причём точка находится на отрезке от p1 до p2.
	// В result записывается сама точка пересечения, если это необходимо.
	// Если пересечения нет - false и result не трогается.
	bool getIntersectionPointOnLine(const CPoint& p1, const CPoint& p2, CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPointOnLine(const CPoint* p1, const CPoint* p2, CPoint* const result = nullptr) const;
	bool getIntersectionPointOnLine(const CPoint& p, const CVector& v, CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPointOnLine(const CPoint* p, const CVector* v, CPoint* const result = nullptr) const;

	double a, b, c, d;
};

