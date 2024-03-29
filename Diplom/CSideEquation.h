#pragma once
#include "CVector.h"
#include <array>

class CSideEquation
{
public:
	CSideEquation() noexcept;
	CSideEquation(const double& _a, const double& _b, const double& _c, const double& _d) noexcept;
	CSideEquation(const CPoint& p1, const CPoint& p2, const CPoint& p3) noexcept;
	CSideEquation(const CPoint* p1, const CPoint* p2, const CPoint* p3);
	CSideEquation(const CPoint& p, const CVector& v1, const CVector& v2) noexcept;
	CSideEquation(const CPoint* p, const CVector* v1, const CVector* v2);
	CSideEquation(const std::array<CPoint, 3>& vs) noexcept;
	CSideEquation(const CSideEquation& other) noexcept;
	CSideEquation(const CSideEquation* other);
	CSideEquation(const CVector& n) noexcept;
	CSideEquation(const CVector* n);
	CSideEquation(const CVector& n, const CPoint& p) noexcept;
	CSideEquation(const CVector* n, const CPoint* p);

	// ������ � ����� ���������� �������� ���������� d, � � ����� - ��������� ��� ���� ����������� ������� ���������.
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
	int getNormedValue(const CPoint& p) const noexcept;
	inline int getNormedValue(const CPoint* p) const
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
	inline CSideEquation& Normalize()
	{
		const double nL = CVector(a, b, c).Length();
		if (abs(nL) < 1e-6) return *this;
		const double _nl = 1. / nL;
		a *= _nl;
		b *= _nl;
		c *= _nl;
		d *= _nl;
		return *this;
	}
	
	// ���������� true - ���� ���� ����������� ������ � ���������. � result ������������ ���� ����� �����������, ���� ��� ����������. 
	// ���� ����������� ��� - false � result �� ���������.
	bool getIntersectionPoint(const CPoint& p1, const CPoint& p2, CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPoint(const CPoint* p1, const CPoint* p2, CPoint* const result = nullptr) const;
	bool getIntersectionPoint(const CPoint& p, const CVector& v,  CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPoint(const CPoint* p, const CVector* v,  CPoint* const result = nullptr) const;
	
	// ��������� true - ���� ���� ����������� ������ � ���������, ������ ����� ��������� �� ������� �� p1 �� p2.
	// � result ������������ ���� ����� �����������, ���� ��� ����������.
	// ���� ����������� ��� - false � result �� ���������.
	bool getIntersectionPointOnLine(const CPoint& p1, const CPoint& p2, CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPointOnLine(const CPoint* p1, const CPoint* p2, CPoint* const result = nullptr) const;
	bool getIntersectionPointOnLine(const CPoint& p, const CVector& v, CPoint* const result = nullptr) const noexcept;
	bool getIntersectionPointOnLine(const CPoint* p, const CVector* v, CPoint* const result = nullptr) const;

	double a, b, c, d;
};

