#pragma once
#include "CVector.h"

class Tetrahedron
{
public:
	Tetrahedron() noexcept;
	Tetrahedron(const CPoint& p0, const CPoint& p1, const CPoint& p2, const CPoint& p3) noexcept;
	Tetrahedron(const CPoint* p0, const CPoint* p1, const CPoint* p2, const CPoint* p3);
	Tetrahedron(const CPoint& p, const CVector& v1, const CVector& v2, const CVector& v3) noexcept;
	Tetrahedron(const CPoint* p, const CVector* v1, const CVector* v2, const CVector* v3);
	CPoint Vert[4]; // 0 - вершина
	inline double getVolume() const noexcept
	{
		CVector v1 = CVector(Vert[0]).createVector(Vert[1]);
		CVector v2 = CVector(Vert[0]).createVector(Vert[2]);
		CVector v3 = CVector(Vert[0]).createVector(Vert[3]);
		
		return abs(v1.Mixed(v2, v3)) / 6.;
	}
};

