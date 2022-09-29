#pragma once
#include "CVector.h"

class Tetrahedron
{
public:
	Tetrahedron() noexcept;
	CPoint Vert[4]; // 0 - вершина
	inline double getVolume() const noexcept
	{
		CVector v1 = CVector(Vert[0]).createVector(Vert[1]);
		CVector v2 = CVector(Vert[0]).createVector(Vert[2]);
		CVector v3 = CVector(Vert[0]).createVector(Vert[3]);
		
		return v1.Mixed(v2, v3) / 6;
	}
};

