#pragma once
#include "CVector.h"
#include "CSideEquation.h"
#include "CException.h"
#include <array>

// обход тетраэдра таков, что вершина 0 находится по положительную сторону основания уровнения основания
class Tetrahedron
{
public:
	Tetrahedron() noexcept; 
	Tetrahedron(const CPoint& p0, const CPoint& p1, const CPoint& p2, const CPoint& p3) noexcept; // p0 - вершина, остальные - основание в правильном обходе
	Tetrahedron(const CPoint* p0, const CPoint* p1, const CPoint* p2, const CPoint* p3); // p0 - вершина, остальные - основание в правильном обходе
	Tetrahedron(const CPoint& p, const CVector& v1, const CVector& v2, const CVector& v3) noexcept; // p0 - вершина, v[1,2,3] - ведут к вершинам основания в правильном обходе
	Tetrahedron(const CPoint* p, const CVector* v1, const CVector* v2, const CVector* v3); // p0 - вершина, v[1,2,3] - ведут к вершинам основания в правильном обходе
	Tetrahedron(const Tetrahedron& other);
	Tetrahedron& operator=(const Tetrahedron& other);
	CPoint Vert[4]; // 0 - вершина
	inline double getVolume() const noexcept
	{
		CVector v1 = CVector(Vert[0]).createVector(Vert[1]);
		CVector v2 = CVector(Vert[0]).createVector(Vert[2]);
		CVector v3 = CVector(Vert[0]).createVector(Vert[3]);
		
		return abs(v1.Mixed(v2, v3)) / 6.;
	}
	CSideEquation getSideEquation(int sideId) const; // 0 - основание, [1,2,3] - заменяет данную вершину на главную вершину
	std::array<CPoint, 3> getSideVertices(int sideId) const;// 0 - основание, [1,2,3] - заменяет данную вершину на главную вершину
	inline int size() const { return 4; }
};
