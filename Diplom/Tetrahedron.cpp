#include "Tetrahedron.h"

Tetrahedron::Tetrahedron() noexcept
	: Vert{}
{
}

Tetrahedron::Tetrahedron(const CPoint& p0, const CPoint& p1, const CPoint& p2, const CPoint& p3) noexcept
{
	Vert[0] = p0;
	Vert[1] = p1;
	Vert[2] = p2;
	Vert[3] = p3;
}

Tetrahedron::Tetrahedron(const CPoint* p0, const CPoint* p1, const CPoint* p2, const CPoint* p3)
	: Tetrahedron(*p0, *p1, *p2, *p3)
{
}

Tetrahedron::Tetrahedron(const CPoint& p, const CVector& v1, const CVector& v2, const CVector& v3) noexcept
	: Tetrahedron(p, (v1 + p).likeCPoint(), (v2 + p).likeCPoint(), (v3 + p).likeCPoint())
{
}

Tetrahedron::Tetrahedron(const CPoint* p, const CVector* v1, const CVector* v2, const CVector* v3)
	: Tetrahedron(*p, *v1, *v2, *v3)
{
}

Tetrahedron::Tetrahedron(const Tetrahedron& other)
{
	Vert[0] = other.Vert[0];
	Vert[1] = other.Vert[1];
	Vert[2] = other.Vert[2];
	Vert[3] = other.Vert[3];
}

Tetrahedron& Tetrahedron::operator=(const Tetrahedron& other)
{
	if (this == &other)
		return *this;

	Vert[0] = other.Vert[0];
	Vert[1] = other.Vert[1];
	Vert[2] = other.Vert[2];
	Vert[3] = other.Vert[3];

	return *this;
}

CSideEquation Tetrahedron::getSideEquation(int sideId) const
{
	return CSideEquation(getSideVertices(sideId));
}

std::array<CPoint, 3> Tetrahedron::getSideVertices(int sideId) const
{
	switch (sideId)
	{
	case 0:
		return {Vert[1], Vert[2], Vert[3]};
	case 1:
		return {Vert[3], Vert[2], Vert[0]};
	case 2:
		return {Vert[3], Vert[0], Vert[1]};
	case 3:
		return {Vert[0], Vert[2], Vert[1]};
	default:
		throw CException("Неверный индекс грани!", { {"Принятый", sideId}, {"Максимальный", 3} });
	}
}
