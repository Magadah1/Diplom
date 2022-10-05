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
