#include "CIrregFace.h"

CIrregFace::CIrregFace() noexcept
	:cell1(-1), cell2(-1)
{
}

CIrregFace::CIrregFace(const int& c1, const int& c2) noexcept
	: cell1(c1), cell2(c2)
{
}

CIrregFace& CIrregFace::operator=(const CIrregFace& other) noexcept
{
	if (this == &other)
		return *this;

	this->cell1 = other.cell1;
	this->cell2 = other.cell2;
	this->nodes = other.nodes;

	return *this;
}
