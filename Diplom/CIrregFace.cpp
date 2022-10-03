#include "CIrregFace.h"

CIrregFace::CIrregFace() noexcept
	:cell1(-1), cell2(-1)
{
}

CIrregFace::CIrregFace(const int& c1, const int& c2) noexcept
	: cell1(c1), cell2(c2)
{
}
