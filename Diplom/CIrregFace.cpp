#include "CIrregFace.h"

CIrregFace::CIrregFace() noexcept
	:cell1(0), cell2(0)
{
}

CIrregFace::CIrregFace(const int& c1, const int& c2) noexcept
	: cell1(c1), cell2(c2)
{
}
