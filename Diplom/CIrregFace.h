#pragma once
#include <vector>

class CIrregFace
{
public:
	CIrregFace() noexcept;
	CIrregFace(const int& c1, const int& c2) noexcept;
	int cell1, cell2;
	std::vector<int> nodes;
};
