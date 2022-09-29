#pragma once
#include <vector>

class CIrregFace
{
public:
	CIrregFace() noexcept;
	CIrregFace(const int& c1, const int& c2) noexcept;
	int cell1, cell2;
	std::vector<int> nodes; // Если смотрим со стороны клетки cell1, то проходим nodes в прямом направлении, если с cell2 - то в обратном.
};
