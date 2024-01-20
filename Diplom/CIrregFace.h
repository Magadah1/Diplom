#pragma once
#include <vector>
#include "CSideEquation.h"
#include "CException.h"
class CIrregFace
{
public:
	CIrregFace() noexcept;
	CIrregFace(const int& c1, const int& c2) noexcept;

	CIrregFace& operator=(const CIrregFace& other) noexcept;

	int cell1, cell2;
	std::vector<int> nodes; // ���� ������� �� ������� ������ cell1, �� �������� nodes � ������ �����������, ���� � cell2 - �� � ��������.
};
