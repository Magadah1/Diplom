#include "CIrregMesh.h"

CIrregMesh::CIrregMesh() noexcept
{
}

std::vector<CIrregCell> CIrregMesh::FindContactBorder(const int& cellNumber, const CPoint& startPoint, const CPoint& endPoint, const double& volume)
{
    constexpr double eps = 1e-8; // �������� � C++11, ���� �� �������� - �������� �� const double

    return std::vector<CIrregCell>();
}
