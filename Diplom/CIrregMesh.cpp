#include "CIrregMesh.h"

CIrregMesh::CIrregMesh() noexcept
{
}

std::vector<CIrregCell> CIrregMesh::FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume)
{
    constexpr double eps = 1e-8; // �������� � C++11, ���� �� �������� - �������� �� const double
    const CIrregCell& cell = cells[cellNumber]; // �������� ������ �����
    CPoint mid = (CVector(startPoint) + endPoint) / 2.; // �������� ������� [startPoint, endPoint]
    // ������ ���������, ������������ �� startPoint � endPoint
    CSideEquation plane(CVector(startPoint).createVector(endPoint)); 

    std::vector<CIrregCell> resultCells; // ����� ������.

    // ����� �������� �� ��������������� ������� �� ������� ���������

    double tempVolume{};

    while (true)
    {
        plane.recalcD(mid); // ������ ��������� �������� ����� ����� mid

        if (abs(tempVolume - volume) < eps)
            break;
        else if (tempVolume > volume)
            endPoint = mid;
        else
            startPoint = mid;
    }



    return resultCells;
}
