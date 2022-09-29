#include "CIrregMesh.h"

CIrregMesh::CIrregMesh() noexcept
{
}

std::vector<CIrregCell> CIrregMesh::FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume)
{
    constexpr double eps = 1e-8; // по€вилс€ в C++11, если не работает - заменить на const double
    const CIrregCell& cell = cells[cellNumber]; // получаем клетку сетки
    CPoint mid = (CVector(startPoint) + endPoint) / 2.; // —ередина отрезка [startPoint, endPoint]
    // —троим плоскость, направленную от startPoint к endPoint
    CSideEquation plane(CVector(startPoint).createVector(endPoint)); 

    std::vector<CIrregCell> resultCells; // Ќовые клетки.

    // ќбъЄм отсекаем по противоположную сторону от нормали плоскости

    double tempVolume{};

    while (true)
    {
        plane.recalcD(mid); // теперь плоскость проходит через точку mid

        if (abs(tempVolume - volume) < eps)
            break;
        else if (tempVolume > volume)
            endPoint = mid;
        else
            startPoint = mid;
    }



    return resultCells;
}
