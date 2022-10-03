#include "CIrregMesh.h"

CIrregMesh::CIrregMesh() noexcept
{
}

std::vector<CIrregCell> CIrregMesh::FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume)
{
    constexpr double eps = 1e-8; // появился в C++11, если не работает - заменить на const double
    const CIrregCell& cell = cells[cellNumber]; // получаем клетку сетки

    CPoint mid = (CVector(startPoint) + endPoint) / 2.; // Середина отрезка [startPoint, endPoint]
    
    CSideEquation plane(CVector(startPoint).createVector(endPoint)); // Строим плоскость, направленную от startPoint к endPoint

    std::vector<CIrregCell> resultCells(2); // Новые клетки. 0 - та, чей объём ищем (по другую сторону от нормали). 1 - оставшаяся.
    int newCellId = cells.size(); // новый номер у клетки 1. У 0 он сохранится.
    int newFaceId = faces.size(); // Для каждой стороны, которая будет разбита новыми ячейками нужно добавить индекс. Эти будут для ячейки 1.
    int newNodeId = 0; // Для каждой новой вершины добавим индекс. Они будут отрицательными, но в их массиве будут реверсироваться.

    // Объём отсекаем по противоположную сторону от нормали плоскости
    double tempVolume{};

    //
    std::vector<CIrregFace> resultFaces; // Стороны новых клеток; СТРАННО
    std::vector<CSurfaceNode> planePoints; // Новые вершины, которые могут получится при пересечении с плоскостью отсечения. С "отрицательным индексом"
    //

    while (true)
    {
        // БЛОК 1. РАЗБИВАЕМ КЛЕТКУ НА 2 С ПОМОЩЬЮ ПЛОСКОСТИ.
        plane.recalcD(mid); // теперь плоскость проходит через точку mid
        
        // Проходим по всем сторонам клетки
        for (int cellFaceI = 0; cellFaceI < cell.facesInd.size(); ++cellFaceI) 
        {
            const CIrregFace& face = faces[cell.facesInd[cellFaceI]]; // текущая сторона

            // Потенциально разбиваем сторону клетки на 2. Первая сохранит свой номер в дальнейшем.
            CIrregFace newVolumeFace, newOtherFace;
            if (face.cell1 == cellNumber)
            {
                newVolumeFace.cell1 = cellNumber;
                newVolumeFace.cell2 = face.cell2;

                newOtherFace.cell1 = cellNumber;
                newOtherFace.cell2 = face.cell2;
            }
            else
            {
                newVolumeFace.cell2 = cellNumber;
                newVolumeFace.cell1 = face.cell1;

                newOtherFace.cell2 = cellNumber;
                newOtherFace.cell1 = face.cell1;
            }

            // Будем собирать точки пересечения.
            CSurfaceNode interPoint; 
            for (int faceNodeI = 0; faceNodeI < face.nodes.size(); ++faceNodeI) // А ПОРЯДОК?????????
            {
                const CSurfaceNode& Node1 = nodes[faceNodeI];                               // первая точка для отрезка.
                const CSurfaceNode& Node2 = nodes[(faceNodeI + 1ull) % face.nodes.size()];  // вторая точка для отрезка. Типа ребро.

                const double Node1V = plane.getNormedValue(Node1);
                const double Node2V = plane.getNormedValue(Node2);

                if (Node1V == Node2V && Node1V) // если вершины по одну сторону от плоскости, то забиваем на пересечение
                {
                    if (Node1V == -1)
                        newVolumeFace.nodes.push_back(faceNodeI);
                    else
                        newOtherFace.nodes.push_back(faceNodeI);
                }
                else
                {
                    if (Node1V == Node2V && !Node1V) // обе точки на плоскости, добавляем первую из них в каждую из новых клеток и добавляем к новой стороне, что образуется плоскостью.
                    {
                        newVolumeFace.nodes.push_back(faceNodeI);
                        newOtherFace.nodes.push_back(faceNodeI);
                        planePoints.push_back(Node1); --newNodeId;
                    }
                    else
                    {
                        // если точки по разные стороны, то находим пересечение. Новую точку добавляем в специальный массив и присваеваем ей "отрицательный" индекс.
                        if (!plane.getIntersectionPointOnLine(Node1, Node2, &interPoint)) 
                        {
                            throw std::exception("Странно, должно было сработать!\n");
                        }

                        planePoints.push_back(interPoint); --newNodeId;

                        if (Node1V == -1)
                            newVolumeFace.nodes.push_back(faceNodeI);
                        else
                            newOtherFace.nodes.push_back(faceNodeI);

                        newVolumeFace.nodes.push_back(newNodeId);
                        newOtherFace.nodes.push_back(newNodeId);

                    }
                }
            }

            resultFaces.push_back(newVolumeFace);
            resultFaces.push_back(newOtherFace);
            newVolumeFace.nodes.clear();
            newOtherFace.nodes.clear();
        }
        // БЛОК 1.1. ОПРЕДЕЛИТЬ СТОРОНУ, ПОЛУЧАЕМУЮ КАК РЕЗУЛЬТАТ ПЕРЕСЕЧЕНИЯ С ПЛОСКОСТЬЮ

        // БЛОК 2. СЧИТАЕМ ОБЪЁМ НУЖНОЙ КЛЕТКИ. ВОЗМОЖНО ПО МЕДИАНЕ!!!!

        // БЛОК 3. СРАВНЕНИЕ ОБЪЕМОВ.
        if (abs(tempVolume - volume) < eps) // проверяем, сколько объёма нашли
            break;
        else
        {
            resultCells.clear();
            resultFaces.clear();
            newFaceId = faces.size(); // обновляем номера

            if (tempVolume > volume)
                endPoint = mid;
            else
                startPoint = mid;

            mid = (CVector(startPoint) + endPoint) / 2.;
        }
    }



    return resultCells;
}
