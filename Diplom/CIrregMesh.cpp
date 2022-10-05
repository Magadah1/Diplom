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
    int newFaceId = faces.size(); // Для каждой стороны, которая будет разбита новыми ячейками нужно добавить индекс. Эти будут для ячейки 1, а также сторона, полученная с помощью плоскости.
    int newNodeId = 0; // Для каждой новой вершины добавим индекс. Они будут отрицательными, но в их массиве будут реверсироваться.

    // Объём отсекаем по противоположную сторону от нормали плоскости
    double tempVolume{};

    //
    std::vector<CIrregFace> resultFaces[2]; // Стороны новых клеток; СТРАННО 0 - клетка по другую сторону от нормали, 1 - другая.
    std::vector<CSurfaceNode> planePoints; // Новые вершины, которые могут получится при пересечении с плоскостью отсечения. С "отрицательным индексом" у ранее поделённых сторон.
    //

    while (true)
    {
        // БЛОК 1. РАЗБИВАЕМ КЛЕТКУ НА 2 С ПОМОЩЬЮ ПЛОСКОСТИ.
        plane.recalcD(mid); // теперь плоскость проходит через точку mid
        
        // Проходим по всем сторонам клетки
        for (int cellFaceI = 0; cellFaceI < cell.facesInd.size(); ++cellFaceI) 
        {
            const CIrregFace& face = faces[cell.facesInd[cellFaceI]]; // текущая сторона
            resultCells[0].facesInd.push_back(cell.facesInd[cellFaceI]); // добавляем номер стороны для нашей клетки.

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

            resultFaces[0].push_back(newVolumeFace);
            resultFaces[1].push_back(newOtherFace);
            newVolumeFace.nodes.clear();
            newOtherFace.nodes.clear();
        }
        // БЛОК 1.1. ОПРЕДЕЛИТЬ СТОРОНУ, ПОЛУЧАЕМУЮ КАК РЕЗУЛЬТАТ ПЕРЕСЕЧЕНИЯ С ПЛОСКОСТЬЮ
        CIrregFace planeFace;
        // Нужно как-то определить:
        // planeFace.cell1 = ?
        // planeFace.cell2 = ?
        // planeFace.nodes <= unique planePoints
        faces.push_back(planeFace);
        

        // БЛОК 2. СЧИТАЕМ ОБЪЁМ НУЖНОЙ КЛЕТКИ. ВОЗМОЖНО ПО МЕДИАНЕ!!!!
        CIrregCell& newCell = resultCells[0];
        newCell.facesInd.push_back(newFaceId);

        std::vector<Tetrahedron> cellTetrahedrons = spliteCellByTetrahedrons(newCell);
        for (int i = 0; i < cellTetrahedrons.size(); ++i)
            tempVolume += cellTetrahedrons[i].getVolume();
        

        // БЛОК 3. СРАВНЕНИЕ ОБЪЕМОВ.
        if (abs(tempVolume - volume) < eps) // проверяем, сколько объёма нашли
        {
            // БЛОК 4. ИЗМЕНЕНИЕ ДАННЫХ В ИСХОДНОЙ СЕТКЕ.


            break;
        }
        else
        {
            //resultCells.clear();
            resultFaces[0].clear();
            resultFaces[1].clear();
            // обновляем номера
            newFaceId = faces.size(); 
            newNodeId = 0;
            // удаляем добавленную сторону
            faces.pop_back();
            if (tempVolume > volume)
                endPoint = mid;
            else
                startPoint = mid;

            mid = (CVector(startPoint) + endPoint) / 2.;
        }
    }



    return resultCells;
}

CSurfaceNode CIrregMesh::getCellCenter(const CIrregCell& cell) const noexcept
{
    std::vector<CSurfaceNode> allNodes;
    for (int faceId = 0; faceId < cell.facesInd.size(); ++faceId)
        for (int nodeId = 0; nodeId < faces[faceId].nodes.size(); ++nodeId)
            allNodes.push_back(nodes[faces[faceId].nodes[nodeId]]);

    std::vector<CSurfaceNode> uniquiNodes;
    for (int i = 0; i < allNodes.size(); ++i)
    {
        bool was = false;
        for (int j =0 ;j < uniquiNodes.size(); ++j)
            if (CVector(allNodes[i]) == CVector(uniquiNodes[j]))
            {
                was = true;
                break;
            }
        if (!was)
            uniquiNodes.push_back(allNodes[i]);
    }

    CVector center;
    for (int i = 0; i < uniquiNodes.size(); ++i)
        center += uniquiNodes[i];

    center /= uniquiNodes.size();

    return center.toCSurfaceNode();
}

CSurfaceNode CIrregMesh::getFaceCenter(const CIrregFace& face) const noexcept
{
    CVector center;
    for (int i = 0; i < face.nodes.size(); ++i)
        center += nodes[face.nodes[i]];
    center /= face.nodes.size();

    return center.toCSurfaceNode();
}

std::vector<Tetrahedron> CIrregMesh::spliteCellByTetrahedrons(const CIrregCell& cell) const noexcept
{
    std::vector<Tetrahedron> res;

    CSurfaceNode cellCenter = getCellCenter(cell);

    for (int faceId = 0; faceId < cell.facesInd.size(); +faceId)
    {
        const CIrregFace& face = faces[faceId];

        CSurfaceNode faceCenter = getFaceCenter(face);

        for (int nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
        {
            const CSurfaceNode& node1 = nodes[face.nodes[nodeId]];
            const CSurfaceNode& node2 = nodes[face.nodes[(1ull + nodeId) % face.nodes.size()]];
            res.push_back(Tetrahedron(cellCenter, faceCenter, node1, node2));
        }
    }

    return res;
}
