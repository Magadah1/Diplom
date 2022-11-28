#include "CIrregMesh.h"

CIrregMesh::CIrregMesh() noexcept
{
}

std::pair<int, int> CIrregMesh::FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume)
{
    if (cellNumber < 0 || cellNumber >= cells.size())
        throw std::exception("Ячейки с таким номером не существует!\n");

    constexpr double eps = 1e-8; // появился в C++11, если не работает - заменить на const double
    const CIrregCell& cell = cells[cellNumber]; // получаем клетку сетки

    if (getCellVolume(cell) < volume)
        throw std::exception("Искомый объём больше объёма самой ячейки!\n");

    CPoint mid = (CVector(startPoint) + endPoint) / 2.; // Середина отрезка [startPoint, endPoint]
    
    CSideEquation plane(CVector(startPoint).createVector(endPoint)); // Строим плоскость, направленную от startPoint к endPoint

    //std::vector<CIrregCell> resultCells(2); // Новые клетки. 0 - та, чей объём ищем (по другую сторону от нормали). 1 - оставшаяся.
    const int newCellId = cells.size(); // новый номер у клетки 1. У 0 он сохранится.
    const int newFaceId = faces.size(); // Для каждой стороны, которая будет разбита новыми ячейками нужно добавить индекс. Эти будут для ячейки 1, а также сторона, полученная с помощью плоскости.
    int newNodeId = 0; // Для каждой новой вершины добавим индекс. Они будут отрицательными, но в их массиве будут реверсироваться.

    // Объём отсекаем по противоположную сторону от нормали плоскости
    double tempVolume{};

    //
    std::vector<CIrregFace> resultFaces[2]; // Стороны новых клеток; СТРАННО 0 - клетка по другую сторону от нормали, 1 - другая.
    std::vector<CSurfaceNode> planePoints; // Новые вершины, которые могут получится при пересечении с плоскостью отсечения. С "отрицательным индексом" у ранее поделённых сторон.
    std::vector<int> rightUniquiePlanePoints; // Тут будут лишь уникальные точки плоскости, причём расположенные в правильном (друг за другом) порядке.
    // 0 - индекс исходной стороны, 1 - индекс в resultFaces[0], 2 - индекс в resultFaces[1]. Если нет разбиения у стороны для новой клетки, то индекс 1 или 2 будет равен -1
    using FaceSplitting = std::tuple<int, int, int>;
    std::vector<FaceSplitting> faceSplit; // Связи старых сторон с потенкицально новыми.
    faceSplit.reserve(cell.facesInd.size());
    //
    size_t iterCRITICAL{ 1 };
    // БЛОК 1. РАЗБИЕНИЕ КЛЕТКИ НА 2
    while (true)
    {
        plane.recalcD(mid); // теперь плоскость проходит через точку mid
        tempVolume = 0.0;
        // БЛОК 1.1. РАЗБИВАЕМ КЛЕТКУ НА 2 С ПОМОЩЬЮ ПЛОСКОСТИ.
        
        // Проходим по всем сторонам клетки
        for (int cellFaceI = 0; cellFaceI < cell.facesInd.size(); ++cellFaceI) 
        {
            const int& faceID = cell.facesInd[cellFaceI]; // индекс текущей стороны
            const CIrregFace& face = faces[faceID]; // сама текущая сторона


            // ВОПРОС!!!
            //resultCells[0].facesInd.push_back(cell.facesInd[cellFaceI]); // добавляем номер стороны для нашей клетки. ??????????

            // Потенциально разбиваем сторону клетки на 2. Первая сохранит свой номер в дальнейшем.
            CIrregFace newVolumeFace, newOtherFace;
            if (face.cell1 == cellNumber)
            {
                newVolumeFace.cell1 = cellNumber;
                newVolumeFace.cell2 = face.cell2;

                newOtherFace.cell1 = newCellId;
                newOtherFace.cell2 = face.cell2;
            }
            else
            {
                newVolumeFace.cell2 = cellNumber;
                newVolumeFace.cell1 = face.cell1;

                newOtherFace.cell2 = newCellId;
                newOtherFace.cell1 = face.cell1;
            }

            // Будем собирать точки пересечения.
            CSurfaceNode interPoint;
            bool duplicatePoint = false;    // Полученные точки пересечения будем брать лишь единожды.
            int duplicateId{};              // Если есть дубликат, то запишем его индекс
            for (int faceNodeI = 0; faceNodeI < face.nodes.size(); ++faceNodeI)
            {
                const int Node1ID = face.cell1 == cellNumber ? face.nodes[faceNodeI] : face.nodes[(faceNodeI + 1ull) % face.nodes.size()];  // Индекс первой точки в глобальном массиве.
                const int Node2ID = face.cell1 == cellNumber ? face.nodes[(faceNodeI + 1ull) % face.nodes.size()] : face.nodes[faceNodeI];  // Индекс второй точки в глобальном массиве.
                const CSurfaceNode& Node1 = nodes[Node1ID];    // первая точка для отрезка.
                const CSurfaceNode& Node2 = nodes[Node2ID];    // вторая точка для отрезка. Типа ребро.

                const int Node1V = plane.getNormedValue(Node1);
                const int Node2V = plane.getNormedValue(Node2);

                if (Node1V == Node2V && Node1V) // если вершины по одну сторону от плоскости, то забиваем на пересечение
                {
                    if (Node1V == -1)
                        newVolumeFace.nodes.push_back(Node1ID);
                    else
                        newOtherFace.nodes.push_back(Node1ID);
                }
                else
                {
                    if (Node1V == Node2V && !Node1V) // обе точки на плоскости, добавляем первую из них в каждую из новых клеток и добавляем к новой стороне, что образуется плоскостью.
                    {
                        duplicatePoint = false;
                        for (duplicateId = 0; duplicateId < planePoints.size() && !duplicatePoint; ++duplicateId)
                            if (planePoints[duplicateId] == Node1)
                                duplicatePoint = true;

                        if (!duplicatePoint)
                        {
                            planePoints.push_back(Node1); --newNodeId;
                            newVolumeFace.nodes.push_back(newNodeId);
                            newOtherFace.nodes.push_back(newNodeId);
                        }
                        else
                        {
                            newVolumeFace.nodes.push_back(-duplicateId);
                            newOtherFace.nodes.push_back(-duplicateId);
                        }
                    }
                    else if (!Node1V || !Node2V) // одна точка на плоскости. Другая по какую-то сторону
                    {
                        switch (Node1V)
                        {
                        case 1:
                            newOtherFace.nodes.push_back(Node1ID);
                            break;
                        case 0:

                            duplicatePoint = false;
                            for (duplicateId = 0; duplicateId < planePoints.size() && !duplicatePoint; ++duplicateId)
                                if (planePoints[duplicateId] == Node1)
                                    duplicatePoint = true;

                            if (!duplicatePoint)
                            {
                                planePoints.push_back(Node1); --newNodeId;
                                newVolumeFace.nodes.push_back(newNodeId);
                                newOtherFace.nodes.push_back(newNodeId);
                            }
                            else
                            {
                                newVolumeFace.nodes.push_back(-duplicateId);
                                newOtherFace.nodes.push_back(-duplicateId);
                            }
                            break;
                        case -1:
                            newVolumeFace.nodes.push_back(Node1ID);
                            break;
                        }
                    }
                    else
                    {
                        // если точки по разные стороны, то находим пересечение. Новую точку добавляем в специальный массив и присваеваем ей "отрицательный" индекс.
                        if (!plane.getIntersectionPointOnLine(Node1, Node2, &interPoint)) 
                        {
                            throw std::exception("Странно, должно было сработать!\n");
                        }

                        //planePoints.push_back(interPoint); --newNodeId;

                        if (Node1V == -1)
                            newVolumeFace.nodes.push_back(Node1ID);
                        else
                            newOtherFace.nodes.push_back(Node1ID);


                        /*newVolumeFace.nodes.push_back(newNodeId);
                        newOtherFace.nodes.push_back(newNodeId);*/

                        duplicatePoint = false;
                        for (duplicateId = 0; duplicateId < planePoints.size() && !duplicatePoint; ++duplicateId)
                            if (planePoints[duplicateId] == interPoint)
                                duplicatePoint = true;

                        if (!duplicatePoint)
                        {
                            planePoints.push_back(interPoint); --newNodeId;
                            newVolumeFace.nodes.push_back(newNodeId);
                            newOtherFace.nodes.push_back(newNodeId);
                        }
                        else
                        {
                            newVolumeFace.nodes.push_back(-duplicateId);
                            newOtherFace.nodes.push_back(-duplicateId);
                        }
                    }
                }
            }

            FaceSplitting split((int)faceID, (int)resultFaces[0].size(), (int)resultFaces[1].size());
            if (/*!newVolumeFace.nodes.size() || */newVolumeFace.nodes.size() < 3)
                std::get<1>(split) = -1;
            if (/*!newOtherFace.nodes.size() ||*/ newOtherFace.nodes.size() < 3)
                std::get<2>(split) = -1;
            faceSplit.push_back(split);

            if (std::get<1>(split) != -1)
                resultFaces[0].push_back(newVolumeFace);
            if (std::get<2>(split) != -1)
                resultFaces[1].push_back(newOtherFace);
            newVolumeFace.nodes.clear();
            newOtherFace.nodes.clear();
        }
        // БЛОК 1.2. ОПРЕДЕЛИТЬ СТОРОНУ, ПОЛУЧАЕМУЮ КАК РЕЗУЛЬТАТ ПЕРЕСЕЧЕНИЯ С ПЛОСКОСТЬЮ
        CIrregFace planeFace;
        if (planePoints.size()) // если нет точек пересечения, то и не будет новой грани.
        {
            const int initialeNodeID = -1; // Изначальная вершина
            rightUniquiePlanePoints.reserve(planePoints.size());
            rightUniquiePlanePoints.push_back(initialeNodeID);

            size_t startFaceId{};
            for (startFaceId = 0; startFaceId < resultFaces[0].size(); ++startFaceId)
            {
                bool was = false;
                const CIrregFace& face = resultFaces[0][startFaceId];
                for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
                    if (face.nodes[nodeId] == initialeNodeID)
                    {
                        was = true;
                        break;
                    }

                if (was)
                    break;
            } // на этом этапе найдена изначальная вершина и какая-то содержащая её грань.

            int startNodeID{}; // Это соседняя с изначальной вершиной вершина.

            for (size_t nodeId = 0; nodeId < resultFaces[0][startFaceId].nodes.size() && !startNodeID; ++nodeId)
                if (resultFaces[0][startFaceId].nodes[nodeId] < 0 && resultFaces[0][startFaceId].nodes[nodeId] != initialeNodeID)
                    startNodeID = resultFaces[0][startFaceId].nodes[nodeId]; // на этом этапе найдена соседняя вершина к изначальной из той же грани.


            int nextNodeId{}; // Будем искать к ней соседей из другой грани до тех пор, пока не достигнем изначальной вершины
            do
            {
                rightUniquiePlanePoints.push_back(startNodeID); // записываем вершину, которая как бы является началом ребра поиска соседней
                
                if (rightUniquiePlanePoints.size() > planePoints.size())
                    throw std::exception("Скорее всего было больше 2х пересечений в пределах одной грани!\n");

                for (size_t faceID = 0; faceID < resultFaces[0].size(); ++faceID)
                {
                    bool was = false;
                    const CIrregFace& face = resultFaces[0][faceID];

                    for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
                    {
                        if (face.nodes[nodeId] == startNodeID && faceID != startFaceId 
                            && std::count_if(face.nodes.begin(), face.nodes.end(), std::bind(std::less<int>(), std::placeholders::_1, 0)) > 1)
                        {
                            startFaceId = faceID;
                            was = true;
                            break;
                        }
                    }

                    if (was)
                        break;
                } // на этом этапе найдена грань, содержащая соседнюю точку, отличная от текущей.

                nextNodeId = 0; // тут надо прикол
                const CIrregFace& face = resultFaces[0][startFaceId];
                for (size_t nodeId = 0; nodeId < face.nodes.size() && !nextNodeId; ++nodeId)
                    if (face.nodes[nodeId] < 0 && face.nodes[nodeId] != startNodeID)
                        nextNodeId = face.nodes[nodeId]; // на этом этапе найдена следующая соседняя точка.
                
                startNodeID = nextNodeId; // чтобы идти дальше

            } while (nextNodeId != initialeNodeID);
        }
        planeFace.nodes = std::move(rightUniquiePlanePoints);

        std::vector<int> volumeNodes; // Вершины той части клетки, объём которой ищем.
        for (size_t faceId = 0; faceId < resultFaces[0].size(); ++faceId)
        {
            const CIrregFace& face = resultFaces[0][faceId];
            for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
                volumeNodes.push_back(face.nodes[nodeId]);
        }
        for (size_t nodeId = 0; nodeId < planeFace.nodes.size(); ++nodeId)
            volumeNodes.push_back(planeFace.nodes[nodeId]);

        std::sort(volumeNodes.begin(), volumeNodes.end());
        volumeNodes.erase(std::unique(volumeNodes.begin(), volumeNodes.end()), volumeNodes.end()); // оставляем только уникальные вершины

        CVector volumeCenter;
        for (size_t i = 0; i < volumeNodes.size(); ++i)
        {
            const int& nodeID = volumeNodes[i];

            if (nodeID < 0)
                volumeCenter += planePoints[-nodeID - 1];
            else
                volumeCenter += nodes[nodeID];
        }
        volumeCenter /= volumeNodes.size(); // найден центр той части ячейки, объём которой ищем.
        

        // Сначала посчитаем объём нужной части клетки, а потом завершим топологию грани, образованной плоскостью и всего остального

        // БЛОК 2. СЧИТАЕМ ОБЪЁМ НУЖНОЙ КЛЕТКИ.
        for (size_t faceId = 0; faceId < resultFaces[0].size(); ++faceId)
        {
            const CIrregFace& face = resultFaces[0][faceId];

            CVector faceCenter;

            for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
            {
                const int& id = face.nodes[nodeId];

                if (id < 0)
                    faceCenter += planePoints[-id - 1];
                else
                    faceCenter += nodes[id];
            }
            faceCenter /= face.nodes.size();

            for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
            {
                const int& id1 = face.nodes[nodeId];
                const int& id2 = face.nodes[(1ull + nodeId) % face.nodes.size()];

                const CSurfaceNode& Node1 = id1 < 0 ? planePoints[-id1 - 1] : nodes[id1];
                const CSurfaceNode& Node2 = id2 < 0 ? planePoints[-id2 - 1] : nodes[id2];

                tempVolume += Tetrahedron(Node1.likeCPoint(), Node2.likeCPoint(), faceCenter.likeCPoint(), volumeCenter.likeCPoint()).getVolume();
            }
        } // Сначала посчитали по известным граням

        if (planePoints.size()) // Теперь с учетом ещё одной
        {
            CVector planeCenter; // Вычислим центр грани, полученной плоскостью
            for (size_t nodeId = 0; nodeId < planeFace.nodes.size(); ++nodeId)
                planeCenter += planePoints[-planeFace.nodes[nodeId] - 1];
            planeCenter /= planeFace.nodes.size();

            for (size_t nodeId = 0; nodeId < planeFace.nodes.size(); ++nodeId)
            {
                const int& id1 = planeFace.nodes[nodeId];
                const int& id2 = planeFace.nodes[(1ull + nodeId) % planeFace.nodes.size()];

                const CSurfaceNode& Node1 = planePoints[-id1 - 1];
                const CSurfaceNode& Node2 = planePoints[-id2 - 1];

                tempVolume += Tetrahedron(Node1.likeCPoint(), Node2.likeCPoint(), planeCenter.likeCPoint(), volumeCenter.likeCPoint()).getVolume();
            }
        }

        // БЛОК 3. СРАВНЕНИЕ ОБЪЕМОВ.
        if (abs(tempVolume - volume) < eps) // проверяем, сколько объёма нашли
        {
            // БЛОК 3.1. ИЗМЕНЕНИЕ ДАННЫХ В ИСХОДНОЙ СЕТКЕ.
            const CVector N0 = planePoints[-planeFace.nodes[0] - 1];
            const CVector N1 = planePoints[-planeFace.nodes[1] - 1];
            const CVector N2 = planePoints[-planeFace.nodes[2] - 1];

            if (N1.createVector(N0).Mixed(N1.createVector(volumeCenter), N1.createVector(N2)) > 0.0)
            {
                planeFace.cell1 = cellNumber;
                planeFace.cell2 = newCellId;
            }
            else
            {
                planeFace.cell2 = cellNumber;
                planeFace.cell1 = newCellId;
            } // На этом этапе определена топология новой грани. Осталось обновить ВСЕ связи с изначальных и новых ячейках.

            const int positiveNodeID = nodes.size();
            nodes.insert(nodes.end(), planePoints.begin(), planePoints.end());

            // Поэтапно заменим все отрицательные индексы у узлов на новые индексы из общего массива

            for (size_t i = 0; i < planeFace.nodes.size(); ++i)
            {
                int& id = planeFace.nodes[i]; // они все отрицательные <= -1 : (-1, -2, -3 ...)
                
                if (id >= 0)
                    throw std::exception("Был положительный индекс. Глупость.\n");

                id = positiveNodeID - id - 1;
            }
            for (size_t i = 0; i < 2; ++i)
            {
                std::vector<CIrregFace>& nFaces = resultFaces[i];
                for (size_t j = 0; j < nFaces.size(); ++j)
                {
                    CIrregFace& nFace = nFaces[j];
                    for (size_t k = 0; k < nFace.nodes.size(); ++k)
                    {
                        int& id = nFace.nodes[k];

                        if (id < 0)
                            id = positiveNodeID - id - 1;
                    }
                }
            } // Все отрицательные индексы заменены на новые.

            faces.push_back(planeFace);
            cells.push_back(CIrregCell());
            CIrregCell& newCell = cells.back();
            newCell.facesInd.push_back(newFaceId);
            cells[cellNumber].facesInd.push_back(newFaceId);
            // Теперь нужно обновить текущие плоскости и добавить новые.
            
            for (size_t i = 0; i < faceSplit.size(); ++i)
            {
                const int& initialFaceId    = std::get<0>(faceSplit[i]);
                const int& changeFaceId     = std::get<1>(faceSplit[i]);
                const int& newFaceId        = std::get<2>(faceSplit[i]);


                CIrregFace& initialFace = faces[initialFaceId];
                CIrregCell& initialCell = cells[cellNumber];

                if (changeFaceId == -1 && newFaceId != -1)
                {
                    newCell.facesInd.push_back(initialFaceId); // новая клетка имеет ту грань нетронутой, кроме связи.

                    initialFace.cell1 == cellNumber ? initialFace.cell1 = newCellId : initialFace.cell2 = newCellId; // нетронутая грань теперь вместо изначальной клетки ссылается на новую.

                    *std::find(initialCell.facesInd.begin(), initialCell.facesInd.end(), initialFaceId) = -1; // исходная клетка больше не имеет доступа к данной грани. (пока имеет, потом уберём)
                }
                else if (changeFaceId != -1 && newFaceId != -1) // лишь в этом случае исходная грань будет разрезана
                {
                    initialFace = resultFaces[0][changeFaceId]; // заменяем саму исходную грань на новую. Связь от соседней клетки сохранена, но нужно ещё.

                    const int nID = faces.size();
                    faces.push_back(resultFaces[1][newFaceId]); // добавили оставшуюся часть грани

                    const int& cell1ID = faces[nID].cell1;
                    const int& cell2ID = faces[nID].cell2;

                    const int& adjacentCellId = cell1ID == newCellId ? cell2ID : cell1ID; // определили соседнюю клетку, пока ещё не имеющую связь с новой

                    if (adjacentCellId != -1)   // если исходная клетка была не граничной, то добавляем новую связть к соседней клетке.
                        cells[adjacentCellId].facesInd.push_back(nID);

                    newCell.facesInd.push_back(nID); // и сама новая клетка теперь знает о новой грани
                }
                else if (changeFaceId != -1 && newFaceId == -1) // ничего не меняем, связи все есть.
                {
                    
                }
                else
                    throw std::exception("Получилось так, что узлы оказались ни по одну из сторон от плоскости. Глупость!\n"); // не должен выбрасываться никогда!

                // удаляем из исходной ячейки связи с теми гранями, к которым больше нет доступа.
                std::vector<int>& ininitalCellFaces = initialCell.facesInd;
                ininitalCellFaces.erase(std::remove(ininitalCellFaces.begin(), ininitalCellFaces.end(), -1), ininitalCellFaces.end()); 
            }

            return { newCellId, iterCRITICAL};
        }
        else
        {
            // БЛОК 3.2. ПОВТОРНЫЙ РАСЧЁТ УРАВНЕНИЯ ПЛОСКОСТИ И Т.П.
            //resultCells.clear(); resultCells.resize(2);
            resultFaces[0].clear();
            resultFaces[1].clear();
            planePoints.clear();
            rightUniquiePlanePoints.clear();
            faceSplit.clear();
            // обновляем номера
            //newFaceId = faces.size(); 
            newNodeId = 0;
            // удаляем добавленную сторону
            // faces.pop_back();
            if (tempVolume > volume)
                endPoint = mid;
            else
                startPoint = mid;

            mid = (CVector(startPoint) + endPoint) / 2.;
            ++iterCRITICAL;
            if (iterCRITICAL >= 1'000'000)
                throw std::exception("Было более миллиона итераций! Возможно границы постоения плоскости заданы не верно или между ними нет возможности найти точку, дающую нужный объём отсечения!\n");

            if ((CVector(startPoint) - endPoint).Length() < eps)
                throw std::exception("Границы построения плоскости слишком сблизились!\n");
        }
    }

}

void CIrregMesh::spliteFaceByTriangles(const int& faceNumber)
{
    if (faceNumber < 0 || faceNumber >= faces.size())
        throw std::exception("Грани с таким номером не существует!\n");

    CIrregFace& face = faces[faceNumber];
    size_t faceNodesCount = face.nodes.size();
    CIrregCell* faceCells[2]{};
    faceCells[0] = face.cell1 != -1 ? &cells[face.cell1] : nullptr;
    faceCells[1] = face.cell2 != -1 ? &cells[face.cell2] : nullptr;

    CSurfaceNode center = getFaceCenter(face);
    int NDI = nodes.size();
    int NFI = faces.size();

    CIrregFace tempFace;
    tempFace.cell1 = face.cell1;
    tempFace.cell2 = face.cell2;
    std::vector<CIrregFace> newFaces(faceNodesCount);

    nodes.push_back(center);
    for (size_t i = 0; i < faceNodesCount; ++i)
    {
        const int& nId1 = face.nodes[i];
        const int& nId2 = face.nodes[(i + 1ull) % faceNodesCount];

        tempFace.nodes = { nId1, nId2, NDI };

        newFaces[i] = tempFace;
    }

    for (size_t i = 0; i < faceNodesCount; ++i)
        if (!i)
            faces[faceNumber] = newFaces[i];
        else
            faces.push_back(newFaces[i]);

    for (size_t i = 0; i < 2; ++i)
    {
        if (!faceCells[i])
            continue;

        CIrregCell& cell = *faceCells[i];

        for (size_t nfID = 0; nfID < faceNodesCount - 1; ++nfID)
            cell.facesInd.push_back(NFI + nfID);
    }
}

double CIrregMesh::getCellVolume(const CIrregCell& cell) const noexcept
{
    std::vector<Tetrahedron> tets = spliteCellByTetrahedrons(cell);
    
    double v{};

    for (size_t i = 0; i < tets.size(); ++i)
        v += tets[i].getVolume();

    return v;
}

CSurfaceNode CIrregMesh::getCellCenter(const CIrregCell& cell) const noexcept
{
    std::vector<int> allNodes;
    for (int faceId = 0; faceId < cell.facesInd.size(); ++faceId)
    {
        const CIrregFace& face = faces[cell.facesInd[faceId]];
        for (int nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
            allNodes.push_back(face.nodes[nodeId]);
    }

    std::sort(allNodes.begin(), allNodes.end());
    allNodes.erase(std::unique(allNodes.begin(), allNodes.end()), allNodes.end());


    CVector center;
    for (int i = 0; i < allNodes.size(); ++i)
        center += nodes[allNodes[i]];

    center /= allNodes.size();

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

    for (int faceId = 0; faceId < cell.facesInd.size(); ++faceId)
    {
        const CIrregFace& face = faces[cell.facesInd[faceId]];

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
