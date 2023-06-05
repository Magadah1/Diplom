#include "CIrregMesh.h"
extern const double CalculationEps;

CIrregMesh::CIrregMesh() noexcept
{

#ifdef NEWTON
    mode = Mode::OLD;
#endif // NEWTON

}

std::pair<int, int> CIrregMesh::FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume)
{
    if (cellNumber < 0 || cellNumber >= cells.size())
        throw std::exception("Ячейки с таким номером не существует!\n");

    //constexpr double eps = 1e-16; //-16 максимум
    const CIrregCell& cell = cells[cellNumber]; // получаем клетку сетки
    const double cellV = getCellVolume(cell);

    double volumeAtStart = getCutOffCellVolume(cellNumber, CSideEquation(CVector(startPoint).createVector(endPoint), startPoint));
    double volumeAtEnd = getCutOffCellVolume(cellNumber, CSideEquation(CVector(startPoint).createVector(endPoint), endPoint));

    if (volume < volumeAtStart || volume > volumeAtEnd)
        throw std::exception("Искомый объём не может быть найден в заданных границах поиска!\n");
    else if (cellV - CalculationEps < volume)
        throw std::exception("Искомый объём больше объёма самой ячейки!\n");
    else if (volume < CalculationEps)
        throw std::exception("Смысл отрезать 0ой объём?\n");

    CPoint* hope = nullptr; // если повезло, то объём прямо на одной из границ диапазона
    if (abs(volume - volumeAtStart) < CalculationEps)
        hope = &startPoint;
    else if (abs(volume - volumeAtEnd) < CalculationEps)
        hope = &endPoint;

    CPoint mid, oldMid, oldTempMid;
    double k0 = volume / cellV;

    if (!hope)
    {
#ifndef NEWTON
        // Середина отрезка [startPoint, endPoint]
        mid = (CVector(startPoint) + endPoint) / 2.;
#else
        if (mode == Mode::OLD)
            mid = (CVector(startPoint) + endPoint) / 2.;
        else
            oldMid = mid = CVector(startPoint) + CVector(startPoint).createVector(endPoint).Orth() * CVector(startPoint).createVector(endPoint).Length() * k0;

#endif // !NEWTON

    } 
    else
    {
        oldMid = mid = *hope;
    }
    
    CSideEquation plane(CVector(startPoint).createVector(endPoint)); // Строим плоскость, направленную от startPoint к endPoint

    const int newCellId = cells.size(); // новый номер у клетки 1. У 0 он сохранится.
    const int newFaceId = faces.size(); // Для каждой стороны, которая будет разбита новыми ячейками нужно добавить индекс. Эти будут для ячейки 1, а также сторона, полученная с помощью плоскости.
    int newNodeId = 0; // Для каждой новой вершины добавим индекс. Они будут отрицательными, но в их массиве будут реверсироваться.

    // Объём отсекаем по противоположную сторону от нормали плоскости
    double tempVolume{}, oldTempVolume{};

    // Стороны новых ячеек; 0 - ячейка по другую сторону от нормали, 1 - другая.
    std::vector<CIrregFace> resultFaces[2];

    // Новые узлы, которые могут получится при пересечении с плоскостью отсечения. 
    // С "отрицательным индексом" у ранее поделённых сторон.
    std::vector<CSurfaceNode> planePoints;

    // Тут будут лишь уникальные точки контактной плоскости, причём расположенные в правильном (друг за другом) порядке.
    std::vector<int> rightUniquiePlanePoints;

    // Если для отрицательного индекса существует узел в исходной сетке, то потом индекс будет ссылаться на него. 
    // first - отрицательный индекс, second - исходный.
    std::vector<std::pair<int, int>> existingPointsIDs; 

    // 0 - индекс исходной стороны, 1 - индекс в resultFaces[0], 2 - индекс в resultFaces[1]. 
    // Если нет разбиения у стороны для новой ячейки, то индекс 1 или 2 будет равен -1; 
    // Связи старых сторон с потенциально новыми.
    std::vector<INT3> faceSplit; 
    faceSplit.reserve(cell.facesInd.size());

    // 0 - номер исходной грани, 1 - один из узлов ребра, 2 - оставшийся узел ребра.
    // 3 - индекс нового узла, находящегося между этими двумя; 
    // Изначально индексы новых узлов отрицательные.
    std::vector<INT4> edgePoints;

    // Контактная плоскость
    CIrregFace planeFace;

    // Геометрический центр отсекаемой части ячейки
    CVector volumeCenter;

    size_t iterCRITICAL{ 1 };
    const size_t maxIterCRITICAL{ 9999 };

    // БЛОК 1. РАЗБИЕНИЕ КЛЕТКИ НА 2
    while (true)
    {
        plane.recalcD(mid); // теперь плоскость проходит через точку mid
        planeFace = CIrregFace();
        volumeCenter = CVector();
        // БЛОК 1.1. РАЗБИВАЕМ КЛЕТКУ НА 2 С ПОМОЩЬЮ ПЛОСКОСТИ.
        tempVolume = getCutOffCellVolume(cellNumber, plane,
            resultFaces,
            &planePoints,
            &planeFace,
            &volumeCenter,
            &existingPointsIDs,
            &faceSplit,
            &edgePoints
        );
        // код переехал в функцию getCutOffCellVolume

        // БЛОК 3. СРАВНЕНИЕ ОБЪЕМОВ.
        if (abs(tempVolume - volume) < CalculationEps ||  iterCRITICAL == maxIterCRITICAL && abs(tempVolume - volume) < CalculationEps * 1e1) // проверяем, сколько объёма нашли
        {
            if (planeFace.nodes.size() < 3)
                return { cellNumber, iterCRITICAL };

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
            } // На этом этапе определена топология новой грани. Осталось обновить ВСЕ связи в изначальных и новых ячейках.

            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! NEW !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            const int positiveNodeID = nodes.size();
            for (int i = 0; i < planePoints.size(); ++i)
            {
                const int negI = -i - 1;

                bool was = false;

                for (size_t j = 0; j < existingPointsIDs.size() && !was; ++j)
                    if (negI == existingPointsIDs[j].first)
                        was = true;

                if (!was)
                    nodes.push_back(planePoints[i]);
            }

            //const int          = nodes.size(); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            //nodes.insert(nodes.end(), planePoints.begin(), planePoints.end()); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            // Поэтапно заменим все отрицательные индексы у узлов на новые индексы из общего массива

            for (size_t i = 0; i < planeFace.nodes.size(); ++i)
            {
                int& id = planeFace.nodes[i]; // они все отрицательные <= -1 : (-1, -2, -3 ...)

                if (id >= 0)
                    throw std::exception("Был положительный индекс. Глупость.\n"); // уже не глупость....

                bool was = false;

                int countExistingPointsBelowThis{};

                for (size_t j = 0; j < existingPointsIDs.size() && !was; ++j)
                    if (id == existingPointsIDs[j].first)
                    {
                        id = existingPointsIDs[j].second;
                        was = true;
                    }
                    else if (existingPointsIDs[j].first > id)
                        ++countExistingPointsBelowThis;

                if (!was)
                    id = positiveNodeID - 1 - (countExistingPointsBelowThis + id);
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
                        {
                            bool was = false;

                            int countExistingPointsBelowThis{};

                            for (size_t j = 0; j < existingPointsIDs.size() && !was; ++j)
                                if (id == existingPointsIDs[j].first)
                                {
                                    id = existingPointsIDs[j].second;
                                    was = true;
                                }
                                else if (existingPointsIDs[j].first > id)
                                    ++countExistingPointsBelowThis;

                            if (!was)
                                id = positiveNodeID - 1 - (countExistingPointsBelowThis + id); 
                        }
                    }
                }
            } 
            for (size_t i = 0; i < edgePoints.size(); ++i) 
            {
                int& newEdgePointId = std::get<3>(edgePoints[i]);

                int countExistingPointsBelowThis{};

                for (size_t j = 0; j < existingPointsIDs.size(); ++j)
                    if (existingPointsIDs[j].first > newEdgePointId)
                        ++countExistingPointsBelowThis;
                    else
                        break;

                newEdgePointId = positiveNodeID - 1 - (countExistingPointsBelowThis + newEdgePointId);
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

                    (initialFace.cell1 == cellNumber) ? initialFace.cell1 = newCellId : initialFace.cell2 = newCellId; // нетронутая грань теперь вместо изначальной клетки ссылается на новую.

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
                } // остался случай if (changeFaceId != -1 && newFaceId == -1) - тут всё хорошо, ничего не делаем
                
                // удаляем из исходной ячейки связи с теми гранями, к которым больше нет доступа.
                std::vector<int>& ininitalCellFaces = initialCell.facesInd;
                ininitalCellFaces.erase(std::remove(ininitalCellFaces.begin(), ininitalCellFaces.end(), -1), ininitalCellFaces.end());
            }
            for (size_t i = 0; i < edgePoints.size(); ++i)
            {
                const INT4& edgePoint = edgePoints[i];
                const int& faceId = std::get<0>(edgePoint);
                const CIrregFace& face = faces[faceId];
                const int otherCell = face.cell1 == cellNumber ? face.cell2 : face.cell1;
                if (otherCell == -1)
                    continue;

                const int& edgePoint1ID = std::get<1>(edgePoint);
                const int& edgePoint2ID = std::get<2>(edgePoint);
                const int& newEdgePoint = std::get<3>(edgePoint);

                setEdgePoint(otherCell, edgePoint1ID, edgePoint2ID, newEdgePoint);
            } // ставим точки на рёбра

            return { newCellId, iterCRITICAL};
        }
        else
        {
            // БЛОК 3.2. ПОВТОРНЫЙ РАСЧЁТ УРАВНЕНИЯ ПЛОСКОСТИ И Т.П.
            ++iterCRITICAL;
            if (iterCRITICAL > maxIterCRITICAL+1)
                throw CException("Было более десяти тысяч итераций! Проверьте границы поиска!", { {"Искомый", volume}, {"Найденный", tempVolume}, {"Разница", abs(volume - tempVolume)} });
#ifndef NEWTON
            if ((CVector(startPoint) - endPoint).Length() < CalculationEps / 10)
                throw CException("Границы построения плоскости слишком сблизились!", { {"Искомый", volume}, {"Найденный", tempVolume}, {"Разница", abs(volume - tempVolume)} });
#else
            if (mode == Mode::OLD)
            {
                if ((CVector(startPoint) - endPoint).Length() < CalculationEps / 1e4)
                    throw CException("Границы построения плоскости слишком сблизились!", { {"Искомый", volume}, {"Найденный", tempVolume}, {"Разница", abs(volume - tempVolume)} });
            }
            else
            {
                if (iterCRITICAL > 2 && CVector(mid).createVector(oldMid).Length() < CalculationEps / 100)
                    throw CException("Границы построения плоскости слишком сблизились!", { {"Искомый", volume}, {"Найденный", tempVolume}, {"Разница", abs(volume - tempVolume)} });
            }
#endif // !NEWTON
            resultFaces[0].clear();
            resultFaces[1].clear();
            planePoints.clear();
            rightUniquiePlanePoints.clear();
            existingPointsIDs.clear();
            faceSplit.clear();
            edgePoints.clear();
            newNodeId = 0;

#ifndef NEWTON
            if (tempVolume > volume)
                endPoint = mid;
            else
                startPoint = mid;

            mid = (CVector(startPoint) + endPoint) / 2.;
#else
            if (mode == Mode::OLD)
            {
                if (tempVolume > volume)
                    endPoint = mid;
                else
                    startPoint = mid;

                mid = (CVector(startPoint) + endPoint) / 2.;
            }
            else if (mode == Mode::NEW || iterCRITICAL == 2)
            {
                double k = volume / tempVolume;
                CVector currentVector = CVector(startPoint).createVector(mid);
                CVector orthCurrentVector = currentVector.Orth();
                double currentVectorLength = currentVector.Length();
                if (mode == Mode::NEW)
                    oldMid = mid;
                else
                    oldTempMid = mid;////////////

                oldTempVolume = tempVolume;//////



                /*if (mode == Mode::NEW)
                    mid = startPoint + orthCurrentVector * currentVectorLength * k;
                else*/
                    mid = startPoint + orthCurrentVector * currentVectorLength * k0;
            }
            else // if (mode == NEWMODIFIED)
            {
                //oldTempMid = oldMid;
                // k     k+1
                oldMid = mid;

                mid = CVector(oldMid) - (CVector(oldMid) - CVector(oldTempMid)) / (tempVolume - oldTempVolume) * (tempVolume - volume);
                oldTempVolume = tempVolume;
            }
#endif // !NEWTON
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

double CIrregMesh::getCutOffCellVolume(const int& cellNumber, const CSideEquation& plane, 
    std::vector<CIrregFace>* const OUTresultFaces, 
    std::vector<CSurfaceNode>* const OUTplanePoints, 
    CIrregFace* const OUTplaneFace, 
    CVector* const OUTvolumeCenter, 
    std::vector<std::pair<int, int>>* const OUTexistingPointsIDs, 
    std::vector<INT3>* const OUTfaceSplit, 
    std::vector<INT4>* const OUTedgePoints)
{
    const CIrregCell& cell = cells[cellNumber]; // получаем клетку сетки

    const int newCellId = cells.size(); // новый номер у клетки 1. У 0 он сохранится.
    const int newFaceId = faces.size(); // Для каждой стороны, которая будет разбита новыми ячейками нужно добавить индекс. Эти будут для ячейки 1, а также сторона, полученная с помощью плоскости.
    int newNodeId = 0; // Для каждой новой вершины добавим индекс. Они будут отрицательными, но в их массиве будут реверсироваться.

    std::vector<CIrregFace> resultFaces[2]; // Стороны новых клеток; СТРАННО 0 - клетка по другую сторону от нормали, 1 - другая.
    std::vector<CSurfaceNode> planePoints; // Новые вершины, которые могут получится при пересечении с плоскостью отсечения. С "отрицательным индексом" у ранее поделённых сторон.
    std::vector<int> rightUniquiePlanePoints; // Тут будут лишь уникальные точки плоскости, причём расположенные в правильном (друг за другом) порядке.
    std::vector<std::pair<int, int>> existingPointsIDs; // Если для отрицательного индекса существует вершина в исходной сетке, то потом индекс будет ссылаться на неё. first - отрицательный индекс, second - исходный.

    // 0 - индекс исходной стороны, 1 - индекс в resultFaces[0], 2 - индекс в resultFaces[1]. Если нет разбиения у стороны для новой клетки, то индекс 1 или 2 будет равен -1; Связи старых сторон с потенциально новыми.
    std::vector<INT3> faceSplit;
    faceSplit.reserve(cell.facesInd.size());

    // 0 - номер исходной грани, 1 - одна из точек ребра, 2 - оставшаяся точка ребра. [{(Они опорядочены по возрастанию индексов.)}] 3 - индекс новой точки, находящейся между этих двух; Изначально индексы новых точек отрицательные.
    std::vector<INT4> edgePoints;

    double volume{};

    // Проходим по всем сторонам клетки
    for (size_t cellFaceI = 0; cellFaceI < cell.facesInd.size(); ++cellFaceI)
    {
        const int& faceID = cell.facesInd[cellFaceI]; // индекс текущей стороны
        const CIrregFace& face = faces[faceID]; // сама текущая сторона

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

        for (size_t faceNodeI = 0; faceNodeI != face.nodes.size(); ++faceNodeI)
        {
            // Индекс первой точки в глобальном массиве.
            const int Node1ID = face.cell1 == cellNumber ?
                face.nodes[faceNodeI] : face.nodes[(faceNodeI + 1ull) % face.nodes.size()];
            // Индекс второй точки в глобальном массиве.
            const int Node2ID = face.cell1 == cellNumber
                ? face.nodes[(faceNodeI + 1ull) % face.nodes.size()] : face.nodes[faceNodeI];
            const CSurfaceNode& Node1 = nodes[Node1ID];    // первая точка для отрезка.
            const CSurfaceNode& Node2 = nodes[Node2ID];    // вторая точка для отрезка. Node1 -> Node2 = ребро.

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
                        existingPointsIDs.push_back(std::pair<int, int>(newNodeId, (int)Node1ID));
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
                            existingPointsIDs.push_back(std::pair<int, int>(newNodeId, (int)Node1ID));
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

                    if (Node1V == -1)
                        newVolumeFace.nodes.push_back(Node1ID);
                    else
                        newOtherFace.nodes.push_back(Node1ID);

                    duplicatePoint = false;
                    for (duplicateId = 0; duplicateId < planePoints.size() && !duplicatePoint; ++duplicateId)
                        if (planePoints[duplicateId] == interPoint)
                            duplicatePoint = true;

                    INT4 edgePoint;
                    if (!duplicatePoint)
                    {
                        planePoints.push_back(interPoint); --newNodeId;
                        newVolumeFace.nodes.push_back(newNodeId);
                        newOtherFace.nodes.push_back(newNodeId);

                        //INT4 edgePoint((int)faceID, (int)(Node1ID < Node2ID ? Node1ID : Node2ID), (int)(Node1ID < Node2ID ? Node2ID : Node1ID), (int)newNodeId);
                        edgePoint = INT4((int)faceID, (int)Node1ID, (int)Node2ID, (int)newNodeId);
                    }
                    else
                    {
                        newVolumeFace.nodes.push_back(-duplicateId);
                        newOtherFace.nodes.push_back(-duplicateId);
                        edgePoint = INT4((int)faceID, (int)Node1ID, (int)Node2ID, (int)-duplicateId);
                    }
                    edgePoints.push_back(edgePoint);

                    // КОСТЫЛЬ
                    if (face.cell1 != cellNumber)
                    {
                        if (Node1V == -1)
                            std::swap(newVolumeFace.nodes[newVolumeFace.nodes.size() - 1], newVolumeFace.nodes[newVolumeFace.nodes.size() - 2]);
                        else
                            std::swap(newOtherFace.nodes[newOtherFace.nodes.size() - 1], newOtherFace.nodes[newOtherFace.nodes.size() - 2]);
                    }
                }
            }
        }

        INT3 split((int)faceID, (int)resultFaces[0].size(), (int)resultFaces[1].size());

        if (newVolumeFace.nodes.size() < 3)
            std::get<1>(split) = -1;

        if (newOtherFace.nodes.size() < 3)
            std::get<2>(split) = -1;

        faceSplit.push_back(split);

        if (std::get<1>(split) != -1)
            resultFaces[0].push_back(newVolumeFace);

        if (std::get<2>(split) != -1)
            resultFaces[1].push_back(newOtherFace);

        newVolumeFace.nodes.clear();
        newOtherFace.nodes.clear();
    }

    // Если проскость отсекает весь или пустой объёмы, то фукнция вернёт этот объём, никакие передаваемые далее параметры не будут затронуты!
    if (!resultFaces[0].size() || resultFaces[0].size() == 1 && std::count_if(resultFaces[0][0].nodes.begin(), resultFaces[0][0].nodes.end(), std::bind(std::less<int>(), std::placeholders::_1, 0)) == resultFaces[0][0].nodes.size())
        return 0.0;
    if (!resultFaces[1].size() || resultFaces[1].size() == 1 && std::count_if(resultFaces[1][0].nodes.begin(), resultFaces[1][0].nodes.end(), std::bind(std::less<int>(), std::placeholders::_1, 0)) == resultFaces[1][0].nodes.size())
        return getCellVolume(cell);

    // БЛОК 1.2. ОПРЕДЕЛИТЬ СТОРОНУ, ПОЛУЧАЕМУЮ КАК РЕЗУЛЬТАТ ПЕРЕСЕЧЕНИЯ С ПЛОСКОСТЬЮ
    CIrregFace planeFace;
    if (planePoints.size() > 2) // если нет точек пересечения или их меньше 3х, то и не будет новой грани.
    {
        const int initialeNodeID = -1; // Изначальная вершина
        rightUniquiePlanePoints.reserve(planePoints.size());
        rightUniquiePlanePoints.push_back(initialeNodeID);

        size_t startFaceId{};
        for (startFaceId = 0; startFaceId < resultFaces[0].size(); ++startFaceId)
        {
            bool was = false;
            const CIrregFace& face = resultFaces[0][startFaceId];

            if (std::count_if(face.nodes.begin(), face.nodes.end(), std::bind(std::less<int>(), std::placeholders::_1, 0)) < 2)
                continue; // если на грани меньше 2 отрицательных точек, то скипнем эту грань

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
            if (!startNodeID)
                throw std::exception("Неправильно восстановлена последовательность точек пересечения!\n");
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

                if (was)// на этом этапе найдена грань, содержащая соседнюю точку, отличная от текущей.
                {
                    nextNodeId = 0; // тут надо прикол
                    const CIrregFace& face = resultFaces[0][startFaceId];
                    for (size_t nodeId = 0; nodeId < face.nodes.size() && !nextNodeId; ++nodeId)
                        if (face.nodes[nodeId] < 0 && face.nodes[nodeId] != startNodeID)
                            if (std::find(rightUniquiePlanePoints.begin(), rightUniquiePlanePoints.end(), face.nodes[nodeId]) == rightUniquiePlanePoints.end() || face.nodes[nodeId] == initialeNodeID)
                                nextNodeId = face.nodes[nodeId]; // на этом этапе найдена следующая соседняя точка.

                    if (nextNodeId)
                    {
                        startNodeID = nextNodeId; // чтобы идти дальше
                        break;
                    }
                    else
                        was = false;
                }
            } 

            

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

            volume += Tetrahedron(Node1.likeCPoint(), Node2.likeCPoint(), faceCenter.likeCPoint(), volumeCenter.likeCPoint()).getVolume();
        }
    } // Сначала посчитали по известным граням

    if (planePoints.size()) // Теперь с учетом ещё одной (вообще говоря, этот пункт всегда должен работать)
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

            volume += Tetrahedron(Node1.likeCPoint(), Node2.likeCPoint(), planeCenter.likeCPoint(), volumeCenter.likeCPoint()).getVolume();
        }
    }

    if (OUTresultFaces)
    {
        OUTresultFaces[0] = std::move(resultFaces[0]);
        OUTresultFaces[1] = std::move(resultFaces[1]);
    }
    if (OUTplanePoints)
        *OUTplanePoints = std::move(planePoints);
    if (OUTplaneFace)
        *OUTplaneFace = std::move(planeFace);
    if (OUTexistingPointsIDs)
        *OUTexistingPointsIDs = std::move(existingPointsIDs);
    if (OUTfaceSplit)
        *OUTfaceSplit = std::move(faceSplit);
    if (OUTedgePoints)
        *OUTedgePoints = std::move(edgePoints);

    return volume;
}

void CIrregMesh::setEdgePoint(const int& cellNumber, const int& point1ID, const int& point2ID, const int& newPointID) noexcept
{
    std::pair<int, int> sortedPointsIDs = { point1ID < point2ID ? point1ID : point2ID, point1ID < point2ID ? point2ID : point1ID };
    std::vector<int> neighboringCells;
    CIrregCell& cell = cells[cellNumber];
    for (size_t i = 0; i < cell.facesInd.size(); ++i)
    {
        CIrregFace& face = faces[cell.facesInd[i]];
        for (size_t j = 0; j < face.nodes.size(); ++j)
        {
            const int Node1Id = face.nodes[j];
            const int Node2Id = face.nodes[(j + 1ull) % face.nodes.size()];
            std::pair<int, int> egde = { Node1Id < Node2Id ? Node1Id : Node2Id, Node1Id < Node2Id ? Node2Id : Node1Id };

            if (sortedPointsIDs == egde) // зачем считаю второй раз, если есть J?
            {
                std::vector<int>& fns = face.nodes;
                size_t pos{};
                for (; fns[pos] != point1ID && fns[pos] != point2ID; ++pos);

                if (fns[pos + 1] == point1ID || fns[pos + 1] == point2ID) // если 2 подряд
                    ++pos;

                fns.insert(fns.begin() + pos, newPointID); // вставляем новую точку на ребро.

                int neighboringCellID = face.cell1 == cellNumber ? face.cell2 : face.cell1;
                if (neighboringCellID != -1)
                    neighboringCells.push_back(neighboringCellID); // запишем соседние клетки с общим ребром.
                break; // переходим к следующей грани.
            }
        }
    }
    for (size_t i = 0; i < neighboringCells.size(); ++i)
        setEdgePoint(neighboringCells[i], point1ID, point2ID, newPointID); // для данного ребра всех соседних клеток вызовём данную функцию
}

void CIrregMesh::addRandomFigure(const CPoint& center)
{
    int figure = rand() % 3;

    int startFace = faces.size();
    int startNode = nodes.size();

    double randEdge = 0.1 + 100. * rand() / RAND_MAX;
    double randNodeShift = randEdge * rand() / RAND_MAX * (rand() % 2 ? 0.5 : -1);
    if (figure == 0) // кубик
    {
        nodes.push_back(CSurfaceNode(center.X() - randEdge / 2 + randNodeShift, center.Y() - randEdge / 2 + randNodeShift, center.Z() - randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() - randEdge / 2 + randNodeShift, center.Y() - randEdge / 2 + randNodeShift, center.Z() + randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() - randEdge / 2 + randNodeShift, center.Y() + randEdge / 2 - randNodeShift, center.Z() - randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() - randEdge / 2 + randNodeShift, center.Y() + randEdge / 2 - randNodeShift, center.Z() + randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + randEdge / 2 - randNodeShift, center.Y() - randEdge / 2 + randNodeShift, center.Z() - randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + randEdge / 2 - randNodeShift, center.Y() - randEdge / 2 + randNodeShift, center.Z() + randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + randEdge / 2 - randNodeShift, center.Y() + randEdge / 2 - randNodeShift, center.Z() - randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + randEdge / 2 - randNodeShift, center.Y() + randEdge / 2 - randNodeShift, center.Z() + randEdge / 2));

        CIrregFace Faces;
        Faces.cell1 = cells.size();
        Faces.cell2 = -1;

        Faces.nodes = { startNode + 0, startNode + 4, startNode + 6, startNode + 2 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 1, startNode + 3, startNode + 7, startNode + 5 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 1, startNode + 5, startNode + 4, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 3, startNode + 2, startNode + 6, startNode + 7 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 1, startNode + 0, startNode + 2, startNode + 3 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 7, startNode + 6, startNode + 4, startNode + 5 };
        faces.push_back(Faces);

        CIrregCell cell;
        cell.facesInd = { startFace , startFace + 1,startFace + 2,startFace + 3,startFace + 4,startFace + 5 };

        cells.push_back(cell);
    }
    else if (figure == 1) // 8 углов
    {
        nodes.push_back(CSurfaceNode(center.X(), center.Y(), center.Z() + 5 * randEdge / 2 + randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() - 3 * randEdge / 2, center.Y(), center.Z() + 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2, center.Y() - 2 * randEdge / 2, center.Z() + 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X(), center.Y() - 3 * randEdge / 2, center.Z() + 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2, center.Y() - 2 * randEdge / 2, center.Z() + 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + 3 * randEdge / 2, center.Y(), center.Z() + 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2, center.Y() + 2 * randEdge / 2, center.Z() + 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X(), center.Y() + 3 * randEdge / 2, center.Z() + 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2, center.Y() + 2 * randEdge / 2, center.Z() + 3 * randEdge / 2));

        nodes.push_back(CSurfaceNode(center.X() - 3 * randEdge / 2, center.Y(), center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2, center.Y() - 2 * randEdge / 2, center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X(), center.Y() - 3 * randEdge / 2, center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2, center.Y() - 2 * randEdge / 2, center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + 3 * randEdge / 2, center.Y(), center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2, center.Y() + 2 * randEdge / 2, center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X(), center.Y() + 3 * randEdge / 2, center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2, center.Y() + 2 * randEdge / 2, center.Z() - 3 * randEdge / 2));
        nodes.push_back(CSurfaceNode(center.X(), center.Y(), center.Z() - 5 * randEdge / 2 - randNodeShift));

        CIrregFace Faces;
        Faces.cell2 = cells.size();
        Faces.cell1 = -1;

        Faces.nodes = { startNode + 1, startNode + 2, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 2, startNode + 3, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 3, startNode + 4, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 4, startNode + 5, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 5, startNode + 6, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 6, startNode + 7, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 7, startNode + 8, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 8, startNode + 1, startNode + 0 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 1, startNode + 9, startNode + 10, startNode + 2 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 2, startNode + 10, startNode + 11, startNode + 3 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 3, startNode + 11, startNode + 12, startNode + 4 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 4, startNode + 12, startNode + 13, startNode + 5 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 5, startNode + 13, startNode + 14, startNode + 6 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 6, startNode + 14, startNode + 15, startNode + 7 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 7, startNode + 15, startNode + 16, startNode + 8 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 8, startNode + 16, startNode + 9, startNode + 1 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 9, startNode + 17, startNode + 10 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 10, startNode + 17, startNode + 11 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 11, startNode + 17, startNode + 12 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 12, startNode + 17, startNode + 13 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 13, startNode + 17, startNode + 14 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 14, startNode + 17, startNode + 15 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 15, startNode + 17, startNode + 16 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 16, startNode + 17, startNode + 9 };
        faces.push_back(Faces);

        CIrregCell cell;
        cell.facesInd = { startFace + 0,startFace + 1,startFace + 2,startFace + 3,startFace + 4,startFace + 5,startFace + 6,startFace + 7,startFace + 8,startFace + 9,startFace + 10,startFace + 11,
        startFace + 12, startFace + 13, startFace + 14, startFace + 15, startFace + 16, startFace + 17, startFace + 18, startFace + 19, startFace + 20, startFace + 21, startFace + 22, startFace + 23 };

        cells.push_back(cell);
    }
    else if (figure == 2) // гробик
    {
        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2 - randNodeShift, center.Y() - 2.5 * randEdge / 2 - randNodeShift, center.Z() - 1.5 * randEdge / 2 - randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() - 1.5 * randEdge / 2, center.Y() - 2.5 * randEdge / 2, center.Z() - 1.5 * randEdge / 2 - randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2, center.Y() + 1.5 * randEdge / 2, center.Z() - 1.5 * randEdge / 2 - randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2 + randNodeShift, center.Y() + 2.5 * randEdge / 2 + randNodeShift, center.Z() - 1.5 * randEdge / 2 - randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() + 1.5 * randEdge / 2, center.Y() + 2.5 * randEdge / 2, center.Z() - 1.5 * randEdge / 2 - randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2, center.Y() - 1.5 * randEdge / 2, center.Z() - 1.5 * randEdge / 2 - randNodeShift));

        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2 - randNodeShift, center.Y() - 2.5 * randEdge / 2 - randNodeShift, center.Z() + 1.5 * randEdge / 2+randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() - 1.5 * randEdge / 2, center.Y() - 2.5 * randEdge / 2, center.Z() + 1.5 * randEdge / 2+randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2, center.Y() + 1.5 * randEdge / 2, center.Z() + 1.5 * randEdge / 2+randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() + 2.5 * randEdge / 2 + randNodeShift, center.Y() + 2.5 * randEdge / 2 + randNodeShift, center.Z() + 1.5 * randEdge / 2+randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() + 1.5 * randEdge / 2, center.Y() + 2.5 * randEdge / 2, center.Z() + 1.5 * randEdge / 2+randNodeShift));
        nodes.push_back(CSurfaceNode(center.X() - 2.5 * randEdge / 2, center.Y() - 1.5 * randEdge / 2, center.Z() + 1.5 * randEdge / 2+randNodeShift));

        CIrregFace Faces;
        Faces.cell1 = cells.size();
        Faces.cell2 = -1;

        Faces.nodes = { startNode + 0, startNode + 1, startNode + 2, startNode + 3, startNode + 4, startNode + 5 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 0, startNode + 6, startNode + 7, startNode + 1};
        faces.push_back(Faces);

        Faces.nodes = { startNode + 11, startNode + 6, startNode + 0, startNode + 5 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 1, startNode + 7, startNode + 8, startNode + 2 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 9, startNode + 3, startNode + 2, startNode + 8 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 9, startNode + 10, startNode + 4, startNode + 3 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 10, startNode + 11, startNode + 5, startNode + 4 };
        faces.push_back(Faces);

        Faces.nodes = { startNode + 6, startNode + 11, startNode + 10, startNode + 9, startNode + 8, startNode + 7 };
        faces.push_back(Faces);

        CIrregCell cell;
        cell.facesInd = { startFace + 0, startFace + 1, startFace + 2, startFace + 3, startFace + 4, startFace + 5, startFace + 6, startFace + 7 };

        cells.push_back(cell);
    }
}

std::vector<int> CIrregMesh::getCellNodesIds(const int& cellNumber) const
{
    const CIrregCell& cell = cells[cellNumber];
    std::vector<int> res;
    for (int fId = 0; fId < cell.facesInd.size(); ++fId)
    {
        const CIrregFace& face = faces[cell.facesInd[fId]];
        for (int nId = 0; nId < face.nodes.size(); ++nId)
        {
            const int& node = face.nodes[nId];
            if (std::find(res.begin(), res.end(), node) == res.end())
                res.push_back(node);
        }
    }

    std::sort(res.begin(), res.end());

    return res;
}


#ifdef NEWTON
void CIrregMesh::setMode(Mode newMode) noexcept
{
    mode = newMode;
}
#endif // NEWTON


CSurfaceNode CIrregMesh::getCellCenter(const CIrregCell& cell) const noexcept
{
    std::vector<int> allNodes;
    for (size_t faceId = 0; faceId < cell.facesInd.size(); ++faceId)
    {
        const CIrregFace& face = faces[cell.facesInd[faceId]];
        for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
            allNodes.push_back(face.nodes[nodeId]);
    }

    std::sort(allNodes.begin(), allNodes.end());
    allNodes.erase(std::unique(allNodes.begin(), allNodes.end()), allNodes.end());


    CVector center;
    for (size_t i = 0; i < allNodes.size(); ++i)
        center += nodes[allNodes[i]];

    center /= allNodes.size();

    return center.toCSurfaceNode();
}

CSurfaceNode CIrregMesh::getFaceCenter(const CIrregFace& face) const noexcept
{
    CVector center;
    for (size_t i = 0; i < face.nodes.size(); ++i)
        center += nodes[face.nodes[i]];
    center /= face.nodes.size();

    return center.toCSurfaceNode();
}

std::vector<Tetrahedron> CIrregMesh::spliteCellByTetrahedrons(const CIrregCell& cell) const noexcept
{
    std::vector<Tetrahedron> res;

    CSurfaceNode cellCenter = getCellCenter(cell);

    for (size_t faceId = 0; faceId < cell.facesInd.size(); ++faceId)
    {
        const CIrregFace& face = faces[cell.facesInd[faceId]];

        CSurfaceNode faceCenter = getFaceCenter(face);

        for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
        {
            const CSurfaceNode& node1 = nodes[face.nodes[nodeId]];
            const CSurfaceNode& node2 = nodes[face.nodes[(1ull + nodeId) % face.nodes.size()]];
            res.push_back(Tetrahedron(cellCenter, faceCenter, node1, node2));
        }
    }

    return res;
}
