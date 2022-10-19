#include "CIrregMesh.h"

CIrregMesh::CIrregMesh() noexcept
{
}

std::pair<int, int> CIrregMesh::FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume)
{
    constexpr double eps = 1e-8; // �������� � C++11, ���� �� �������� - �������� �� const double
    const CIrregCell& cell = cells[cellNumber]; // �������� ������ �����

    if (getCellVolume(cell) > volume)
        throw std::exception("������� ����� ������ ������ ����� ������!\n");

    CPoint mid = (CVector(startPoint) + endPoint) / 2.; // �������� ������� [startPoint, endPoint]
    
    CSideEquation plane(CVector(startPoint).createVector(endPoint)); // ������ ���������, ������������ �� startPoint � endPoint

    //std::vector<CIrregCell> resultCells(2); // ����� ������. 0 - ��, ��� ����� ���� (�� ������ ������� �� �������). 1 - ����������.
    int newCellId = cells.size(); // ����� ����� � ������ 1. � 0 �� ����������.
    int newFaceId = faces.size(); // ��� ������ �������, ������� ����� ������� ������ �������� ����� �������� ������. ��� ����� ��� ������ 1, � ����� �������, ���������� � ������� ���������.
    int newNodeId = 0; // ��� ������ ����� ������� ������� ������. ��� ����� ��������������, �� � �� ������� ����� ���������������.

    // ����� �������� �� ��������������� ������� �� ������� ���������
    double tempVolume{};

    //
    std::vector<CIrregFace> resultFaces[2]; // ������� ����� ������; ������� 0 - ������ �� ������ ������� �� �������, 1 - ������.
    std::vector<CSurfaceNode> planePoints; // ����� �������, ������� ����� ��������� ��� ����������� � ���������� ���������. � "������������� ��������" � ����� ��������� ������.
    std::vector<int> rightUniquiePlanePoints; // ��� ����� ���� ���������� ����� ���������, ������ ������������� � ���������� (���� �� ������) �������.
    //

    while (true)
    {
        // ���� 1. ��������� ������ �� 2 � ������� ���������.
        plane.recalcD(mid); // ������ ��������� �������� ����� ����� mid
        tempVolume = 0.0;
        
        // �������� �� ���� �������� ������
        for (int cellFaceI = 0; cellFaceI < cell.facesInd.size(); ++cellFaceI) 
        {
            const CIrregFace& face = faces[cell.facesInd[cellFaceI]]; // ������� �������


            // ������!!!
            //resultCells[0].facesInd.push_back(cell.facesInd[cellFaceI]); // ��������� ����� ������� ��� ����� ������. ??????????

            // ������������ ��������� ������� ������ �� 2. ������ �������� ���� ����� � ����������.
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

            // ����� �������� ����� �����������.
            CSurfaceNode interPoint;
            bool duplicatePoint = false;    // ���������� ����� ����������� ����� ����� ���� ��������.
            int duplicateId{};              // ���� ���� ��������, �� ������� ��� ������
            for (int faceNodeI = 0; faceNodeI < face.nodes.size(); ++faceNodeI)
            {
                const int Node1ID = face.cell1 == cellNumber ? face.nodes[faceNodeI] : face.nodes[(faceNodeI + 1ull) % face.nodes.size()];  // ������ ������ ����� � ���������� �������.
                const int Node2ID = face.cell1 == cellNumber ? face.nodes[(faceNodeI + 1ull) % face.nodes.size()] : face.nodes[faceNodeI];  // ������ ������ ����� � ���������� �������.
                const CSurfaceNode& Node1 = face.cell1 == cellNumber ? nodes[Node1ID] : nodes[Node2ID];    // ������ ����� ��� �������.
                const CSurfaceNode& Node2 = face.cell1 == cellNumber ? nodes[Node2ID] : nodes[Node1ID];    // ������ ����� ��� �������. ���� �����.

                const int Node1V = plane.getNormedValue(Node1);
                const int Node2V = plane.getNormedValue(Node2);

                if (Node1V == Node2V && Node1V) // ���� ������� �� ���� ������� �� ���������, �� �������� �� �����������
                {
                    if (Node1V == -1)
                        newVolumeFace.nodes.push_back(Node1ID);
                    else
                        newOtherFace.nodes.push_back(Node1ID);
                }
                else
                {
                    if (Node1V == Node2V && !Node1V) // ��� ����� �� ���������, ��������� ������ �� ��� � ������ �� ����� ������ � ��������� � ����� �������, ��� ���������� ����������.
                    {
                        newVolumeFace.nodes.push_back(Node1ID);
                        newOtherFace.nodes.push_back(Node1ID);

                        duplicatePoint = false;
                        for (duplicateId = 0; duplicateId < planePoints.size() && !duplicatePoint; ++duplicateId)
                            if (planePoints[duplicateId] == Node1)
                                duplicatePoint = true;

                        if (!duplicatePoint)
                            planePoints.push_back(Node1); --newNodeId;
                    }
                    else if (!Node1V || !Node2V) // ���� ����� �� ���������. ������ �� �����-�� �������
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
                        // ���� ����� �� ������ �������, �� ������� �����������. ����� ����� ��������� � ����������� ������ � ����������� �� "�������������" ������.
                        if (!plane.getIntersectionPointOnLine(Node1, Node2, &interPoint)) 
                        {
                            throw std::exception("�������, ������ ���� ���������!\n");
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
                }
            }

            resultFaces[0].push_back(newVolumeFace);
            resultFaces[1].push_back(newOtherFace);
            newVolumeFace.nodes.clear();
            newOtherFace.nodes.clear();
        }
        // ���� 1.1. ���������� �������, ���������� ��� ��������� ����������� � ����������
        CIrregFace planeFace;
        if (planePoints.size()) // ���� ��� ����� �����������, �� � �� ����� ����� �����.
        {
            const int initialeNodeID = -1; // ����������� �������
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
            } // �� ���� ����� ������� ����������� ������� � �����-�� ���������� � �����.

            int startNodeID{}; // ��� �������� � ����������� �������� �������.

            for (size_t nodeId = 0; nodeId < resultFaces[0][startFaceId].nodes.size() && !startNodeID; ++nodeId)
                if (resultFaces[0][startFaceId].nodes[nodeId] < 0 && resultFaces[0][startFaceId].nodes[nodeId] != initialeNodeID)
                    startNodeID = resultFaces[0][startFaceId].nodes[nodeId]; // �� ���� ����� ������� �������� ������� � ����������� �� ��� �� �����.


            int nextNodeId{}; // ����� ������ � ��� ������� �� ������ ����� �� ��� ���, ���� �� ��������� ����������� �������
            do
            {
                rightUniquiePlanePoints.push_back(startNodeID); // ���������� �������, ������� ��� �� �������� ������� ����� ������ ��������
                
                if (rightUniquiePlanePoints.size() > planePoints.size())
                    throw std::exception("������ ����� ���� ������ 2� ����������� � �������� ����� �����!\n");

                for (size_t faceID = 0; faceID < resultFaces[0].size(); ++faceID)
                {
                    bool was = false;
                    const CIrregFace& face = resultFaces[0][faceID];

                    for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
                    {
                        if (face.nodes[nodeId] == startNodeID && faceID != startFaceId)
                        {
                            startFaceId = faceID;
                            was = true;
                            break;
                        }
                    }

                    if (was)
                        break;
                } // �� ���� ����� ������� �����, ���������� �������� �����, �������� �� �������.

                nextNodeId = 0;
                const CIrregFace& face = resultFaces[0][startFaceId];
                for (size_t nodeId = 0; nodeId < face.nodes.size() && !nextNodeId; ++nodeId)
                    if (face.nodes[nodeId] < 0 && face.nodes[nodeId] != startNodeID)
                        nextNodeId = face.nodes[nodeId]; // �� ���� ����� ������� ��������� �������� �����.

                startNodeID = nextNodeId; // ����� ���� ������

            } while (nextNodeId != initialeNodeID);
        }
        planeFace.nodes = rightUniquiePlanePoints;

        std::vector<int> volumeNodes; // ������� ��� ����� ������, ����� ������� ����.
        for (size_t faceId = 0; faceId < resultFaces[0].size(); ++faceId)
        {
            const CIrregFace& face = resultFaces[0][faceId];
            for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
                volumeNodes.push_back(face.nodes[nodeId]);
        }
        for (size_t nodeId = 0; nodeId < rightUniquiePlanePoints.size(); ++nodeId)
            volumeNodes.push_back(rightUniquiePlanePoints[nodeId]);

        std::sort(volumeNodes.begin(), volumeNodes.end());
        volumeNodes.erase(std::unique(volumeNodes.begin(), volumeNodes.end()), volumeNodes.end()); // ��������� ������ ���������� �������

        CVector volumeCenter;
        for (size_t i = 0; i < volumeNodes.size(); ++i)
        {
            const int& nodeID = volumeNodes[i];

            if (nodeID < 0)
                volumeCenter += planePoints[-nodeID - 1];
            else
                volumeCenter += nodes[nodeID];
        }
        volumeCenter /= volumeNodes.size(); // ������ ����� ��� ����� ������, ����� ������� ����.
        

        // ������� ��������� ����� ������ ����� ������, � ����� �������� ��������� �����, ������������ ���������� � ����� ����������

        // ���� 2. ������� ��ڨ� ������ ������.
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
        } // ������� ��������� �� ��������� ������

        if (planePoints.size()) // ������ � ������ ��� �����
        {
            CVector planeCenter; // �������� ����� �����, ���������� ����������
            for (size_t nodeId = 0; nodeId < rightUniquiePlanePoints.size(); ++nodeId)
                planeCenter += planePoints[-rightUniquiePlanePoints[nodeId] - 1];
            planeCenter /= rightUniquiePlanePoints.size();

            for (size_t nodeId = 0; nodeId < rightUniquiePlanePoints.size(); ++nodeId)
            {
                const int& id1 = rightUniquiePlanePoints[nodeId];
                const int& id2 = rightUniquiePlanePoints[(1ull + nodeId) % rightUniquiePlanePoints.size()];

                const CSurfaceNode& Node1 = planePoints[-id1 - 1];
                const CSurfaceNode& Node2 = planePoints[-id2 - 1];

                tempVolume += Tetrahedron(Node1.likeCPoint(), Node2.likeCPoint(), planeCenter.likeCPoint(), volumeCenter.likeCPoint()).getVolume();
            }
        }

        // ���� 3. ��������� �������.
        if (abs(tempVolume - volume) < eps) // ���������, ������� ������ �����
        {
            // ���� 3.1. ��������� ������ � �������� �����.

            // planeFace.cell1 = ?
            // planeFace.cell2 = ?

            break;
        }
        else
        {
            // ���� 3.2. ��������� ���ר� ��������� ��������� � �.�.
            //resultCells.clear(); resultCells.resize(2);
            resultFaces[0].clear();
            resultFaces[1].clear();
            // ��������� ������
            newFaceId = faces.size(); 
            newNodeId = 0;
            // ������� ����������� �������
            // faces.pop_back();
            if (tempVolume > volume)
                endPoint = mid;
            else
                startPoint = mid;

            mid = (CVector(startPoint) + endPoint) / 2.;
        }
    }



    return { 0,0 };
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
        for (int nodeId = 0; nodeId < faces[faceId].nodes.size(); ++nodeId)
            allNodes.push_back(faces[faceId].nodes[nodeId]);

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
