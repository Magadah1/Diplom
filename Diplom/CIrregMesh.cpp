#include "CIrregMesh.h"

CIrregMesh::CIrregMesh() noexcept
{
}

std::vector<CIrregCell> CIrregMesh::FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume)
{
    constexpr double eps = 1e-8; // �������� � C++11, ���� �� �������� - �������� �� const double
    const CIrregCell& cell = cells[cellNumber]; // �������� ������ �����

    CPoint mid = (CVector(startPoint) + endPoint) / 2.; // �������� ������� [startPoint, endPoint]
    
    CSideEquation plane(CVector(startPoint).createVector(endPoint)); // ������ ���������, ������������ �� startPoint � endPoint

    std::vector<CIrregCell> resultCells(2); // ����� ������. 0 - ��, ��� ����� ���� (�� ������ ������� �� �������). 1 - ����������.
    int newCellId = cells.size(); // ����� ����� � ������ 1. � 0 �� ����������.
    int newFaceId = faces.size(); // ��� ������ �������, ������� ����� ������� ������ �������� ����� �������� ������. ��� ����� ��� ������ 1, � ����� �������, ���������� � ������� ���������.
    int newNodeId = 0; // ��� ������ ����� ������� ������� ������. ��� ����� ��������������, �� � �� ������� ����� ���������������.

    // ����� �������� �� ��������������� ������� �� ������� ���������
    double tempVolume{};

    //
    std::vector<CIrregFace> resultFaces[2]; // ������� ����� ������; ������� 0 - ������ �� ������ ������� �� �������, 1 - ������.
    std::vector<CSurfaceNode> planePoints; // ����� �������, ������� ����� ��������� ��� ����������� � ���������� ���������. � "������������� ��������" � ����� ��������� ������.
    //

    while (true)
    {
        // ���� 1. ��������� ������ �� 2 � ������� ���������.
        plane.recalcD(mid); // ������ ��������� �������� ����� ����� mid
        
        // �������� �� ���� �������� ������
        for (int cellFaceI = 0; cellFaceI < cell.facesInd.size(); ++cellFaceI) 
        {
            const CIrregFace& face = faces[cell.facesInd[cellFaceI]]; // ������� �������
            resultCells[0].facesInd.push_back(cell.facesInd[cellFaceI]); // ��������� ����� ������� ��� ����� ������.

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
            for (int faceNodeI = 0; faceNodeI < face.nodes.size(); ++faceNodeI) // � �������?????????
            {
                const CSurfaceNode& Node1 = nodes[faceNodeI];                               // ������ ����� ��� �������.
                const CSurfaceNode& Node2 = nodes[(faceNodeI + 1ull) % face.nodes.size()];  // ������ ����� ��� �������. ���� �����.

                const double Node1V = plane.getNormedValue(Node1);
                const double Node2V = plane.getNormedValue(Node2);

                if (Node1V == Node2V && Node1V) // ���� ������� �� ���� ������� �� ���������, �� �������� �� �����������
                {
                    if (Node1V == -1)
                        newVolumeFace.nodes.push_back(faceNodeI);
                    else
                        newOtherFace.nodes.push_back(faceNodeI);
                }
                else
                {
                    if (Node1V == Node2V && !Node1V) // ��� ����� �� ���������, ��������� ������ �� ��� � ������ �� ����� ������ � ��������� � ����� �������, ��� ���������� ����������.
                    {
                        newVolumeFace.nodes.push_back(faceNodeI);
                        newOtherFace.nodes.push_back(faceNodeI);
                        planePoints.push_back(Node1); --newNodeId;
                    }
                    else
                    {
                        // ���� ����� �� ������ �������, �� ������� �����������. ����� ����� ��������� � ����������� ������ � ����������� �� "�������������" ������.
                        if (!plane.getIntersectionPointOnLine(Node1, Node2, &interPoint)) 
                        {
                            throw std::exception("�������, ������ ���� ���������!\n");
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
        // ���� 1.1. ���������� �������, ���������� ��� ��������� ����������� � ����������
        CIrregFace planeFace;
        // ����� ���-�� ����������:
        // planeFace.cell1 = ?
        // planeFace.cell2 = ?
        // planeFace.nodes <= unique planePoints
        faces.push_back(planeFace);
        

        // ���� 2. ������� ��ڨ� ������ ������. �������� �� �������!!!!
        CIrregCell& newCell = resultCells[0];
        newCell.facesInd.push_back(newFaceId);

        std::vector<Tetrahedron> cellTetrahedrons = spliteCellByTetrahedrons(newCell);
        for (int i = 0; i < cellTetrahedrons.size(); ++i)
            tempVolume += cellTetrahedrons[i].getVolume();
        

        // ���� 3. ��������� �������.
        if (abs(tempVolume - volume) < eps) // ���������, ������� ������ �����
        {
            // ���� 4. ��������� ������ � �������� �����.


            break;
        }
        else
        {
            //resultCells.clear();
            resultFaces[0].clear();
            resultFaces[1].clear();
            // ��������� ������
            newFaceId = faces.size(); 
            newNodeId = 0;
            // ������� ����������� �������
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
