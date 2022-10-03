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
    int newFaceId = faces.size(); // ��� ������ �������, ������� ����� ������� ������ �������� ����� �������� ������. ��� ����� ��� ������ 1.
    int newNodeId = 0; // ��� ������ ����� ������� ������� ������. ��� ����� ��������������, �� � �� ������� ����� ���������������.

    // ����� �������� �� ��������������� ������� �� ������� ���������
    double tempVolume{};

    //
    std::vector<CIrregFace> resultFaces; // ������� ����� ������; �������
    std::vector<CSurfaceNode> planePoints; // ����� �������, ������� ����� ��������� ��� ����������� � ���������� ���������. � "������������� ��������"
    //

    while (true)
    {
        // ���� 1. ��������� ������ �� 2 � ������� ���������.
        plane.recalcD(mid); // ������ ��������� �������� ����� ����� mid
        
        // �������� �� ���� �������� ������
        for (int cellFaceI = 0; cellFaceI < cell.facesInd.size(); ++cellFaceI) 
        {
            const CIrregFace& face = faces[cell.facesInd[cellFaceI]]; // ������� �������

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

            resultFaces.push_back(newVolumeFace);
            resultFaces.push_back(newOtherFace);
            newVolumeFace.nodes.clear();
            newOtherFace.nodes.clear();
        }
        // ���� 1.1. ���������� �������, ���������� ��� ��������� ����������� � ����������

        // ���� 2. ������� ��ڨ� ������ ������. �������� �� �������!!!!

        // ���� 3. ��������� �������.
        if (abs(tempVolume - volume) < eps) // ���������, ������� ������ �����
            break;
        else
        {
            resultCells.clear();
            resultFaces.clear();
            newFaceId = faces.size(); // ��������� ������

            if (tempVolume > volume)
                endPoint = mid;
            else
                startPoint = mid;

            mid = (CVector(startPoint) + endPoint) / 2.;
        }
    }



    return resultCells;
}
