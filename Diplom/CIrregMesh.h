#pragma once
#include <vector>
#include <tuple>
#include "CIrregCell.h"
#include "CIrregFace.h"
#include "CSurfaceNode.h"
#include "CSideEquation.h"
#include "Tetrahedron.h"

class CIrregMesh
{
public:
	CIrregMesh() noexcept;
	std::vector<CIrregCell> cells;
	std::vector<CIrregFace> faces;
	std::vector<CSurfaceNode> nodes;

	std::pair<int, int> FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume);

	void spliteFaceByTriangles(const int& faceNumber); // не затрагивает рёбра.

	double getCellVolume(const CIrregCell& cell) const noexcept;

	void setEdgePoint(const int& cellNumber, const int& point1ID, const int& point2ID, const int& newPointID) noexcept;

	template<typename OStream>
	void sayInfo(OStream& out);

private:
	CSurfaceNode getCellCenter(const CIrregCell& cell) const noexcept;
	CSurfaceNode getFaceCenter(const CIrregFace& face) const noexcept;

	std::vector<Tetrahedron> spliteCellByTetrahedrons(const CIrregCell& cell) const noexcept;
};

template<typename OStream>
inline void CIrregMesh::sayInfo(OStream& out)
{
	out << "Totat cells = " << cells.size() << "; Total faces = " << faces.size() << "; Total nodes = " << nodes.size() << '\n';
	for (size_t cId = 0; cId < cells.size(); ++cId)
	{
		const CIrregCell& cell = cells[cId];

		out << "---------------------------------------------------\n";
		out << "Cell #" << cId << " with volume = " << getCellVolume(cell) << "; with " << cell.facesInd.size() << " faces\n";

		for (size_t fId = 0; fId < cell.facesInd.size(); ++fId)
		{
			const int& faceID = cell.facesInd[fId];
			const CIrregFace& face = faces[faceID];

			out << "\tFace #" << faceID << " with cell1 = " << face.cell1 << " and cell2 = " << face.cell2 << "; with " << face.nodes.size() << " nodes\n";

			for (size_t nId = 0; nId < face.nodes.size(); ++nId)
			{
				const int& nodeID = face.nodes[nId];
				const CSurfaceNode& node = nodes[nodeID];

				out << "\t\tNode #" << nodeID << " = {" << node.X() << " ; " << node.Y() << " ; " << node.Z() << "}\n";
			}

			out << '\n';
		}

		out << "---------------------------------------------------\n\n\n\n";
	}
}
