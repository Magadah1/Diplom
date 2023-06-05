#pragma once
#include <vector>
#include <tuple>
#include "CIrregCell.h"
#include "CIrregFace.h"
#include "CSurfaceNode.h"
#include "CSideEquation.h"
#include "Tetrahedron.h"
#include "CException.h"

#define NEWTON

class CIrregMesh
{
public:
	using INT3 = std::tuple<int, int, int>;         // 3 int-а, идущие подряд
	using INT4 = std::tuple<int, int, int, int>;    // 4 int-а, идущие подрад

	CIrregMesh() noexcept;
	std::vector<CIrregCell> cells;
	std::vector<CIrregFace> faces;
	std::vector<CSurfaceNode> nodes;

	std::pair<int, int> FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume);

	void spliteFaceByTriangles(const int& faceNumber); // не затрагивает рёбра.

	double getCellVolume(const CIrregCell& cell) const noexcept;

	// OUTresultFaces - массив из 2 элементов! 
	// plane - уже проходит через нужную точку!
	// В граничном случае НЕ ИЗМЕНЯЕТ передаваемые параметры даже если они есть!
	double getCutOffCellVolume(const int& cellNumber, const CSideEquation& plane,
		std::vector<CIrregFace>* const OUTresultFaces = nullptr,
		std::vector<CSurfaceNode>* const OUTplanePoints = nullptr,
		CIrregFace* const OUTplaneFace = nullptr,
		CVector* const OUTvolumeCenter = nullptr,
		std::vector<std::pair<int, int>>* const OUTexistingPointsIDs = nullptr,
		std::vector<INT3>* const OUTfaceSplit = nullptr, 
		std::vector<INT4>* const OUTedgePoints = nullptr);

	void setEdgePoint(const int& cellNumber, const int& point1ID, const int& point2ID, const int& newPointID) noexcept;

	template<typename OStream>
	void sayInfo(OStream& out);

	void addRandomFigure(const CPoint& center);

	std::vector<int> getCellNodesIds(const int& cellNumber) const;

#ifdef NEWTON
	enum class Mode
	{
		OLD,
		NEW,
		NEWMODIFIED
	};

	void setMode(Mode newMode) noexcept;
#endif // NEWTON

private:
	CSurfaceNode getCellCenter(const CIrregCell& cell) const noexcept;
	CSurfaceNode getFaceCenter(const CIrregFace& face) const noexcept;

	std::vector<Tetrahedron> spliteCellByTetrahedrons(const CIrregCell& cell) const noexcept;

#ifdef NEWTON
	Mode mode;
#endif // NEWTON
};

template<typename OStream>
inline void CIrregMesh::sayInfo(OStream& out)
{
	out << "Totat cells = " << cells.size() << "; Total faces = " << faces.size() << "; Total nodes = " << nodes.size() << '\n';
	double totalVolume{};
	for (size_t cId = 0; cId < cells.size(); ++cId)
	{
		const CIrregCell& cell = cells[cId];

		out << "---------------------------------------------------\n";
		double cellVolume = getCellVolume(cell);
		totalVolume += cellVolume;
		out << "Cell #" << cId << " with volume = " << cellVolume << "; with " << cell.facesInd.size() << " faces\n";

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

		out << "---------------------------------------------------\n";
	}
	out << "Total cell volume = " << totalVolume << "\n\n\n";

}