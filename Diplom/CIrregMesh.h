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

	double getCellVolume(const CIrregCell& cell) const noexcept;

	template<typename UniqueType, typename CastTo = UniqueType>
	static std::vector<UniqueType> getUniqueElements(const std::vector<UniqueType>& elements);

private:
	CSurfaceNode getCellCenter(const CIrregCell& cell) const noexcept;
	CSurfaceNode getFaceCenter(const CIrregFace& face) const noexcept;

	std::vector<Tetrahedron> spliteCellByTetrahedrons(const CIrregCell& cell) const noexcept;
};

template<typename UniqueType, typename CastTo>
inline std::vector<UniqueType> CIrregMesh::getUniqueElements(const std::vector<UniqueType>& elements)
{
	std::vector<UniqueType> unique;
	for (size_t i = 0; i < elements.size(); ++i)
	{
		bool was = false;
		for (size_t j = 0; j < unique.size(); ++j)
			if (CastTo(elements[i]) == CastTo(unique[j]))
			{
				was = true;
				break;
			}

		if (!was)
			unique.push_back(elements[i]);
	}
	return unique;
}
