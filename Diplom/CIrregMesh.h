#pragma once
#include <vector>
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

	std::vector<CIrregCell> FindContactBorder(const int& cellNumber, CPoint startPoint, CPoint endPoint, const double& volume);

private:
	CSurfaceNode getCellCenter(const CIrregCell& cell) const noexcept;
	CSurfaceNode getFaceCenter(const CIrregFace& face) const noexcept;

	std::vector<Tetrahedron> spliteCellByTetrahedrons(const CIrregCell& cell) const noexcept;
};

