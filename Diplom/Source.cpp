#include <iostream>

#include "CIrregMesh.h"

int main(int argc, char** argv)
{
	CPoint p1(5, 2, 1);
	CPoint p2(2, 3, 2);
	CSideEquation side(0, 0, 1, 0);
	CPoint interPoint;

	if (side.getIntersectionPointOnLine(p1, p2, &interPoint))
		std::cout << interPoint.X() << '\t' << interPoint.Y() << '\t' << interPoint.Z();

	std::cout << "\n" << interPoint.X() << '\t' << interPoint.Y() << '\t' << interPoint.Z();

}