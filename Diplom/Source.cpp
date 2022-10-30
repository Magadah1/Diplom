#include <iostream>

#include "CIrregMesh.h"

int main(int argc, char** argv)
{
	setlocale(0, "");
	CIrregMesh testMesh;
	testMesh.nodes.push_back({ 0,0,0 }); //0
	testMesh.nodes.push_back({ 0,0,1 }); //1
	testMesh.nodes.push_back({ 0,1,0 }); //2
	testMesh.nodes.push_back({ 0,1,1 }); //3
	testMesh.nodes.push_back({ 1,0,0 }); //4
	testMesh.nodes.push_back({ 1,0,1 }); //5 
	testMesh.nodes.push_back({ 1,1,0 }); //6
	testMesh.nodes.push_back({ 1,1,1 }); //7

	CIrregFace testFace;
	testFace.cell1 = 0;
	testFace.cell2 = -1;
	
	testFace.nodes = {0,4,6,2};
	testMesh.faces.push_back(testFace);

	testFace.nodes = { 1,3,7,5 };
	testMesh.faces.push_back(testFace);

	testFace.nodes = { 1,5,4,0 };
	testMesh.faces.push_back(testFace);

	testFace.nodes = { 3,2,6,7 };
	testMesh.faces.push_back(testFace);

	testFace.nodes = { 1,0,2,3 };
	testMesh.faces.push_back(testFace);

	testFace.nodes = { 7,6,4,5 };
	testMesh.faces.push_back(testFace);

	CIrregCell testCell;
	testCell.facesInd = { 0,1,2,3,4,5 };
	testMesh.cells.push_back(testCell);

	try
	{
		//testMesh.sayInfo(std::cout);
		testMesh.FindContactBorder(0, { 0,0,0 }, { 1,1,1 }, 0.52);
	}
	catch (const std::exception& e)
	{
		std::cout << e.what();
		exit(-1);
	}

	testMesh.sayInfo(std::cout);

}