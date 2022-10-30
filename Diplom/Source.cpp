#include <iostream>

#include "CIrregMesh.h"

#include "glut.h"

static CIrregMesh testMesh;
static CIrregMesh initialTestMesh;

enum class WhatToDraw
{
	NOTHING,
	INITIAL,
	FIRST,
	SECOND,
	BOTH,
	COUNT // просто сколько их
};

static WhatToDraw wtd = WhatToDraw::INITIAL;

void glVertex3f(const CPoint& p)
{
	glVertex3f(p.X(), p.Y(), p.Z());
}

void processSpecialKeys(int key, int x, int y) {
	if (key == 27)
		exit(0);

	if (key == GLUT_KEY_LEFT)
	{
		if (wtd == WhatToDraw::NOTHING)
			wtd = WhatToDraw::BOTH;
		else
			wtd = static_cast<WhatToDraw>(static_cast<int>(wtd) - 1);
	}
	else if (key == GLUT_KEY_RIGHT)
	{
		if (wtd == WhatToDraw::BOTH)
			wtd = WhatToDraw::NOTHING;
		else
			wtd = static_cast<WhatToDraw>(static_cast<int>(wtd) + 1);
	}
}

void renderScene()
{
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLoadIdentity();

	glRotatef(-90, 1, 0, 0);
	glRotatef(225, 0, 0,1);
	glRotatef(45, -1, 1, 0);

	glBegin(GL_LINES);
	{
		glColor3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(2, 0, 0);

		glColor3f(0, 1, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 2, 0);

		glColor3f(0, 0, 1);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 2);
	}


	switch (wtd)
	{
	case WhatToDraw::NOTHING:
		// пусто
		break;
	case WhatToDraw::INITIAL:
	{

		glColor3f(1, 1, 1);
		
		const CIrregCell& cell = initialTestMesh.cells[0];
		for (size_t faceID = 0; faceID < cell.facesInd.size(); ++faceID)
		{
			const CIrregFace& face = initialTestMesh.faces[cell.facesInd[faceID]];
			glBegin(GL_LINE_LOOP);
			{
				for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
				{
					const int& nID = face.nodes[nodeId];
					glVertex3f(initialTestMesh.nodes[nID]);
				}
			}
			glEnd();
		}

	}
		break;
	case WhatToDraw::FIRST:
	{

		glColor3f(1, 0, 1);

		const CIrregCell& cell = testMesh.cells[0];
		for (size_t faceID = 0; faceID < cell.facesInd.size(); ++faceID)
		{
			const CIrregFace& face = testMesh.faces[cell.facesInd[faceID]];
			glBegin(GL_LINE_LOOP);
			{
				for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
				{
					const int& nID = face.nodes[nodeId];
					glVertex3f(testMesh.nodes[nID]);
				}
			}
			glEnd();
		}

	}
		break;
	case WhatToDraw::SECOND:
	{

		glColor3f(0, 1, 1);

		const CIrregCell& cell = testMesh.cells[1];
		for (size_t faceID = 0; faceID < cell.facesInd.size(); ++faceID)
		{
			const CIrregFace& face = testMesh.faces[cell.facesInd[faceID]];
			glBegin(GL_LINE_LOOP);
			{
				for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
				{
					const int& nID = face.nodes[nodeId];
					glVertex3f(testMesh.nodes[nID]);
				}
			}
			glEnd();
		}

	}
		break;
	case WhatToDraw::BOTH:
	{
		glColor3f(1, 0,	1);

		const CIrregCell& cell1 = testMesh.cells[0];
		for (size_t faceID = 0; faceID < cell1.facesInd.size(); ++faceID)
		{
			const CIrregFace& face = testMesh.faces[cell1.facesInd[faceID]];
			glBegin(GL_LINE_LOOP);
			{
				for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
				{
					const int& nID = face.nodes[nodeId];
					glVertex3f(testMesh.nodes[nID]);
				}
			}
			glEnd();
		}

		glColor3f(0, 1, 1);

		const CIrregCell& cell2 = testMesh.cells[1];
		for (size_t faceID = 0; faceID < cell2.facesInd.size(); ++faceID)
		{
			const CIrregFace& face = testMesh.faces[cell2.facesInd[faceID]];
			glBegin(GL_LINE_LOOP);
			{
				for (size_t nodeId = 0; nodeId < face.nodes.size(); ++nodeId)
				{
					const int& nID = face.nodes[nodeId];
					glVertex3f(testMesh.nodes[nID]);
				}
			}
			glEnd();
		}

	}
		break;
	case WhatToDraw::COUNT:
		// пусто
		break;
	default:
		break;
	}


	glutSwapBuffers();
}

void changeSize(int w, int h) 
{
	if (h == 0)
		h = 1;
	float ratio = 1.0 * w / h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0, 0, w, h);
	glOrtho(-2, 2, -2, 2, -2, 2);

	glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv)
{
	setlocale(0, "");
	// CIrregMesh testMesh;
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

	initialTestMesh = testMesh;

	try
	{
		testMesh.sayInfo(std::cout);
		testMesh.FindContactBorder(0, { 0,0,0 }, { 1,1,1 }, 0.5643);
		testMesh.sayInfo(std::cout);
	}
	catch (const std::exception& e)
	{
		std::cout << e.what();
		return -1;
	}

	glutInit(&argc, argv);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(1300, 900);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("Отображение");
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);
	glutSpecialFunc(processSpecialKeys);
	glutMainLoop();
}