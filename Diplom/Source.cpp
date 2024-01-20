#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>

#include "CIrregMesh.h"
#include "CMatrix.h"

#include "glut.h"

static CIrregMesh testMesh;
static CIrregMesh initialTestMesh;

static int wtd = -1; // -1 - ничего не рисуем. size() - рисуем все. Другое число - номер клетки, который рисуем.
static int gw = 1300, gh = 1300;

extern const double CalculationEps	= 1e-11; //16 //12	//11
extern const double EqualPointsEps	= 1e-11; //16 //11	//11
extern const double PlaneEps		= 1e-9; //13  //9	//9

void glVertex3f(const CPoint& p)
{
	glVertex3f(p.X(), p.Y(), p.Z());
}

void setRandColor(int i)
{
	srand(i * 123 + 1678862);
	int r = rand() % 256; double _r = r / 255.;
	int g = rand() % 256; double _g = g / 255.;
	int b = rand() % 256; double _b = b / 255.;
	glColor3f(_r, _g, _b);
}

void processNormalKeys(unsigned char key, int x, int y) 
{
	switch (key)
	{
	case 'a':
		glTranslatef(0, -1, 0);
		break;
	case 'd':
		glTranslatef(0, 1, 0);
		break;
	case 'w':
		glTranslatef(-1, 0, 0);
		break;
	case 's':
		glTranslatef(1, 0, 0);
		break;
	case 'q':
		glTranslatef(0, 0, -1);
		break;
	case 'e':
		glTranslatef(0, 0, 1);
	default:
		break;
	}
}

void processSpecialKeys(int key, int x, int y) {

	if (key == GLUT_KEY_LEFT)
	{
		if (wtd == -1)
			wtd = testMesh.cells.size();
		else
			--wtd;

	}
	else if (key == GLUT_KEY_RIGHT)
	{
		if (wtd == testMesh.cells.size())
			wtd = -1;
		else
			++wtd;
	}
	else if (key == GLUT_KEY_UP)
		glScalef(1.1, 1.1, 1.1);
	else if (key == GLUT_KEY_DOWN)
		glScalef(0.9, 0.9, 0.9);
}

void renderScene()
{
	glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	
	glLineWidth(3);
	if (wtd == testMesh.cells.size())
	{
		for (size_t i = 0; i < wtd; ++i)
		{
			const CIrregCell& cell = testMesh.cells[i];
			if (wtd == 1)
				glColor3f(88. / 255, 196. / 255, 221. / 255);
			else if (i == 0)
				glColor3f(0xFF / 255., 0x45 / 255., 0);
			else if (i == 1)
				glColor3f(0x6B / 255., 0xE4 / 255., 0);
			else if (i == 2)
				glColor3f(0x0A / 255., 0x64 / 255., 0xA4 / 255.);
			else
				setRandColor(i);
			
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

	}
	else if (wtd != -1)
	{
		const CIrregCell& cell = testMesh.cells[wtd];
		if (wtd == 0)
			glColor3f(0xFF / 255., 0x45 / 255., 0);
		else if (wtd == 1)
			glColor3f(0x6B / 255., 0xE4 / 255., 0);
		else if (wtd == 2)
			glColor3f(0x0A / 255., 0x64 / 255., 0xA4 / 255.);
		else
			setRandColor(wtd);

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

	glutSwapBuffers();
}

void changeSize(int w, int h) 
{
	gw = w; gh = h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0, 0, w, h);
	glOrtho(-3, 3, -3, 3, -3, 3);

	//glRotatef(-135, 1, 0, 0);			//2
	//glRotatef(75, 0, 1, 0); //3			//2
	//glRotatef(-35, 1, 0, 0);//3			//2

	glRotatef(-90, 1, 0, 0);//1			//2
	glRotatef(225, 0, 0,1);	//1			//2
	glRotatef(45, -1, 1, 0);//1			//2

	glMatrixMode(GL_MODELVIEW);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
}

int main(int argc, char** argv)
{
	setlocale(0, "");

	return 0;
	glutInit(&argc, argv);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(gw, gh);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("Отображение");
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);
	glutSpecialFunc(processSpecialKeys);
	glutKeyboardFunc(processNormalKeys);
	glutMainLoop();
}
