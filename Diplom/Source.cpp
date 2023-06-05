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

static int wtd = -1; // -1 - ������ �� ������. size() - ������ ���. ������ ����� - ����� ������, ������� ������.
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
	//glLoadIdentity();

	/*glRotatef(-90, 1, 0, 0);
	glRotatef(225, 0, 0,1);
	glRotatef(45, -1, 1, 0);*/

	/*glBegin(GL_LINES);
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
	glEnd();*/
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

std::chrono::microseconds avgTime{};
int avgIter{};
int total{};
int minIter = 100000;
int maxIter = -1;

template <typename OStream>
void iteract(int cellNumber, CPoint start, CPoint end, double volume, OStream& OUT, bool log = false, bool nextThrow = false)
{
	
	std::pair<int, int> res;
	try
	{
		std::chrono::steady_clock::time_point startT = std::chrono::steady_clock::now();
		res = testMesh.FindContactBorder(cellNumber, start, end, volume);
		std::chrono::steady_clock::time_point endT = std::chrono::steady_clock::now();
		std::chrono::microseconds timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(endT - startT);
		if (log)
			OUT << "\n----Volume = " << volume << "\tIterations = " << res.second << " with in " << timeDiff.count() << " microseconds with volume = " << testMesh.getCellVolume(testMesh.cells[cellNumber]) << '\n';
		if (res.second != 1 && res.second <= 100)
		{
			avgTime += timeDiff;
			avgIter += res.second;
			++total;

			if (res.second > maxIter) maxIter = res.second;
			if (res.second < minIter) minIter= res.second;
		}
	}
	catch (const std::exception& e)
	{
		const CException* exPtr = dynamic_cast<const CException*>(&e);
		if (exPtr)
		{
			OUT << '\n';
			OUT << exPtr->what();
			double vI = exPtr->getValueByKey("�������");
			double vF = exPtr->getValueByKey("���������");
			double d = abs(vI - vF);

			OUT.setf(std::ios::scientific);
			OUT << "������ ������� = " << d << '\n';
			OUT.unsetf(std::ios::scientific);
			OUT << '\n';
		}
		else
		{
			OUT << "\nError = " << e.what();
			OUT << "Volume = " << volume << '\n';
		}
		if (nextThrow)
			throw e;
	}

	
}

void stupidCOUT()
{
	testMesh.sayInfo(std::cout);
}

int main(int argc, char** argv)
{
	setlocale(0, "");
	return -1;
	/*for (int i = 0; i < 100; ++i)
		testMesh.addRandomFigure(CPoint(
			-100 + 200. * rand() / RAND_MAX,
			-100 + 200. * rand() / RAND_MAX,
			-100 + 200. * rand() / RAND_MAX));

	int startCellsCount = testMesh.cells.size();

	testMesh.setMode(CIrregMesh::Mode::OLD);

	for (int i = 0; i < startCellsCount; ++i)
	{
		const CIrregCell& cell = testMesh.cells[i];
		std::vector<int> nodes = testMesh.getCellNodesIds(i);

		int rN1 = nodes[0] + rand() % nodes.size();
		int rN2 = nodes[0] + rand() % nodes.size();

		if (rN1 == rN2)
			++rN2;

		rN2 = nodes[0] + (rN2 - nodes[0]) % nodes.size();

		const CSurfaceNode& node1 = testMesh.nodes[rN1];
		const CSurfaceNode& node2 = testMesh.nodes[rN2];

		CSideEquation plane(CVector(node1).createVector(node2).Norm());

		try
		{
			plane.recalcD(node1);
			double v1 = testMesh.getCutOffCellVolume(i, plane);

			plane.recalcD(node2);
			double v2 = testMesh.getCutOffCellVolume(i, plane);

			double randV = v1 + (v2 - v1) * rand() / RAND_MAX;


			iteract(i, testMesh.nodes[rN1], testMesh.nodes[rN2], randV, std::cout, true);
		}
		catch (const std::exception& e)
		{
			std::cout << e.what();
		}
		
	}
	return -1;*/


	// �����
	//testMesh.nodes.push_back({ 0,0,0 }); //0
	//testMesh.nodes.push_back({ 0,0,1 }); //1
	//testMesh.nodes.push_back({ 0,1,0 }); //2
	//testMesh.nodes.push_back({ 0,1,1 }); //3
	//testMesh.nodes.push_back({ 1,0,0 }); //4
	//testMesh.nodes.push_back({ 1,0,1 }); //5 
	//testMesh.nodes.push_back({ 1,1,0 }); //6
	//testMesh.nodes.push_back({ 1,1,1 }); //7
	//CIrregFace testFace;
	//testFace.cell1 = 0;
	//testFace.cell2 = -1;

	//testFace.nodes = { 0,4,6,2 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,3,7,5 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,5,4,0 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 3,2,6,7 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,0,2,3 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 7,6,4,5 };
	//testMesh.faces.push_back(testFace);

	//CIrregCell testCell;
	//testCell.facesInd = { 0,1,2,3,4,5 };

	//testMesh.cells.push_back(testCell);

	//initialTestMesh = testMesh;
	//srand(static_cast<unsigned>(time(nullptr)));

	//for (size_t i = 0; i != 2; ++i)
	//{

	//	int N1 = rand() % testMesh.nodes.size();
	//	int N2 = rand() % testMesh.nodes.size();

	//	if (N1 == N2)
	//		N2 = (N2 + 1) % testMesh.nodes.size();

	//	CSurfaceNode& n1 = testMesh.nodes[N1];
	//	CSurfaceNode& n2 = testMesh.nodes[N2];
	//	CSideEquation side(CVector(n1).createVector(n2));

	//	side.recalcD(n1);
	//	double v1 = testMesh.getCutOffCellVolume(0, side);
	//	side.recalcD(n2);
	//	double v2 = testMesh.getCutOffCellVolume(0, side);
	//	double rv = v1 + (v2 - v1) * rand() / RAND_MAX;
	//	iteract(0, n1, n2, rv, std::cout, true);
	//};

	
	/*const double cV = testMesh.getCellVolume(testMesh.cells[0]);
	for (size_t i = 0; i != 1000; ++i)
	{
		int N1 = rand() % testMesh.nodes.size();
		int N2 = rand() % testMesh.nodes.size();

		if (N1 == N2)
			N2 = (N2 + 1) % testMesh.nodes.size();

		double rV = cV * rand() / RAND_MAX;
		iteract(0, testMesh.nodes[N1], testMesh.nodes[N2], rV, std::cout, true);

		testMesh = initialTestMesh;
	}*/

	//std::cout << "\n\nAvg:\nTotal = " << total << "\tAvgIter = " << avgIter / total << "\tAvgTime = " << avgTime.count() / total 
	//	<< "\tRange = (" << minIter << " - " << maxIter << ')';
	//return 21;
	
	// ���� ��� ������
	//testMesh.nodes.push_back({ 0,0,0 });	//0
	//testMesh.nodes.push_back({ 1,0,0 });	//1
	//testMesh.nodes.push_back({ 5,4,0 });	//2
	//testMesh.nodes.push_back({ 5,5,0 });	//3
	//testMesh.nodes.push_back({ 4,5,0 });	//4
	//testMesh.nodes.push_back({ 0,1,0 });	//5

	//testMesh.nodes.push_back({ 0,0,3 });	//6
	//testMesh.nodes.push_back({ 1,0,3 });	//7
	//testMesh.nodes.push_back({ 5,4,3 });	//8
	//testMesh.nodes.push_back({ 5,5,3 });	//9
	//testMesh.nodes.push_back({ 4,5,3 });	//10
	//testMesh.nodes.push_back({ 0,1,3 });	//11
	//
	//CIrregFace tFace;
	//tFace.cell1 = 0;
	//tFace.cell2 = -1;

	//tFace.nodes = { 0, 1, 2, 3, 4, 5 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 0, 6, 7, 1 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 11, 6, 0, 5 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 1, 7, 8, 2 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 9, 3, 2, 8};
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 9, 10, 4, 3 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 10, 11, 5, 4};
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 6, 11, 10, 9, 8, 7};
	//testMesh.faces.push_back(tFace);

	//CIrregCell tCell;
	//tCell.facesInd = { 0,1,2,3,4,5,6,7 };

	//testMesh.cells.push_back(tCell);
	//
	//for (size_t i = 0; i != testMesh.nodes.size(); ++i)
	//{
	//	CSurfaceNode& node = testMesh.nodes[i];
	//	node.likeCPoint() -= CPoint(2.5, 2.5, 2.5);
	//}

	//initialTestMesh = testMesh;
	/*srand(static_cast<unsigned>(time(nullptr)));
	int N1 = rand() % testMesh.nodes.size();
	int N2 = rand() % testMesh.nodes.size();

	if (N1 == N2)
		N2 = (N2 + 1) % testMesh.nodes.size();

	CSurfaceNode& n1 = testMesh.nodes[N1];
	CSurfaceNode& n2 = testMesh.nodes[N2];
	CSideEquation side(CVector(n1).createVector(n2));

	side.recalcD(n1);
	double v1 = testMesh.getCutOffCellVolume(0, side);
	side.recalcD(n2);
	double v2 = testMesh.getCutOffCellVolume(0, side);
	double rv = v1 + (v2 - v1) * rand() / RAND_MAX;
	iteract(0, n1, n2, rv, std::cout, true);*/
	/*for (size_t i = 0; i != 1000; ++i)
	{
		int N1 = rand() % testMesh.nodes.size();
		int N2 = rand() % testMesh.nodes.size();

		if (N1 == N2)
			N2 = (N2 + 1) % testMesh.nodes.size();

		CSurfaceNode& n1 = testMesh.nodes[N1];
		CSurfaceNode& n2 = testMesh.nodes[N2];
		CSideEquation side(CVector(n1).createVector(n2));

		side.recalcD(n1);
		double v1 = testMesh.getCutOffCellVolume(0, side);
		side.recalcD(n2);
		double v2 = testMesh.getCutOffCellVolume(0, side);
		double rv = v1 + (v2 - v1) * rand() / RAND_MAX;
		iteract(0, n1, n2, rv, std::cout, true);

		testMesh = initialTestMesh;
	}*/

	/*std::cout << "\n\nAvg:\nTotal = " << total << "\tAvgIter = " << avgIter / total << "\tAvgTime = " << avgTime.count() / total 
		<< "\tRange = (" << minIter << " - " << maxIter << ')';
	return 21;*/

	// 8 �������� �����
	//testMesh.nodes.push_back({ 0,0,4 });		//0
	//testMesh.nodes.push_back({ -3,0,2 });		//1
	//testMesh.nodes.push_back({ -2.5,-2,2 });	//2
	//testMesh.nodes.push_back({ 0,-3,2 });		//3
	//testMesh.nodes.push_back({ 2.5,-2,2 });		//4
	//testMesh.nodes.push_back({ 3,0,2 });		//5
	//testMesh.nodes.push_back({ 2.5,2,2 });		//6
	//testMesh.nodes.push_back({ 0,3,2 });		//7
	//testMesh.nodes.push_back({ -2.5,2,2 });		//8
	//testMesh.nodes.push_back({ -3,0,-2 });		//9
	//testMesh.nodes.push_back({ -2.5,-2,-2 });	//10
	//testMesh.nodes.push_back({ 0,-3,-2 });		//11
	//testMesh.nodes.push_back({ 2.5,-2,-2 });	//12
	//testMesh.nodes.push_back({ 3,0,-2 });		//13
	//testMesh.nodes.push_back({ 2.5,2,-2 });		//14
	//testMesh.nodes.push_back({ 0,3,-2 });		//15
	//testMesh.nodes.push_back({ -2.5,2,-2 });	//16
	//testMesh.nodes.push_back({ 0,0,-4 });		//17

	//CIrregFace tFace;
	//tFace.cell1 = -1;
	//tFace.cell2 = 0;

	//tFace.nodes = { 1,2,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 2,3,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 3,4,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 4,5,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 5,6,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 6,7,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 7,8,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 8,1,0 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 1,9,10,2 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 2,10,11,3 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 3,11,12,4 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 4,12,13,5 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 5,13,14,6 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 6,14,15,7 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 7,15,16,8 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 8,16,9,1 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 9,17,10 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 10,17,11 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 11,17,12 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 12,17,13 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 13,17,14 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 14,17,15 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 15,17,16 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 16,17,9 };
	//testMesh.faces.push_back(tFace);

	//CIrregCell tCell;
	//tCell.facesInd = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23 };

	//testMesh.cells.push_back(tCell);

	//initialTestMesh = testMesh;
	//srand(static_cast<unsigned>(time(nullptr)));
	//int N1 = rand() % testMesh.nodes.size();
	//int N2 = rand() % testMesh.nodes.size();

	//if (N1 == N2)
	//	N2 = (N2 + 1) % testMesh.nodes.size();

	//CSurfaceNode& n1 = testMesh.nodes[N1];
	//CSurfaceNode& n2 = testMesh.nodes[N2];
	//CSideEquation side(CVector(n1).createVector(n2));

	//side.recalcD(n1);
	//double v1 = testMesh.getCutOffCellVolume(0, side);
	//side.recalcD(n2);
	//double v2 = testMesh.getCutOffCellVolume(0, side);
	//double rv = v1 + (v2 - v1) * rand() / RAND_MAX;
	//iteract(0, n1, n2, rv, std::cout, true);
	/*for (size_t i = 0; i != 1000; ++i)
	{
		int N1 = rand() % testMesh.nodes.size();
		int N2 = rand() % testMesh.nodes.size();

		if (N1 == N2)
			N2 = (N2 + 1) % testMesh.nodes.size();

		CSurfaceNode& n1 = testMesh.nodes[N1];
		CSurfaceNode& n2 = testMesh.nodes[N2];
		CSideEquation side(CVector(n1).createVector(n2));

		side.recalcD(n1);
		double v1 = testMesh.getCutOffCellVolume(0, side);
		side.recalcD(n2);
		double v2 = testMesh.getCutOffCellVolume(0, side);
		double rv = v1 + (v2 - v1) * rand() / RAND_MAX;
		iteract(0, n1, n2, rv, std::cout, true);

		testMesh = initialTestMesh;
	}


	std::cout << "\n\nAvg:\nTotal = " << total << "\tAvgIter = " << avgIter / total << "\tAvgTime = " << avgTime.count() / total 
		<< "\tRange = (" << minIter << " - " << maxIter << ')';
	return 21;*/


	//constexpr int total = 1e5;
	//int gOld{}, gNew{}, gNewM{}, OldCan{}, NewCan{};
	//const double v = testMesh.getCellVolume(testMesh.cells[0]);
	//for (size_t i = 0; i < total; ++i)
	//{
	//	int rn1 = rand() % testMesh.nodes.size();
	//	int rn2 = rand() % testMesh.nodes.size();

	//	if (rn2 == rn1)
	//		rn2 = (rn2 + 1) % testMesh.nodes.size();

	//	double rv = static_cast<double>(rand()) / RAND_MAX * v;
	//	int* ItOld = nullptr, * ItNew = nullptr, *ItNewM = nullptr;

	//	try
	//	{
	//		testMesh.setMode(CIrregMesh::Mode::OLD);
	//		auto rOld = testMesh.FindContactBorder(0, testMesh.nodes[rn1], testMesh.nodes[rn2], rv);
	//		++gOld;
	//		ItOld = new int(rOld.second);
	//	}
	//	catch (const std::exception& e)
	//	{
	//		std::cout << e.what();
	//	}
	//	testMesh = initialTestMesh;
	//	try
	//	{
	//		testMesh.setMode(CIrregMesh::Mode::NEW);
	//		auto rNew = testMesh.FindContactBorder(0, testMesh.nodes[rn1], testMesh.nodes[rn2], rv);
	//		++gNew;
	//		ItNew = new int(rNew.second);
	//	}
	//	catch (const std::exception& e)
	//	{
	//		std::cout << e.what();
	//	}
	//	/*testMesh = initialTestMesh;
	//	try
	//	{
	//		testMesh.setMode(CIrregMesh::Mode::NEWMODIFIED);
	//		auto rNewM = testMesh.FindContactBorder(0, testMesh.nodes[rn1], testMesh.nodes[rn2], rv);
	//		++gNewM;
	//		ItNewM = new int(rNewM.second);
	//	}
	//	catch (const std::exception& e)
	//	{

	//	}*/
	//	std::cout << "Iterations:\tOld = " << (ItOld ? std::to_string(*ItOld) : "none")
	//		<< "\tNew = " << (ItNew ? std::to_string(*ItNew) : "none")
	//		<< "\tNewModified = " << (ItNewM ? std::to_string(*ItNewM) : "none")
	//		<< "\tDifNew = " << (ItOld && ItNew ? std::to_string(*ItOld - *ItNew) : "none")
	//		<< "\tDifNewModidied = " << (ItOld && ItNewM ? std::to_string(*ItOld - *ItNewM) : "none") << '\n';

	//	if (ItOld && !ItNew)
	//		++OldCan;

	//	if (!ItOld && ItNew)
	//		++NewCan;

	//	if (ItOld)
	//		delete ItOld;
	//	if (ItNew)
	//		delete ItNew;
	//	if (ItNewM)
	//		delete ItNewM;
	//	testMesh = initialTestMesh;
	//}
	//std::cout << "\nTotal goods:\tOld = " << gOld << "\tNew = " << gNew << "\tNewModified = " << gNewM;
	//std::cout << "\nTimes, when Old solved = " << OldCan << "\tNew solved = " << NewCan << '\n';
	////std::cout << "Total =" << total << "\tGood =" << gOld;
	//return -1;

	//testMesh.nodes.push_back({ 0,0,0 });	//0
	//testMesh.nodes.push_back({ 1,0,0 });	//1
	//testMesh.nodes.push_back({ 5,4,0 });	//2
	//testMesh.nodes.push_back({ 5,5,0 });	//3
	//testMesh.nodes.push_back({ 4,5,0 });	//4
	//testMesh.nodes.push_back({ 0,1,0 });	//5

	//testMesh.nodes.push_back({ 0,0,3 });	//6
	//testMesh.nodes.push_back({ 1,0,3 });	//7
	//testMesh.nodes.push_back({ 5,4,3 });	//8
	//testMesh.nodes.push_back({ 5,5,3 });	//9
	//testMesh.nodes.push_back({ 4,5,3 });	//10
	//testMesh.nodes.push_back({ 0,1,3 });	//11
	//
	//CIrregFace tFace;
	//tFace.cell1 = 0;
	//tFace.cell2 = -1;

	//tFace.nodes = { 0, 1, 2, 3, 4, 5 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 0, 6, 7, 1 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 11, 6, 0, 5 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 1, 7, 8, 2 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 9, 3, 2, 8};
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 9, 10, 4, 3 };
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 10, 11, 5, 4};
	//testMesh.faces.push_back(tFace);

	//tFace.nodes = { 6, 11, 10, 9, 8, 7};
	//testMesh.faces.push_back(tFace);

	//CIrregCell tCell;
	//tCell.facesInd = { 0,1,2,3,4,5,6,7 };

	//testMesh.cells.push_back(tCell);

	//initialTestMesh = testMesh;

	//constexpr int total = 1e1;
	//int good{};
	//const double cV = testMesh.getCellVolume(testMesh.cells[0]);
	//CMatrix4x4d m;
	//m.setIdentityMatrix();
	//for (size_t i = 0; i < total; ++i)
	//{
	//	const double vTF = static_cast<double>(rand()) / RAND_MAX * cV;
	//	m.createRTMatrix(
	//		{
	//			-5 + double(rand()) / RAND_MAX * 10,
	//			-5 + double(rand()) / RAND_MAX * 10,
	//			-5 + double(rand()) / RAND_MAX * 10
	//		},
	//		{
	//			-1 + double(rand()) / RAND_MAX * 2,
	//			-1 + double(rand()) / RAND_MAX * 2,
	//			-1 + double(rand()) / RAND_MAX * 2
	//		},
	//		rand() % 360,
	//		false
	//	);
	//	m.rotateMesh(testMesh);

	//	try
	//	{
	//		iteract(0, testMesh.nodes[0], testMesh.nodes[9], vTF, std::cout);
	//		++good;
	//	}
	//	catch (const std::exception& e)
	//	{

	//	}

	//	testMesh = initialTestMesh;
	//}
	//std::cout << "\nTotal = " << total << "\tGood = " << good;
	//return 0;

	//CMatrix4x4d m;
	//m.setIdentityMatrix();
	//
	//testMesh.nodes.push_back({ 0,0,0 }); //0
	//testMesh.nodes.push_back({ 0,0,1 }); //1
	//testMesh.nodes.push_back({ 0,1,0 }); //2
	//testMesh.nodes.push_back({ 0,1,1 }); //3
	//testMesh.nodes.push_back({ 1,0,0 }); //4
	//testMesh.nodes.push_back({ 1,0,1 }); //5 
	//testMesh.nodes.push_back({ 1,1,0 }); //6
	//testMesh.nodes.push_back({ 1,1,1 }); //7
	//CIrregFace testFace;
	//testFace.cell1 = 0;
	//testFace.cell2 = -1;

	//testFace.nodes = { 0,4,6,2 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,3,7,5 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,5,4,0 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 3,2,6,7 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,0,2,3 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 7,6,4,5 };
	//testMesh.faces.push_back(testFace);

	//CIrregCell testCell;
	//testCell.facesInd = { 0,1,2,3,4,5 };

	//testMesh.cells.push_back(testCell);

	//initialTestMesh = testMesh;
	//const double CellV = testMesh.getCellVolume(testMesh.cells[0]);

	//constexpr int total = 1e5;
	//int g{};
	//int rndG{};
	//int rndV{};
	//double rndVolume{};
	//srand(100);
	//for (size_t i = 0; i < total; ++i)
	//{
	//	try
	//	{
	//		/*m.createTransferMatrix(
	//			{
	//				-5 + double(rand()) / RAND_MAX * 10,
	//				-5 + double(rand()) / RAND_MAX * 10,
	//				-5 + double(rand()) / RAND_MAX * 10
	//			}
	//		);*/

	//		/*m.createRotationMatrix(
	//			{
	//				-1 + double(rand()) / RAND_MAX * 2,
	//				-1 + double(rand()) / RAND_MAX * 2,
	//				-1 + double(rand()) / RAND_MAX * 2
	//			},
	//			rand() % 360,
	//			false
	//		);*/

	//		m.createRTMatrix(
	//			{
	//				-5 + double(rand()) / RAND_MAX * 10,
	//				-5 + double(rand()) / RAND_MAX * 10,
	//				-5 + double(rand()) / RAND_MAX * 10
	//			},
	//			{
	//				-1 + double(rand()) / RAND_MAX * 2,
	//				-1 + double(rand()) / RAND_MAX * 2,
	//				-1 + double(rand()) / RAND_MAX * 2
	//			},
	//			rand() % 360,
	//			false
	//		);

	//		for (size_t j = 0; j < testMesh.nodes.size(); ++j)
	//			testMesh.nodes[j] = m * initialTestMesh.nodes[j];
	//		
	//		rndG = rand() % testMesh.faces.size();
	//		testMesh.spliteFaceByTriangles(rndG); // 0
	//		//testMesh.spliteFaceByTriangles(rand() % testMesh.faces.size());
	//		rndV = rand() % 7 + 1;
	//		rndVolume = CellV * double(rand()) / RAND_MAX;
	//		iteract(0, testMesh.nodes[0], testMesh.nodes[rndV], rndVolume, std::cout, false);

	//		//testMesh = initialTestMesh;
	//		++g;
	//	}
	//	catch (const std::exception&)
	//	{
	//		std::cout.setf(std::ios::scientific);
	//		std::cout << "Face =" << rndG << "\t Node =" << rndV << "\t VolumeToFind =" << rndVolume << "\t Dif =" << abs(CellV - rndVolume) << "\t I =" << i << '\n';
	//		std::cout.unsetf(std::ios::scientific);
	//	}
	//	testMesh = initialTestMesh;
	//}
	//std::cout << "\n---Good=" << g << " of " << total << "---\n";
	//testMesh.spliteFaceByTriangles(5);
	//return -223;
	//std::ofstream OUT("out.txt");

	//// ����� �� 3 "�������" ���������������.
	//iteract(0, testMesh.nodes[0], testMesh.nodes[4], 1. / 3, std::cout, true);
	//iteract(1, testMesh.nodes[0], testMesh.nodes[4], 1. / 3, std::cout, true);
	//
	//// �������� 1/3 �� "�������" ����������������.
	//iteract(0, testMesh.nodes[0], testMesh.nodes[1], 1. / 9, std::cout, true);
	//iteract(1, testMesh.nodes[0], testMesh.nodes[1], 1. / 9, std::cout, true);
	//iteract(2, testMesh.nodes[0], testMesh.nodes[1], 1. / 9, std::cout, true);
	//
	//// ����� ���������� 2/3 �� �������. �� ����� �������� "�������" ���������������.
	//iteract(3, testMesh.nodes[0], testMesh.nodes[1], 1. / 9, std::cout, true);
	//iteract(4, testMesh.nodes[0], testMesh.nodes[1], 1. / 9, std::cout, true);
	//iteract(5, testMesh.nodes[0], testMesh.nodes[1], 1. / 9, std::cout, true);
	//
	//// ����� ������ "�������" ��������������� �� ������ ������.
	//iteract(0, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(1, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(2, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(0, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//iteract(1, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//iteract(2, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//
	//// ����� ������� "�������" ��������������� �� ������ ������.
	//iteract(3, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(4, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(5, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(3, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//iteract(4, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//iteract(5, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//
	//// ����� ������� "�������" ��������������� �� ������ ������.
	//iteract(6, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(7, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(8, testMesh.nodes[0], testMesh.nodes[2], 2. / 27, std::cout, true);
	//iteract(6, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//iteract(7, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);
	//iteract(8, testMesh.nodes[0], testMesh.nodes[2], 1. / 27, std::cout, true);

	//std::cout << "Total cells = " << testMesh.cells.size() << "\tTotal faces = " << testMesh.faces.size() << "\tTotal nodes = " << testMesh.nodes.size();
	////return -1;

	//std::thread t(stupidCOUT);

	//testMesh.sayInfo(OUT);

	//OUT.close();
	//system("start out.txt");

	/*int maxN = 0;
	for (const auto& el : testMesh.faces)
	if (el.nodes.size() > maxN)
	maxN = el.nodes.size();

	int maxF = 0;
	for (const auto& el : testMesh.cells)
	if (el.facesInd.size() > maxF)
	maxF = el.facesInd.size();


	for (size_t i = 0; i < testMesh.faces.size(); ++i)
	{
	const CIrregFace& f = testMesh.faces[i];
	if (maxN == f.nodes.size())
	{
	std::cout << i << '\t' << maxN << '\n';
	}
	}
	std::cout << '\n';
	for (size_t i = 0; i < testMesh.cells.size(); ++i)
	{
	const CIrregCell& c = testMesh.cells[i];
	if (maxF == c.facesInd.size())
	{
	std::cout << i << '\t' << maxF << '\n';
	}
	}*/

	/*if (maxF > 6 || maxN > 4)
	testMesh.sayInfo(std::cout);
	return -3;*/

	//testMesh.FindContactBorder(19, testMesh.nodes[0], testMesh.nodes[1], 1. / 54);

	//std::thread t(stupidCOUT);

	//testMesh.sayInfo(std::cout);

	// �� ��������! ����� ������ �������� ��������
	/*const double cellVolume = initialTestMesh.getCellVolume(initialTestMesh.cells[0]);

	testMesh.sayInfo(std::cout);

	srand(1234512);

	for (size_t i = 0; i < 2; ++i)
	testMesh.spliteFaceByTriangles(rand() % testMesh.faces.size());

	iteract(0, testMesh.nodes[0], testMesh.nodes[7], 0.52, std::cout, true);

	std::thread t(stupidCOUT);*/
	//testMesh.sayInfo(std::cout);

	// �� ��������! ����� ����� ������ � ���������...
	//constexpr size_t total = 1e1;
	//const double cellVolume = initialTestMesh.getCellVolume(initialTestMesh.cells[0]);
	//size_t good{};
	//for (size_t i = 0; i < total; ++i)
	//{
	//	/*int randomSplitsCount = rand() % 100;
	//	for (int j = 0; j < randomSplitsCount; ++j)
	//		testMesh.spliteFaceByTriangles(rand() % testMesh.faces.size());*/

	//	m.createRTMatrix(
	//		{
	//		-123.12 + double(rand()) / RAND_MAX * 246.24,
	//		-123.12 + double(rand()) / RAND_MAX * 246.24,
	//		-123.12 + double(rand()) / RAND_MAX * 246.24
	//		},
	//		{
	//		-6 + double(rand()) / RAND_MAX * 12,
	//		-6 + double(rand()) / RAND_MAX * 12,
	//		-6 + double(rand()) / RAND_MAX * 12
	//		},
	//		rand() % 360,
	//		false
	//	);

	//	double randVolume = double(rand()) / RAND_MAX * cellVolume;
	//	try
	//	{
	//		for (size_t i = 0; i < testMesh.nodes.size(); ++i)
	//			testMesh.nodes[i] = m * testMesh.nodes[i];
	//		iteract(0, testMesh.nodes[0], testMesh.nodes[7], randVolume, std::cout);
	//		++good;
	//	}
	//	catch (const std::exception& e)
	//	{
	//		std::cout << e.what();
	//		std::cout << "RandV = " << randVolume << "\trsc = " << /*randomSplitsCount*/ 0 << "\tcellVolume = " << testMesh.getCellVolume(testMesh.cells[0]) << '\n';
	//	}


	//	testMesh = initialTestMesh;
	//}
	//std::cout << "Good = " << good << "\terrors = " << total - good << '\n';
	//std::thread t(stupidCOUT); 
	//return -2;


	// CIrregMesh testMesh;
	//testMesh.nodes.push_back({ 0,0,0 }); //0
	//testMesh.nodes.push_back({ 0,0,1 }); //1
	//testMesh.nodes.push_back({ 0,1,0 }); //2
	//testMesh.nodes.push_back({ 0,1,1 }); //3
	//testMesh.nodes.push_back({ 1,0,0 }); //4
	//testMesh.nodes.push_back({ 1,0,1 }); //5 
	//testMesh.nodes.push_back({ 1,1,0 }); //6
	//testMesh.nodes.push_back({ 1,1,1 }); //7
	//testMesh.nodes.push_back({ 1,0.5,0.5 }); //8
	//testMesh.nodes.push_back({ 1,0.5,1 }); //9
	//testMesh.nodes.push_back({ 1,1,0.5 }); //10
	//testMesh.nodes.push_back({ 1,0.5,0 }); //11
	//testMesh.nodes.push_back({ 1,0,0.5 }); //12


	//CIrregFace testFace;
	//testFace.cell1 = 0;
	//testFace.cell2 = -1;
	//
	//testFace.nodes = { 0,4,11,6,2 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,3,7,9,5 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,5,12,4,0 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 3,2,6,10,7 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 1,0,2,3 };
	//testMesh.faces.push_back(testFace);

	////testFace.nodes = { 7,6,4,5 };
	////testMesh.faces.push_back(testFace);

	//testFace.nodes = { 5,9,8,12 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 9,7,8 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 7,10,8 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 8,10,6,11 };
	//testMesh.faces.push_back(testFace);

	//testFace.nodes = { 12,8,11,4 };
	//testMesh.faces.push_back(testFace);

	//CIrregCell testCell;
	//testCell.facesInd = { 0,1,2,3,4,5,6,7,8,9 };
	//testMesh.cells.push_back(testCell);

	/*initialTestMesh = testMesh;

	testMesh.sayInfo(std::cout);*/



	//testMesh.sayInfo(std::cout);

	/*try
	{
	m.createRTMatrix({
	-5 + double(rand()) / RAND_MAX * 10,
	-5 + double(rand()) / RAND_MAX * 10,
	-5 + double(rand()) / RAND_MAX * 10
	},
	{
	-1 + double(rand()) / RAND_MAX * 2,
	-1 + double(rand()) / RAND_MAX * 2,
	-1 + double(rand()) / RAND_MAX * 2
	},
	rand() % 360,
	false
	);

	for (size_t i = 0; i < testMesh.nodes.size(); ++i)
	testMesh.nodes[i] = m * testMesh.nodes[i];
	iteract(0, testMesh.nodes[0], testMesh.nodes[7], double(rand()) / RAND_MAX, std::cout);
	}
	catch (const std::exception& e)
	{
	std::cout << e.what();
	}*/

	//double minV = 2, rv;

	//size_t countGoods{};
	//std::chrono::steady_clock::time_point startT = std::chrono::steady_clock::now();
	//constexpr size_t TOTAL = 1e6;
	//for (size_t j = 0; j < TOTAL; ++j)
	//	try
	//	{
	//		/*m.createTransferMatrix({ 
	//			-5 + double(rand()) / RAND_MAX * 10,
	//			-5 + double(rand()) / RAND_MAX * 10,
	//			-5 + double(rand()) / RAND_MAX * 10
	//			});*/

	//		/*m.createRotationMatrix(
	//			{
	//			-1 + double(rand()) / RAND_MAX * 2,
	//			- 1 + double(rand()) / RAND_MAX * 2,
	//			- 1 + double(rand()) / RAND_MAX * 2
	//			}, rand() % 360, false
	//		);*/
	//		
	//		m.createRTMatrix({
	//			-123.12 + double(rand()) / RAND_MAX * 246.24,
	//			-123.12 + double(rand()) / RAND_MAX * 246.24,
	//			-123.12 + double(rand()) / RAND_MAX * 246.24
	//			},
	//			{
	//			-6 + double(rand()) / RAND_MAX * 12,
	//			-6 + double(rand()) / RAND_MAX * 12,
	//			-6 + double(rand()) / RAND_MAX * 12
	//			},
	//			rand() % 360,
	//			false
	//		);

	//		//m.setIdentityMatrix();

	//		testMesh = initialTestMesh;
	//		for (size_t i = 0; i < testMesh.nodes.size(); ++i)
	//			testMesh.nodes[i] = m * testMesh.nodes[i];

	//		/*getResult = testMesh.FindContactBorder(0, { 1,0,0 }, { 1,1,1 }, 0.5);
	//		std::cout << "\n\n-----------------------------Iterations = " << getResult.second << " -----------------------------\n\n";
	//		getResult = testMesh.FindContactBorder(0, { 1,0,0 }, { 1,1,1 }, 0.2);
	//		std::cout << "\n\n-----------------------------Iterations = " << getResult.second << " -----------------------------\n\n";*/
	//		/*for (int i = 40; i > -1; --i)
	//		{
	//			getResult = testMesh.FindContactBorder(40 - i, { 0,0,0 }, { 1,1,1 }, 0.02);
	//			std::cout << "\n\n-----------------------------Iterations = " << getResult.second << " -----------------------------\n\n";
	//		}*/

	//	rv = double(rand()) / RAND_MAX;
	//		iteract(0, testMesh.nodes[0], testMesh.nodes[7], rv, std::cout);
	//		//testMesh = initialTestMesh;
	//		++countGoods;
	//		//testMesh.sayInfo(std::cout);
	//	}
	//	catch (const std::exception& e)
	//	{
	//		double v = testMesh.getCellVolume(testMesh.cells[0]);
	//		if (v < minV)
	//			minV = v;
	//		std::cout << e.what();
	//		//return -1;
	//	}
	//	std::chrono::steady_clock::time_point endT = std::chrono::steady_clock::now();
	//	std::chrono::seconds timeDiff = std::chrono::duration_cast<std::chrono::seconds>(endT - startT);

	//	std::cout << "\nTotal time = " << timeDiff.count() << " with " << countGoods << " succesfull tries of " << TOTAL << " total tries!\nMinV = " << minV;

	//	return -1;

	glutInit(&argc, argv);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(gw, gh);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("�����������");
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);
	glutSpecialFunc(processSpecialKeys);
	glutKeyboardFunc(processNormalKeys);
	glutMainLoop();
	//t.join();
}
