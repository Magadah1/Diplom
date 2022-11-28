#include <iostream>
#include <chrono>

#include "CIrregMesh.h"
#include "CMatrix.h"

#include "glut.h"

static CIrregMesh testMesh;
static CIrregMesh initialTestMesh;

static int wtd = -1; // -1 - ничего не рисуем. size() - рисуем все. Другое число - номер клетки, который рисуем.
static int gw = 1300, gh = 1300;

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
}

void renderScene()
{
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glLoadIdentity();

	/*glRotatef(-90, 1, 0, 0);
	glRotatef(225, 0, 0,1);
	glRotatef(45, -1, 1, 0);*/

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
	glEnd();
	if (wtd == testMesh.cells.size())
	{
		for (size_t i = 0; i < wtd; ++i)
		{
			const CIrregCell& cell = testMesh.cells[i]; 
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
	glOrtho(-4, 4, -4, 4, -4, 4);

	glRotatef(-90, 1, 0, 0);
	glRotatef(225, 0, 0,1);
	glRotatef(45, -1, 1, 0);

	glMatrixMode(GL_MODELVIEW);
}

template <typename OStream>
void iteract(int cellNumber, CPoint start, CPoint end, double volume, OStream& OUT, bool log = false)
{
	if (log)
		OUT << "\n----Volume = " << volume;
	std::chrono::steady_clock::time_point startT = std::chrono::steady_clock::now();
	std::pair<int, int> res = testMesh.FindContactBorder(cellNumber, start, end, volume);
	std::chrono::steady_clock::time_point endT = std::chrono::steady_clock::now();
	std::chrono::microseconds timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(endT - startT);

	if (log)
		OUT << "\tIterations = " << res.second << " with in " << timeDiff.count() << " milliseconds with volume = " << testMesh.getCellVolume(testMesh.cells[cellNumber]);
}

int main(int argc, char** argv)
{
	setlocale(0, "");
	CMatrix4x4d m;
	
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

	testFace.nodes = { 0,4,6,2 };
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
	const double cellVolume = initialTestMesh.getCellVolume(initialTestMesh.cells[0]);

	testMesh.sayInfo(std::cout);

	constexpr size_t total = 1e3;
	size_t good{};
	for (size_t i = 0; i < total; ++i)
	{
		int randomSplitsCount = rand() % 100;
		for (int j = 0; j < randomSplitsCount; ++j)
			testMesh.spliteFaceByTriangles(rand() % testMesh.faces.size());

		m.createRTMatrix(
			{
			-123.12 + double(rand()) / RAND_MAX * 246.24,
			-123.12 + double(rand()) / RAND_MAX * 246.24,
			-123.12 + double(rand()) / RAND_MAX * 246.24
			},
			{
			-6 + double(rand()) / RAND_MAX * 12,
			-6 + double(rand()) / RAND_MAX * 12,
			-6 + double(rand()) / RAND_MAX * 12
			},
			rand() % 360,
			false
		);

		double randVolume = double(rand()) / RAND_MAX * cellVolume;
		try
		{
			for (size_t i = 0; i < testMesh.nodes.size(); ++i)
				testMesh.nodes[i] = m * testMesh.nodes[i];
			iteract(0, testMesh.nodes[0], testMesh.nodes[7], randVolume, std::cout);
			++good;
		}
		catch (const std::exception& e)
		{
			std::cout << e.what();
			std::cout << "RandV = " << randVolume << "\trsc = " << randomSplitsCount << "\tcellVolume = " << testMesh.getCellVolume(testMesh.cells[0]) << '\n';
		}


		testMesh = initialTestMesh;
	}
	std::cout << "Good = " << good << "\terrors = " << total - good << '\n';
	testMesh.sayInfo(std::cout);
	return -2;


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
	glutCreateWindow("Отображение");
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);
	glutSpecialFunc(processSpecialKeys);
	glutKeyboardFunc(processNormalKeys);
	glutMainLoop();
}