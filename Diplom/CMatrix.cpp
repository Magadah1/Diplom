#include "CMatrix.h"

//void CMatrix4x4d::createRotationMatrix(double alpha, double betha, double gamma, bool isRadians) noexcept
//{
//	if (!isRadians) // если в градусах
//	{
//		double k = acos(-1) / 180.;
//
//		alpha *= k;
//		betha *= k;
//		gamma *= k;
//	}
//	setIdentityMatrix();
//
//	mat[0][0] = cos(betha) * cos(gamma); mat[0][1] = -sin(gamma) * cos(betha); mat[0][2] = sin(betha);
//	mat[1][0] = sin(alpha) * sin(betha) * sin(gamma) + sin(gamma) * cos(alpha); mat[1][1] = -sin(alpha) * sin(betha) * sin(gamma) + cos(alpha) * cos(gamma); mat[1][2] = -sin(alpha) * cos(betha);
//	mat[2][0] = sin(alpha) * sin(gamma) - sin(betha) * cos(alpha) * cos(gamma); mat[2][1] = sin(alpha) * cos(gamma) + sin(betha) * sin(gamma) * cos(alpha); mat[2][2] = cos(alpha) * cos(betha);
//}

void CMatrix4x4d::createRotationMatrix(CVector axis, double tetha, bool isRadians) noexcept
{
	setIdentityMatrix();
	if (!tetha)
		return;

	if (!isRadians)
		tetha *= acos(-1) / 180.;

	axis.Norm();

	mat[0][0] = cos(tetha) + (1 - cos(tetha)) * axis.X() * axis.X(); mat[0][1] = (1 - cos(tetha)) * axis.X() * axis.Y() - sin(tetha) * axis.Z(); mat[0][2] = (1 - cos(tetha)) * axis.X() * axis.Z() + sin(tetha) * axis.Y();
	mat[1][0] = (1 - cos(tetha)) * axis.Y() * axis.X() + sin(tetha) * axis.Z(); mat[1][1] = cos(tetha) + (1 - cos(tetha)) * axis.Y() * axis.Y(); mat[1][2] = (1 - cos(tetha)) * axis.Y() * axis.Z() - sin(tetha) * axis.X();
	mat[2][0] = (1 - cos(tetha)) * axis.Z() * axis.X() - sin(tetha) * axis.Y(); mat[2][1] = (1 - cos(tetha)) * axis.Z() * axis.Y() + sin(tetha) * axis.X(); mat[2][2] = cos(tetha) + (1 - cos(tetha)) * axis.Z() * axis.Z();
}

void CMatrix4x4d::createTransferMatrix(CVector movement) noexcept
{
	setIdentityMatrix();
	mat[0][3] = movement.X();
	mat[1][3] = movement.Y();
	mat[2][3] = movement.Z();
}
//
//void CMatrix4x4d::createRTMatrix(CVector movement, double alpha, double betha, double gamma, bool isRadians) noexcept
//{
//	createRotationMatrix(alpha, betha, gamma, isRadians);
//
//	mat[0][3] = movement.X();
//	mat[1][3] = movement.Y();
//	mat[2][3] = movement.Z();
//}

void CMatrix4x4d::createRTMatrix(CVector movement, CVector axis, double tetha, bool isRadians) noexcept
{
	createRotationMatrix(axis, tetha, isRadians);

	mat[0][3] = movement.X();
	mat[1][3] = movement.Y();
	mat[2][3] = movement.Z();
}

void CMatrix4x4d::setIdentityMatrix() noexcept
{
	for (size_t i = 0; i < M(); ++i)
		for (size_t j = 0; j < N(); ++j)
			mat[i][j] = i == j;
}

CPoint CMatrix4x4d::operator*(const CPoint& p) const noexcept
{
	CPoint res;

	res.X() = mat[0][0] * p.X() + mat[0][1] * p.Y() + mat[0][2] * p.Z() + mat[0][3];
	res.Y() = mat[1][0] * p.X() + mat[1][1] * p.Y() + mat[1][2] * p.Z() + mat[1][3];
	res.Z() = mat[2][0] * p.X() + mat[2][1] * p.Y() + mat[2][2] * p.Z() + mat[2][3];

	return res;
}

CMatrix4x4d::CMatrix4x4d() noexcept
	: CMatrix()
{
}
