#pragma once
#include <array>
#include "CIrregMesh.h"

template<typename elementType, size_t m, size_t n>
class CMatrix
{
public:
	CMatrix() noexcept;

	size_t N() const noexcept;
	size_t M() const noexcept;

	elementType& operator()(const size_t& mPos, const size_t& nPos);
	const elementType& operator()(const size_t& mPos, const size_t& nPos) const;

	void rotateMesh(CIrregMesh& mesh) const noexcept;

protected:
	std::array<std::array<elementType, n>, m> mat{};
};

template<typename elementType, size_t m, size_t n>
inline CMatrix<elementType, m, n>::CMatrix() noexcept
{
}

template<typename elementType, size_t m, size_t n>
inline size_t CMatrix<elementType, m, n>::N() const noexcept
{
	return m;
}

template<typename elementType, size_t m, size_t n>
inline size_t CMatrix<elementType, m, n>::M() const noexcept
{
	return n;
}

template<typename elementType, size_t m, size_t n>
inline elementType& CMatrix<elementType, m, n>::operator()(const size_t& mPos, const size_t& nPos)
{
	return mat[mPos][nPos];
}

template<typename elementType, size_t m, size_t n>
inline const elementType& CMatrix<elementType, m, n>::operator()(const size_t& mPos, const size_t& nPos) const
{
	return mat[mPos][nPos];
}

template<typename elementType, size_t m, size_t n>
inline void CMatrix<elementType, m, n>::rotateMesh(CIrregMesh& mesh) const noexcept
{
	for (size_t pInd = 0; pInd < mesh.nodes.size(); ++pInd)
	{
		CPoint& p = mesh.nodes[pInd];
		CPoint res;

		res.X() = mat[0][0] * p.X() + mat[0][1] * p.Y() + mat[0][2] * p.Z() + mat[0][3];
		res.Y() = mat[1][0] * p.X() + mat[1][1] * p.Y() + mat[1][2] * p.Z() + mat[1][3];
		res.Z() = mat[2][0] * p.X() + mat[2][1] * p.Y() + mat[2][2] * p.Z() + mat[2][3];

		p = res;
	}
}

class CMatrix4x4d
	: public CMatrix<double, 4, 4>
{
public:
	//void createRotationMatrix(double alpha, double betha, double gamma, bool isRadians = true) noexcept;
	void createRotationMatrix(CVector axis, double tetha, bool isRadians = true) noexcept;
	void createTransferMatrix(CVector movement) noexcept;
	//void createRTMatrix(CVector movement, double alpha, double betha, double gamma, bool isRadians) noexcept;
	void createRTMatrix(CVector movement, CVector axis, double tetha, bool isRadians) noexcept;

	void setIdentityMatrix() noexcept;

	CPoint operator*(const CPoint& p) const noexcept;

	CMatrix4x4d() noexcept;
private:

};