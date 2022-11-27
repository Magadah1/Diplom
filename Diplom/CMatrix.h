#pragma once
#include <array>
#include "CVector.h"
//#include <type_traits>

template<typename elementType, size_t m, size_t n>
class CMatrix
{
public:
	CMatrix() noexcept;

	size_t N() const noexcept;
	size_t M() const noexcept;

	elementType& operator()(const size_t& mPos, const size_t& nPos);
	const elementType& operator()(const size_t& mPos, const size_t& nPos) const;

protected:
	std::array<std::array<elementType, n>, m> mat;
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