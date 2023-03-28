#pragma once
#include <functional>
#include <algorithm>
class CPoint
{
public:
	CPoint() noexcept;
	CPoint(const double& _x, const double& _y, const double& _z) noexcept;
	CPoint(const CPoint& other) noexcept;
	CPoint(const CPoint* other);
	CPoint& operator=(const CPoint& other) noexcept;
	inline double& X() noexcept
	{
		return x;
	}
	inline double& Y() noexcept
	{
		return y;
	}
	inline double& Z() noexcept
	{
		return z;
	}
	inline const double& X() const noexcept
	{
		return x;
	}
	inline const double& Y() const noexcept
	{
		return y;
	}
	inline const double& Z() const noexcept
	{
		return z;
	}
	inline CPoint& operator+=(const CPoint& other) noexcept
	{
		x += other.x;
		y += other.y;
		z += other.z;

		return *this;
	}
	inline CPoint& operator-=(const CPoint& other) noexcept
	{
		x -= other.x;
		y -= other.y;
		z -= other.z;

		return *this;
	}
	inline CPoint operator+(const CPoint& other) noexcept
	{
		return CPoint(x + other.x, y + other.y, z + other.z);
	}
	inline CPoint operator-(const CPoint& other) noexcept
	{
		return CPoint(x - other.x, y - other.y, z - other.z);
	}
protected:
	double x, y, z;
};

