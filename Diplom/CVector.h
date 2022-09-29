#pragma once
#include "CSurfaceNode.h"
#include <cmath>
class CVector :
    public CPoint
{
public:
    CVector() noexcept;
    CVector(const double& _x, const double& _y, const double& _z) noexcept;
    CVector(const double& _xS, const double& _yS, const double& _zS, const double& _xE, const double& _yE, const double& _zE) noexcept;
    CVector(const CPoint& p) noexcept;
    CVector(const CPoint* p);
    CVector(const CPoint& start, const CPoint& end) noexcept;
    CVector(const CPoint* start, const CPoint* end);
    CVector(const CVector& other) noexcept;
    CVector(const CVector* other);
    CVector operator=(const CPoint& other) noexcept;

    inline double Length() const noexcept
    {
        return sqrt(x * x + y * y + z * z);
    }
    inline double LengthSqr() const noexcept
    {
        return x * x + y * y + z * z;
    }
    inline CVector& Norm() noexcept
    {
        double l = Length();
        if (!l)
            return *this;

        l = 1 / l;

        x *= l;
        y *= l;
        z *= l;

        return *this;
    }
    inline double operator*(const CVector& other) const noexcept
    {
        return x * other.x + y * other.y + z * other.z;
    }
    inline CVector operator*(const double& n) const noexcept
    {
        return CVector(x * n, y * n, z * n);
    }
    inline CVector operator/(const double& n) const
    {
        return CVector(x / n, y / n, z / n);
    }
    inline CVector operator+(const CVector& other) const noexcept
    {
        return CVector(x + other.x, y + other.y, z + other.z);
    }
    inline CVector operator-(const CVector& other) const noexcept
    {
        return CVector(x - other.x, y - other.y, z - other.z);
    }
    inline CVector& operator+=(const CVector& other) noexcept
    {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }
    inline CVector& operator-=(const CVector& other) noexcept
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return *this;
    }
    inline CVector& operator*=(const double& n) noexcept
    {
        x *= n;
        y *= n;
        z *= n;

        return *this;
    }
    inline CVector& operator/=(const double& n)
    {
        x /= n;
        y /= n;
        z /= n;

        return *this;
    }
    inline bool operator==(const CVector& other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
    inline bool operator!=(const CVector& other) const noexcept
    {
        return (*this == other);
    }
    inline CVector operator%(const CVector& other) const noexcept
    {
        return CVector(y * other.z - z * other.y, -(x * other.z - z * other.x), x * other.y - y * other.x);
    }
    inline CVector operator-() const noexcept
    {
        return CVector(-x, -y, -z);
    }
    inline double Mixed(const CVector& v2, const CVector& v3) const noexcept
    {
        return (*this % v2) * v3;
    }
    inline double Mixed(const CVector* v2, const CVector* v3) const 
    {
        return Mixed(*v2, *v3);
    }
    inline CPoint toCPoint() const noexcept
    {
        return CPoint(x, y, z);
    }
    inline CSurfaceNode toCSurfaceNode() const noexcept
    {
        return CSurfaceNode(x, y, z);
    }
    inline CPoint& likePoint() noexcept
    {
        return *this;
    }
    inline const CPoint& likePoint() const noexcept
    {
        return *this;
    }
    inline CVector createVector(const CPoint& end) const noexcept
    {
        return CVector(end.X() - x, end.Y() - y, end.Z() - z);
    }
    inline CVector createVector(const CPoint* end) const
    {
        return createVector(*end);
    }
};

