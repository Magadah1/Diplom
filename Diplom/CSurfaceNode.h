#pragma once
#include "CPoint.h"

class CSurfaceNode :
    public CPoint
{
public:
    CSurfaceNode() noexcept;
    CSurfaceNode(const double& _x, const double& _y, const double& _z) noexcept;
    CSurfaceNode(const CPoint& p) noexcept;
    CSurfaceNode(const CPoint* p);
    CSurfaceNode(const CSurfaceNode& other) noexcept;
    CSurfaceNode(const CSurfaceNode* other);
    inline CPoint& likeCPoint() noexcept
    {
        return *this;
    }
    inline const CPoint& likeCPoint() const noexcept
    {
        return *this;
    }
    inline bool operator==(const CSurfaceNode& other) const noexcept
    {
        return sqrt(pow(other.x - this->x, 2) + pow(other.y - this->y, 2) + pow(other.z - this->z, 2)) < 1e-8; // 1e-8
    }
    inline bool operator!=(const CSurfaceNode& other) const noexcept
    {
        return !(*this == other);
    }
};

