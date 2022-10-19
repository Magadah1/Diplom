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
        return this->x == other.x && this->y == other.y && this->z == other.z;
    }
    inline bool operator!=(const CSurfaceNode& other) const noexcept
    {
        return !(*this == other);
    }
};

