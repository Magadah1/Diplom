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
};

