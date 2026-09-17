#pragma once

#include "Mesh.hpp"
#include <memory>

class MeshFactory {
public:
    static std::unique_ptr<Mesh> MakeSquareBox(double sideLen, double depth);
    static std::unique_ptr<Mesh> MakeAxisTriad(double length);
    static std::unique_ptr<Mesh> MakeCoordinateArrows(double length, double arrowHeadLength = 0.6, double shaftRadius = 0.04, double headRadius = 0.12);
    static std::unique_ptr<Mesh> MakeEarthSphere(double radius, int stacks, int slices);
    static std::unique_ptr<Mesh> MakeOrbitPath(const std::vector<Vec3>& points);
};
