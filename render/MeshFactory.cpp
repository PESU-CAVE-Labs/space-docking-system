#include "MeshFactory.hpp"
#include <cmath>

std::unique_ptr<Mesh> MeshFactory::MakeSquareBox(double sideLen, double depth) {
    double hl = sideLen / 2.0;
    double hd = depth / 2.0;

    std::vector<Vertex> vertices = {
        // Front face (Z = +hd)
        {Vec3(-hl, -hl, hd), Vec3(0, 0, 1)},
        {Vec3( hl, -hl, hd), Vec3(0, 0, 1)},
        {Vec3( hl,  hl, hd), Vec3(0, 0, 1)},
        {Vec3(-hl,  hl, hd), Vec3(0, 0, 1)},
        // Back face (Z = -hd)
        {Vec3(-hl, -hl, -hd), Vec3(0, 0, -1)},
        {Vec3( hl, -hl, -hd), Vec3(0, 0, -1)},
        {Vec3( hl,  hl, -hd), Vec3(0, 0, -1)},
        {Vec3(-hl,  hl, -hd), Vec3(0, 0, -1)},
        // Left face (X = -hl)
        {Vec3(-hl, -hl, -hd), Vec3(-1, 0, 0)},
        {Vec3(-hl, -hl,  hd), Vec3(-1, 0, 0)},
        {Vec3(-hl,  hl,  hd), Vec3(-1, 0, 0)},
        {Vec3(-hl,  hl, -hd), Vec3(-1, 0, 0)},
        // Right face (X = hl)
        {Vec3(hl, -hl, -hd), Vec3(1, 0, 0)},
        {Vec3(hl, -hl,  hd), Vec3(1, 0, 0)},
        {Vec3(hl,  hl,  hd), Vec3(1, 0, 0)},
        {Vec3(hl,  hl, -hd), Vec3(1, 0, 0)},
        // Top face (Y = hl)
        {Vec3(-hl, hl, -hd), Vec3(0, 1, 0)},
        {Vec3( hl, hl, -hd), Vec3(0, 1, 0)},
        {Vec3( hl, hl,  hd), Vec3(0, 1, 0)},
        {Vec3(-hl, hl,  hd), Vec3(0, 1, 0)},
        // Bottom face (Y = -hl)
        {Vec3(-hl, -hl, -hd), Vec3(0, -1, 0)},
        {Vec3( hl, -hl, -hd), Vec3(0, -1, 0)},
        {Vec3( hl, -hl,  hd), Vec3(0, -1, 0)},
        {Vec3(-hl, -hl,  hd), Vec3(0, -1, 0)}
    };

    std::vector<unsigned int> indices = {
        0, 1, 2,  2, 3, 0,       // Front
        6, 5, 4,  4, 7, 6,       // Back
        8, 9, 10, 10, 11, 8,     // Left
        14, 13, 12, 12, 15, 14,  // Right
        16, 17, 18, 18, 19, 16,  // Top
        22, 21, 20, 20, 23, 22   // Bottom
    };

    return std::make_unique<Mesh>(vertices, indices, GL_TRIANGLES);
}

std::unique_ptr<Mesh> MeshFactory::MakeAxisTriad(double length) {
    std::vector<Vertex> vertices = {
        // X axis (Red)
        {Vec3(0, 0, 0), Vec3(1, 0.1, 0.1)}, {Vec3(length, 0, 0), Vec3(1, 0.1, 0.1)},
        // Y axis (Green)
        {Vec3(0, 0, 0), Vec3(0.1, 1, 0.1)}, {Vec3(0, length, 0), Vec3(0.1, 1, 0.1)},
        // Z axis (Blue)
        {Vec3(0, 0, 0), Vec3(0.2, 0.5, 1)}, {Vec3(0, 0, length), Vec3(0.2, 0.5, 1)}
    };
    
    std::vector<unsigned int> indices = { 0, 1, 2, 3, 4, 5 };
    return std::make_unique<Mesh>(vertices, indices, GL_LINES);
}

std::unique_ptr<Mesh> MeshFactory::MakeCoordinateArrows(double length, double arrowHeadLength, double shaftRadius, double headRadius) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    const int segments = 12;
    double shaftLen = length - arrowHeadLength;
    if (shaftLen < 0.05) shaftLen = 0.05;

    auto addArrow = [&](const Vec3& dir, const Vec3& u, const Vec3& v, const Vec3& color) {
        unsigned int baseIdx = static_cast<unsigned int>(vertices.size());

        // 1. Shaft Cylinder
        for (int i = 0; i < segments; ++i) {
            double angle = 2.0 * M_PI * i / segments;
            Vec3 radial = u * (std::cos(angle) * shaftRadius) + v * (std::sin(angle) * shaftRadius);
            vertices.push_back({radial, color});
            vertices.push_back({dir * shaftLen + radial, color});
        }

        for (int i = 0; i < segments; ++i) {
            int next = (i + 1) % segments;
            unsigned int b0 = baseIdx + i * 2;
            unsigned int t0 = baseIdx + i * 2 + 1;
            unsigned int b1 = baseIdx + next * 2;
            unsigned int t1 = baseIdx + next * 2 + 1;

            indices.push_back(b0);
            indices.push_back(b1);
            indices.push_back(t1);

            indices.push_back(b0);
            indices.push_back(t1);
            indices.push_back(t0);
        }

        // 2. Arrowhead Cone base disk & tip
        unsigned int coneBaseStart = static_cast<unsigned int>(vertices.size());
        Vec3 coneBaseCenter = dir * shaftLen;
        vertices.push_back({coneBaseCenter, color});
        unsigned int coneCenterIdx = coneBaseStart;

        for (int i = 0; i < segments; ++i) {
            double angle = 2.0 * M_PI * i / segments;
            Vec3 radial = u * (std::cos(angle) * headRadius) + v * (std::sin(angle) * headRadius);
            vertices.push_back({coneBaseCenter + radial, color});
        }

        Vec3 tipPos = dir * length;
        vertices.push_back({tipPos, color});
        unsigned int tipIdx = static_cast<unsigned int>(vertices.size() - 1);

        for (int i = 0; i < segments; ++i) {
            int next = (i + 1) % segments;
            unsigned int c0 = coneBaseStart + 1 + i;
            unsigned int c1 = coneBaseStart + 1 + next;

            indices.push_back(coneCenterIdx);
            indices.push_back(c1);
            indices.push_back(c0);

            indices.push_back(c0);
            indices.push_back(c1);
            indices.push_back(tipIdx);
        }
    };

    // +X (Red)
    addArrow(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1.0, 0.15, 0.15));
    // +Y (Green)
    addArrow(Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 0, 0), Vec3(0.15, 1.0, 0.15));
    // +Z (Blue)
    addArrow(Vec3(0, 0, 1), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0.2, 0.5, 1.0));

    // Center origin cube
    double c = shaftRadius * 1.5;
    unsigned int oIdx = static_cast<unsigned int>(vertices.size());
    Vec3 oColor(0.9, 0.9, 0.9);
    vertices.push_back({Vec3(-c, -c, -c), oColor});
    vertices.push_back({Vec3( c, -c, -c), oColor});
    vertices.push_back({Vec3( c,  c, -c), oColor});
    vertices.push_back({Vec3(-c,  c, -c), oColor});
    vertices.push_back({Vec3(-c, -c,  c), oColor});
    vertices.push_back({Vec3( c, -c,  c), oColor});
    vertices.push_back({Vec3( c,  c,  c), oColor});
    vertices.push_back({Vec3(-c,  c,  c), oColor});

    unsigned int cubeInds[] = {
        0,1,2, 2,3,0, 4,5,6, 6,7,4,
        0,4,7, 7,3,0, 1,5,6, 6,2,1,
        3,2,6, 6,7,3, 0,1,5, 5,4,0
    };
    for (unsigned int ci : cubeInds) {
        indices.push_back(oIdx + ci);
    }

    return std::make_unique<Mesh>(vertices, indices, GL_TRIANGLES);
}

std::unique_ptr<Mesh> MeshFactory::MakeEarthSphere(double radius, int stacks, int slices) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= stacks; ++i) {
        double V = (double)i / (double)stacks;
        double phi = V * M_PI;

        for (int j = 0; j <= slices; ++j) {
            double U = (double)j / (double)slices;
            double theta = U * (M_PI * 2);

            double x = std::cos(theta) * std::sin(phi);
            double y = std::cos(phi);
            double z = std::sin(theta) * std::sin(phi);

            vertices.push_back({Vec3(x * radius, y * radius, z * radius), Vec3(x, y, z)});
        }
    }

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            unsigned int first = (i * (slices + 1)) + j;
            unsigned int second = first + slices + 1;

            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(first + 1);

            indices.push_back(second);
            indices.push_back(second + 1);
            indices.push_back(first + 1);
        }
    }

    return std::make_unique<Mesh>(vertices, indices, GL_TRIANGLES);
}

std::unique_ptr<Mesh> MeshFactory::MakeOrbitPath(const std::vector<Vec3>& points) {
    std::vector<Vertex> vertices;
    for (const auto& p : points) {
        vertices.push_back({p, Vec3(0,1,0)});
    }
    std::vector<unsigned int> indices;
    for (size_t i = 0; i < points.size(); ++i) {
        indices.push_back(i);
    }
    return std::make_unique<Mesh>(vertices, indices, GL_LINE_STRIP);
}
