#pragma once

#include "Vec3.hpp"
#include <array>

class Mat4 {
public:
    // Row-major storage for easier human reading.
    std::array<std::array<double, 4>, 4> m;

    Mat4() {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m[i][j] = (i == j) ? 1.0 : 0.0;
    }

    // Multiply matrix by matrix
    Mat4 operator*(const Mat4& rhs) const {
        Mat4 res;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                res.m[i][j] = 0;
                for (int k = 0; k < 4; ++k) {
                    res.m[i][j] += m[i][k] * rhs.m[k][j];
                }
            }
        }
        return res;
    }
};
