#pragma once

#include "Vec3.hpp"
#include <array>

class Mat3 {
public:
    // Row-major storage for easier human reading, 
    // but we can convert to GLM / column-major when needed.
    std::array<std::array<double, 3>, 3> m;

    Mat3() {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m[i][j] = (i == j) ? 1.0 : 0.0;
    }

    Mat3(double m00, double m01, double m02,
         double m10, double m11, double m12,
         double m20, double m21, double m22) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    // Multiply matrix by vector
    Vec3 operator*(const Vec3& v) const {
        return Vec3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    // Multiply matrix by matrix
    Mat3 operator*(const Mat3& rhs) const {
        Mat3 res;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                res.m[i][j] = 0;
                for (int k = 0; k < 3; ++k) {
                    res.m[i][j] += m[i][k] * rhs.m[k][j];
                }
            }
        }
        return res;
    }

    Mat3 Transposed() const {
        return Mat3(
            m[0][0], m[1][0], m[2][0],
            m[0][1], m[1][1], m[2][1],
            m[0][2], m[1][2], m[2][2]
        );
    }
};
