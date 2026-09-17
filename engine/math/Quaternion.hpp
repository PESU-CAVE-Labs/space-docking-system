#pragma once

#include "Vec3.hpp"
#include "Mat3.hpp"

class Quaternion {
public:
    // Storage order matches SOW: q = [q1, q2, q3, q4]
    // where q4 is the scalar component (cos(alpha/2))
    double q1, q2, q3, q4;

    Quaternion() : q1(0), q2(0), q3(0), q4(1.0) {}
    Quaternion(double q1, double q2, double q3, double q4) 
        : q1(q1), q2(q2), q3(q3), q4(q4) {}

    static Quaternion FromAxisAngle(const Vec3& axis, double angleRad);
    
    // 4-branch robust extraction from the SOW
    static Quaternion FromDCM(const Mat3& A);

    // Exact matrix formula from the SOW
    Mat3 ToDCM() const;

    // Hamilton product, scalar-last convention
    Quaternion operator*(const Quaternion& rhs) const;

    Quaternion Normalized() const;
    Quaternion Conjugate() const;
    
    // v_rotated = q * v * q^-1
    Vec3 Rotate(const Vec3& v) const;

    // q̇ = 0.5 * Ω(ω) * q
    Quaternion Derivative(const Vec3& omega_body) const;
};
