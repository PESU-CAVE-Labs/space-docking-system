#include "Quaternion.hpp"
#include <cmath>

Quaternion Quaternion::FromAxisAngle(const Vec3& axis, double angleRad) {
    double halfAngle = angleRad * 0.5;
    double sinHalf = std::sin(halfAngle);
    Vec3 normAxis = axis.Normalized();
    return Quaternion(
        normAxis.x * sinHalf,
        normAxis.y * sinHalf,
        normAxis.z * sinHalf,
        std::cos(halfAngle)
    );
}

Quaternion Quaternion::FromDCM(const Mat3& A) {
    // Implementing the 4-branch SOW logic for numerical stability
    double a11 = A.m[0][0], a12 = A.m[0][1], a13 = A.m[0][2];
    double a21 = A.m[1][0], a22 = A.m[1][1], a23 = A.m[1][2];
    double a31 = A.m[2][0], a32 = A.m[2][1], a33 = A.m[2][2];
    
    double trace = a11 + a22 + a33;
    
    // Find maximum of trace, a11, a22, a33 to avoid division by zero or small numbers
    double maxVal = trace;
    int maxIdx = 0;
    if (a11 > maxVal) { maxVal = a11; maxIdx = 1; }
    if (a22 > maxVal) { maxVal = a22; maxIdx = 2; }
    if (a33 > maxVal) { maxVal = a33; maxIdx = 3; }

    Quaternion q;
    
    if (maxIdx == 0) {
        // Trace is max
        double den = std::sqrt(1.0 + trace); // 2 * q4
        q.q4 = 0.5 * den;
        den = 0.5 / den; // 0.25 / q4
        q.q1 = (a23 - a32) * den;
        q.q2 = (a31 - a13) * den;
        q.q3 = (a12 - a21) * den;
    } else if (maxIdx == 1) {
        // a11 is max
        double den = std::sqrt(1.0 + a11 - a22 - a33); // 2 * q1
        q.q1 = 0.5 * den;
        den = 0.5 / den; // 0.25 / q1
        q.q2 = (a12 + a21) * den;
        q.q3 = (a13 + a31) * den;
        q.q4 = (a23 - a32) * den;
    } else if (maxIdx == 2) {
        // a22 is max
        double den = std::sqrt(1.0 - a11 + a22 - a33); // 2 * q2
        q.q2 = 0.5 * den;
        den = 0.5 / den; // 0.25 / q2
        q.q1 = (a12 + a21) * den;
        q.q3 = (a23 + a32) * den;
        q.q4 = (a31 - a13) * den;
    } else {
        // a33 is max
        double den = std::sqrt(1.0 - a11 - a22 + a33); // 2 * q3
        q.q3 = 0.5 * den;
        den = 0.5 / den; // 0.25 / q3
        q.q1 = (a13 + a31) * den;
        q.q2 = (a23 + a32) * den;
        q.q4 = (a12 - a21) * den;
    }
    
    // We can enforce q4 to be positive for uniqueness, although not strictly necessary
    if (q.q4 < 0) {
        q.q1 = -q.q1;
        q.q2 = -q.q2;
        q.q3 = -q.q3;
        q.q4 = -q.q4;
    }
    return q.Normalized();
}

Mat3 Quaternion::ToDCM() const {
    double q1_2 = q1 * q1;
    double q2_2 = q2 * q2;
    double q3_2 = q3 * q3;
    double q4_2 = q4 * q4;

    return Mat3(
        q1_2 - q2_2 - q3_2 + q4_2, 2.0 * (q1 * q2 + q3 * q4),       2.0 * (q1 * q3 - q2 * q4),
        2.0 * (q1 * q2 - q3 * q4), -q1_2 + q2_2 - q3_2 + q4_2,      2.0 * (q2 * q3 + q1 * q4),
        2.0 * (q1 * q3 + q2 * q4), 2.0 * (q2 * q3 - q1 * q4),       -q1_2 - q2_2 + q3_2 + q4_2
    );
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
    // Hamilton product: scalar last
    return Quaternion(
        q4*rhs.q1 + q3*rhs.q2 - q2*rhs.q3 + q1*rhs.q4,
       -q3*rhs.q1 + q4*rhs.q2 + q1*rhs.q3 + q2*rhs.q4,
        q2*rhs.q1 - q1*rhs.q2 + q4*rhs.q3 + q3*rhs.q4,
       -q1*rhs.q1 - q2*rhs.q2 - q3*rhs.q3 + q4*rhs.q4
    );
}

Quaternion Quaternion::Normalized() const {
    double mag = std::sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    if (mag > 1e-12) {
        return Quaternion(q1/mag, q2/mag, q3/mag, q4/mag);
    }
    return Quaternion(); // Identity
}

Quaternion Quaternion::Conjugate() const {
    return Quaternion(-q1, -q2, -q3, q4);
}

Vec3 Quaternion::Rotate(const Vec3& v) const {
    Quaternion vq(v.x, v.y, v.z, 0.0);
    Quaternion res = (*this) * vq * Conjugate();
    return Vec3(res.q1, res.q2, res.q3);
}

Quaternion Quaternion::Derivative(const Vec3& omega) const {
    // q_dot = 0.5 * Omega * q (or 0.5 * q * omega_quat)
    Quaternion omega_quat(omega.x, omega.y, omega.z, 0.0);
    Quaternion res = (*this) * omega_quat;
    return Quaternion(res.q1 * 0.5, res.q2 * 0.5, res.q3 * 0.5, res.q4 * 0.5);
}
