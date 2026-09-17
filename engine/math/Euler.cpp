#include "Euler.hpp"
#include <cmath>

Mat3 EulerToDCM(double yaw, double pitch, double roll) {
    // Exact equations from SOW Figure 6 and subsequent matrices
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    return Mat3(
        cp*cy,                  cp*sy,                 -sp,
        sr*sp*cy - cr*sy,       sr*sp*sy + cr*cy,       sr*cp,
        cr*sp*cy + sr*sy,       cr*sp*sy - sr*cy,       cr*cp
    );
}

EulerAngles DCMToEuler(const Mat3& A) {
    EulerAngles e;
    
    // a13 = -sin(pitch)
    e.pitch = std::asin(-A.m[0][2]);

    if (std::cos(e.pitch) > 1e-8) {
        // Not at gimbal lock
        e.roll = std::atan2(A.m[1][2], A.m[2][2]);
        e.yaw  = std::atan2(A.m[0][1], A.m[0][0]);
    } else {
        // Gimbal lock case
        e.roll = 0.0;
        e.yaw  = std::atan2(-A.m[1][0], A.m[1][1]);
    }

    return e;
}
