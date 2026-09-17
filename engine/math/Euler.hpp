#pragma once

#include "Mat3.hpp"

struct EulerAngles {
    double yaw;   // Rotation around Z axis (psi)
    double pitch; // Rotation around new Y axis (theta)
    double roll;  // Rotation around new X axis (phi)
};

// Converts intrinsic Yaw-Pitch-Roll (Z-Y-X) Euler angles to a DCM
Mat3 EulerToDCM(double yaw, double pitch, double roll);

// Extracts intrinsic Yaw-Pitch-Roll (Z-Y-X) Euler angles from a DCM
EulerAngles DCMToEuler(const Mat3& A);
