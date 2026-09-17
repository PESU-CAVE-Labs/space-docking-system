#include <gtest/gtest.h>
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "Vec3.hpp"
#include "Mat3.hpp"
#include "Quaternion.hpp"
#include "Euler.hpp"

const double EPS = 1e-6;

TEST(MathEngine, Vec3Operations) {
    Vec3 a(1, 2, 3);
    Vec3 b(4, 5, 6);
    
    Vec3 c = a + b;
    EXPECT_DOUBLE_EQ(c.x, 5);
    EXPECT_DOUBLE_EQ(c.y, 7);
    EXPECT_DOUBLE_EQ(c.z, 9);
    
    Vec3 cross = a.Cross(b);
    EXPECT_DOUBLE_EQ(cross.x, -3);
    EXPECT_DOUBLE_EQ(cross.y, 6);
    EXPECT_DOUBLE_EQ(cross.z, -3);
    
    EXPECT_DOUBLE_EQ(a.Dot(b), 32);
}

TEST(MathEngine, EulerToDCM) {
    // Test 90 degree yaw
    Mat3 rotYaw = EulerToDCM(M_PI/2, 0, 0);
    EXPECT_NEAR(rotYaw.m[0][0], 0, EPS);
    EXPECT_NEAR(rotYaw.m[0][1], 1, EPS);
    EXPECT_NEAR(rotYaw.m[1][0], -1, EPS);
    EXPECT_NEAR(rotYaw.m[1][1], 0, EPS);
    
    // Reverse
    EulerAngles e = DCMToEuler(rotYaw);
    EXPECT_NEAR(e.yaw, M_PI/2, EPS);
    EXPECT_NEAR(e.pitch, 0, EPS);
    EXPECT_NEAR(e.roll, 0, EPS);
}

TEST(MathEngine, QuaternionDCMConversion) {
    // Generate a test DCM
    Mat3 A = EulerToDCM(0.1, 0.2, 0.3);
    
    Quaternion q = Quaternion::FromDCM(A);
    Mat3 A_reconstructed = q.ToDCM();
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(A.m[i][j], A_reconstructed.m[i][j], EPS);
        }
    }
}
