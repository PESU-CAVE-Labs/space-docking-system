#include <gtest/gtest.h>
#include "frames/Frame.hpp"
#include "frames/FrameTransformer.hpp"

TEST(FrameEngine, LVLH_Creation) {
    SimState state;
    state.chaserECI.position = Vec3(7000e3, 0, 0); // 7000km on X
    state.chaserECI.velocity = Vec3(0, 7.5e3, 0);  // 7.5km/s on Y
    
    LVLHFrame lvlh(true); // Chaser
    
    Mat3 dcm = lvlh.DCMFromECI(state);
    
    // Z axis should be -X (Nadir)
    EXPECT_NEAR(dcm.m[2][0], -1.0, 1e-6);
    EXPECT_NEAR(dcm.m[2][1], 0.0, 1e-6);
    EXPECT_NEAR(dcm.m[2][2], 0.0, 1e-6);
    
    // Y axis should be -Z (Orbit Normal: -(r x v))
    // r x v = 7000e3 X 7.5e3 = +Z.  -(r x v) = -Z
    EXPECT_NEAR(dcm.m[1][0], 0.0, 1e-6);
    EXPECT_NEAR(dcm.m[1][1], 0.0, 1e-6);
    EXPECT_NEAR(dcm.m[1][2], -1.0, 1e-6);
}
