#pragma once

#include "math/Vec3.hpp"

// Earth's gravitational parameter, m^3/s^2
constexpr double kEarthMu = 3.986004418e14;

class TwoBodyGravity {
public:
    static Vec3 AccelerationECI(const Vec3& r_ECI, double mu = kEarthMu);
};
