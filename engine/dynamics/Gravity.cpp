#include "Gravity.hpp"
#include <cmath>

Vec3 TwoBodyGravity::AccelerationECI(const Vec3& r_ECI, double mu) {
    double r_mag = r_ECI.Magnitude();
    if (r_mag < 1.0) {
        return Vec3(0, 0, 0); // avoid singularity at origin
    }
    double factor = -mu / (r_mag * r_mag * r_mag);
    return r_ECI * factor;
}
