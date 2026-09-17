#include "J2Perturbation.hpp"
#include "dynamics/Gravity.hpp"
#include <cmath>

constexpr double kJ2 = 0.00108263;
constexpr double kEarthRadius = 6371e3; // meters

ForcesTorques J2PerturbationModel::Compute(const RigidBody& body, const SimState&) const {
    ForcesTorques ft;
    ft.torqueBody = Vec3(0, 0, 0); // J2 primarily causes a translational force perturbation

    double x = body.positionECI.x;
    double y = body.positionECI.y;
    double z = body.positionECI.z;
    double r_mag = body.positionECI.Magnitude();
    
    if (r_mag > 1.0) {
        double r2 = r_mag * r_mag;
        double r3 = r2 * r_mag;
        double r5 = r3 * r2;
        
        double factor = -1.5 * kEarthMu * kJ2 * (kEarthRadius * kEarthRadius) / r5;
        double z2_over_r2 = (z * z) / r2;
        
        double fx = factor * x * (1.0 - 5.0 * z2_over_r2);
        double fy = factor * y * (1.0 - 5.0 * z2_over_r2);
        double fz = factor * z * (3.0 - 5.0 * z2_over_r2);
        
        ft.forceECI = Vec3(fx, fy, fz) * body.mass;
    } else {
        ft.forceECI = Vec3(0, 0, 0);
    }
    
    return ft;
}
