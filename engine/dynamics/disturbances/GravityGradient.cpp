#include "GravityGradient.hpp"
#include "dynamics/Gravity.hpp"
#include <cmath>

ForcesTorques GravityGradientTorqueModel::Compute(const RigidBody& body, const SimState&) const {
    ForcesTorques ft;
    ft.forceECI = Vec3(0, 0, 0); // Already handled by main Gravity model

    // T = 3 * mu / r^3 * (r_hat x (I * r_hat))
    double r_mag = body.positionECI.Magnitude();
    if (r_mag > 1.0) {
        Vec3 r_hat_ECI = body.positionECI / r_mag;
        Vec3 r_hat_body = body.attitude.Rotate(r_hat_ECI); // from ECI to body
        
        Vec3 I_r_hat = body.inertiaTensorBody * r_hat_body;
        Vec3 cross = r_hat_body.Cross(I_r_hat);
        
        double factor = 3.0 * kEarthMu / (r_mag * r_mag * r_mag);
        ft.torqueBody = cross * factor;
    } else {
        ft.torqueBody = Vec3(0, 0, 0);
    }
    
    return ft;
}
