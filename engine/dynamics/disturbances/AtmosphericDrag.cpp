#include "AtmosphericDrag.hpp"

ForcesTorques AtmosphericDragModel::Compute(const RigidBody& body, const SimState&) const {
    ForcesTorques ft;
    
    // F = -0.5 * rho * Cd * A * |v| * v
    double v_mag = body.velocityECI.Magnitude();
    if (v_mag > 1e-12) {
        // F = -0.5 * rho * Cd * A * |v| * v
        double factor = 0.5 * atmosphericDensity * dragCoefficient * crossSectionalArea * v_mag;
        ft.forceECI = -body.velocityECI * factor;
    } else {
        ft.forceECI = Vec3(0, 0, 0);
    }
    
    // Drag torque is typically modeled based on Center of Pressure offset from Center of Mass.
    // For this simple box model, we'll assume a zero CoP offset -> zero drag torque.
    ft.torqueBody = Vec3(0, 0, 0);
    
    return ft;
}
