#pragma once

#include "DisturbanceModel.hpp"

class AtmosphericDragModel : public IDisturbanceModel {
public:
    double dragCoefficient = 2.2;
    double crossSectionalArea = 1.0; // m^2
    double atmosphericDensity = 1e-12; // kg/m^3 (approx for LEO)

    ForcesTorques Compute(const RigidBody& body, const SimState& env) const override;
};
