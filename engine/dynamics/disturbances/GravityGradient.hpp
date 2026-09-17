#pragma once

#include "DisturbanceModel.hpp"

class GravityGradientTorqueModel : public IDisturbanceModel {
public:
    ForcesTorques Compute(const RigidBody& body, const SimState& env) const override;
};
