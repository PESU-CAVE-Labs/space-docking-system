#pragma once

#include "DisturbanceModel.hpp"

class J2PerturbationModel : public IDisturbanceModel {
public:
    ForcesTorques Compute(const RigidBody& body, const SimState& env) const override;
};
