#pragma once

#include "SimState.hpp"
#include "dynamics/RigidBody.hpp"

class IDisturbanceModel {
public:
    virtual ForcesTorques Compute(const RigidBody& body, const SimState& env) const = 0;
    virtual ~IDisturbanceModel() = default;
};
