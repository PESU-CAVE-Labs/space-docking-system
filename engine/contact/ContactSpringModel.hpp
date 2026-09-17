#pragma once

#include "SimState.hpp"
#include "dynamics/RigidBody.hpp"

class ContactSpringModel {
public:
    // Extremely stiff spring for a visually flush collision
    double k_spring = 10000000.0;   // 10,000,000 N/m
    double c_damper = 100000.0;     // 100,000 N/(m/s)
    double engagementDistance = 0.02; // 2 cm engagement bounding box

    // Compute contact forces applied to Chaser and Target dock frames.
    // The RelativeState should be the state of Chaser Dock w.r.t Target Dock, 
    // evaluated in the Target Dock frame (which acts as the reference for contact).
    void ApplyContactForces(RigidBody& chaser, RigidBody& target, const RelativeState& dockRelPose) const;
};
