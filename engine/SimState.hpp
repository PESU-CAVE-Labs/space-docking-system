#pragma once

#include "math/Vec3.hpp"
#include "math/Quaternion.hpp"

// Core state data bundle to pass immutable snapshots to renderer/frames
struct RelativeState {
    Vec3 position;
    Vec3 velocity;
    Quaternion attitude;       // from reference frame to this frame
    Vec3 angularVelocity;      // in this frame's body axes
};

struct SimState {
    double timestampS = 0.0;

    RelativeState chaserECI;
    RelativeState targetECI;

    // Derived states for UI and logging
    RelativeState relativeDockState; // Chaser Dock w.r.t Target Dock (14-element Y vector)
    
    // Additional physics data to be added as needed
    Vec3 chaserForceECI;
    Vec3 targetForceECI;
};
