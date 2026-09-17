#pragma once

#include "math/Vec3.hpp"
#include "math/Mat3.hpp"
#include "math/Quaternion.hpp"
#include <string>

struct ForcesTorques {
    Vec3 forceECI;
    Vec3 torqueBody;
};

struct StateDerivative {
    Vec3 velocity;
    Vec3 acceleration;
    Quaternion q_dot;
    Vec3 omega_dot;
};

class RigidBody {
public:
    std::string name;

    // Mass properties
    double mass = 1000.0;
    Mat3 inertiaTensorBody;
    Mat3 invInertiaTensorBody; // Precomputed inverse for fast update
    Vec3 cgOffsetFromGeomCenter;
    Vec3 dockFrameOffsetBody;

    // State (ECI frame)
    Vec3 positionECI;
    Vec3 velocityECI;
    Quaternion attitude;      // ECI -> Body
    Vec3 angularVelocityBody; // ω, expressed in body axes

    // Accumulators
    Vec3 forceAccumECI;
    Vec3 torqueAccumBody;

    RigidBody(const std::string& name);

    void SetInertia(const Mat3& I);

    void ClearAccumulators();
    void ApplyForceECI(const Vec3& F);
    void ApplyTorqueBody(const Vec3& T);
    void ApplyForceTorqueBodyFrame(const Vec3& F_body, const Vec3& T_body);

    StateDerivative ComputeDerivative(const ForcesTorques& externalBody) const;
    void ApplyStateUpdate(const StateDerivative& deriv, double dt);
};
