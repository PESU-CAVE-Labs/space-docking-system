#include "RigidBody.hpp"
#include <cmath>

RigidBody::RigidBody(const std::string& name) : name(name) {
    SetInertia(Mat3(
        1000, 0, 0,
        0, 1000, 0,
        0, 0, 1000
    ));
}

void RigidBody::SetInertia(const Mat3& I) {
    inertiaTensorBody = I;
    // Calculate inverse manually for a 3x3 matrix (assuming symmetric, positive definite)
    double a11 = I.m[0][0], a12 = I.m[0][1], a13 = I.m[0][2];
    double a21 = I.m[1][0], a22 = I.m[1][1], a23 = I.m[1][2];
    double a31 = I.m[2][0], a32 = I.m[2][1], a33 = I.m[2][2];

    double det = a11 * (a22 * a33 - a32 * a23) -
                 a12 * (a21 * a33 - a31 * a23) +
                 a13 * (a21 * a32 - a31 * a22);
    
    if (std::abs(det) > 1e-12) {
        double invdet = 1.0 / det;
        invInertiaTensorBody.m[0][0] = (a22 * a33 - a32 * a23) * invdet;
        invInertiaTensorBody.m[0][1] = (a13 * a32 - a12 * a33) * invdet;
        invInertiaTensorBody.m[0][2] = (a12 * a23 - a13 * a22) * invdet;
        invInertiaTensorBody.m[1][0] = (a23 * a31 - a21 * a33) * invdet;
        invInertiaTensorBody.m[1][1] = (a11 * a33 - a13 * a31) * invdet;
        invInertiaTensorBody.m[1][2] = (a21 * a13 - a11 * a23) * invdet;
        invInertiaTensorBody.m[2][0] = (a21 * a32 - a31 * a22) * invdet;
        invInertiaTensorBody.m[2][1] = (a31 * a12 - a11 * a32) * invdet;
        invInertiaTensorBody.m[2][2] = (a11 * a22 - a21 * a12) * invdet;
    }
}

void RigidBody::ClearAccumulators() {
    forceAccumECI = Vec3(0, 0, 0);
    torqueAccumBody = Vec3(0, 0, 0);
}

void RigidBody::ApplyForceECI(const Vec3& F) {
    forceAccumECI += F;
}

void RigidBody::ApplyTorqueBody(const Vec3& T) {
    torqueAccumBody += T;
}

void RigidBody::ApplyForceTorqueBodyFrame(const Vec3& F_body, const Vec3& T_body) {
    torqueAccumBody += T_body;
    // Rotate force to ECI
    forceAccumECI += attitude.Conjugate().Rotate(F_body);
}

StateDerivative RigidBody::ComputeDerivative(const ForcesTorques& externalBody) const {
    StateDerivative deriv;
    deriv.velocity = velocityECI;
    
    // Translational: m*a = F_gravity(accumulated) + F_external_ECI
    Vec3 totalForce = forceAccumECI + externalBody.forceECI;
    deriv.acceleration = totalForce / mass;
    
    // Rotational: I*w_dot = T_ext - w x (I*w)
    Vec3 totalTorque = torqueAccumBody + externalBody.torqueBody;
    Vec3 w_x_Iw = angularVelocityBody.Cross(inertiaTensorBody * angularVelocityBody);
    deriv.omega_dot = invInertiaTensorBody * (totalTorque - w_x_Iw);
    
    // Attitude kinematics: q_dot = 0.5 * Omega * q
    deriv.q_dot = attitude.Derivative(angularVelocityBody);
    
    return deriv;
}

void RigidBody::ApplyStateUpdate(const StateDerivative& deriv, double dt) {
    positionECI += deriv.velocity * dt;
    velocityECI += deriv.acceleration * dt;
    angularVelocityBody += deriv.omega_dot * dt;
    
    attitude.q1 += deriv.q_dot.q1 * dt;
    attitude.q2 += deriv.q_dot.q2 * dt;
    attitude.q3 += deriv.q_dot.q3 * dt;
    attitude.q4 += deriv.q_dot.q4 * dt;
    
    attitude = attitude.Normalized();
}
