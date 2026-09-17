#include "Integrator.hpp"
#include "Gravity.hpp"

void RK4Integrator::Step(RigidBody& body, const ForcesTorques& externalBody, double dt) {
    // 1st Stage
    StateDerivative k1 = body.ComputeDerivative(externalBody);
    
    // Save original state
    Vec3 pos0 = body.positionECI;
    Vec3 vel0 = body.velocityECI;
    Quaternion q0 = body.attitude;
    Vec3 w0 = body.angularVelocityBody;
    
    // Evaluate position-dependent gravitational field dynamically at intermediate RK4 substages
    bool hasGravityField = (pos0.Magnitude() > 1000.0);
    Vec3 g0 = hasGravityField ? TwoBodyGravity::AccelerationECI(pos0) : Vec3(0, 0, 0);

    // 2nd Stage
    body.positionECI = pos0 + k1.velocity * (dt * 0.5);
    body.velocityECI = vel0 + k1.acceleration * (dt * 0.5);
    body.attitude = Quaternion(
        q0.q1 + k1.q_dot.q1 * (dt * 0.5),
        q0.q2 + k1.q_dot.q2 * (dt * 0.5),
        q0.q3 + k1.q_dot.q3 * (dt * 0.5),
        q0.q4 + k1.q_dot.q4 * (dt * 0.5)
    ).Normalized();
    body.angularVelocityBody = w0 + k1.omega_dot * (dt * 0.5);
    StateDerivative k2 = body.ComputeDerivative(externalBody);
    if (hasGravityField) {
        k2.acceleration += (TwoBodyGravity::AccelerationECI(body.positionECI) - g0);
    }
    
    // 3rd Stage
    body.positionECI = pos0 + k2.velocity * (dt * 0.5);
    body.velocityECI = vel0 + k2.acceleration * (dt * 0.5);
    body.attitude = Quaternion(
        q0.q1 + k2.q_dot.q1 * (dt * 0.5),
        q0.q2 + k2.q_dot.q2 * (dt * 0.5),
        q0.q3 + k2.q_dot.q3 * (dt * 0.5),
        q0.q4 + k2.q_dot.q4 * (dt * 0.5)
    ).Normalized();
    body.angularVelocityBody = w0 + k2.omega_dot * (dt * 0.5);
    StateDerivative k3 = body.ComputeDerivative(externalBody);
    if (hasGravityField) {
        k3.acceleration += (TwoBodyGravity::AccelerationECI(body.positionECI) - g0);
    }
    
    // 4th Stage
    body.positionECI = pos0 + k3.velocity * dt;
    body.velocityECI = vel0 + k3.acceleration * dt;
    body.attitude = Quaternion(
        q0.q1 + k3.q_dot.q1 * dt,
        q0.q2 + k3.q_dot.q2 * dt,
        q0.q3 + k3.q_dot.q3 * dt,
        q0.q4 + k3.q_dot.q4 * dt
    ).Normalized();
    body.angularVelocityBody = w0 + k3.omega_dot * dt;
    StateDerivative k4 = body.ComputeDerivative(externalBody);
    if (hasGravityField) {
        k4.acceleration += (TwoBodyGravity::AccelerationECI(body.positionECI) - g0);
    }
    
    // Final accumulation
    StateDerivative finalDeriv;
    finalDeriv.velocity = (k1.velocity + k2.velocity * 2.0 + k3.velocity * 2.0 + k4.velocity) / 6.0;
    finalDeriv.acceleration = (k1.acceleration + k2.acceleration * 2.0 + k3.acceleration * 2.0 + k4.acceleration) / 6.0;
    
    finalDeriv.q_dot.q1 = (k1.q_dot.q1 + k2.q_dot.q1 * 2.0 + k3.q_dot.q1 * 2.0 + k4.q_dot.q1) / 6.0;
    finalDeriv.q_dot.q2 = (k1.q_dot.q2 + k2.q_dot.q2 * 2.0 + k3.q_dot.q2 * 2.0 + k4.q_dot.q2) / 6.0;
    finalDeriv.q_dot.q3 = (k1.q_dot.q3 + k2.q_dot.q3 * 2.0 + k3.q_dot.q3 * 2.0 + k4.q_dot.q3) / 6.0;
    finalDeriv.q_dot.q4 = (k1.q_dot.q4 + k2.q_dot.q4 * 2.0 + k3.q_dot.q4 * 2.0 + k4.q_dot.q4) / 6.0;
    
    finalDeriv.omega_dot = (k1.omega_dot + k2.omega_dot * 2.0 + k3.omega_dot * 2.0 + k4.omega_dot) / 6.0;
    
    // Restore and apply final derivative
    body.positionECI = pos0;
    body.velocityECI = vel0;
    body.attitude = q0;
    body.angularVelocityBody = w0;
    
    body.ApplyStateUpdate(finalDeriv, dt);
}
