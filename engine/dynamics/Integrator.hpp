#pragma once

#include "RigidBody.hpp"

class IIntegrator {
public:
    virtual void Step(RigidBody& body, const ForcesTorques& externalBody, double dt) = 0;
    virtual ~IIntegrator() = default;
};

class RK4Integrator : public IIntegrator {
public:
    void Step(RigidBody& body, const ForcesTorques& externalBody, double dt) override;
};
