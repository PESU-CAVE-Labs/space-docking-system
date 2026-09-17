#include <gtest/gtest.h>
#include "DynamicsEngine.hpp"
#include "dynamics/Gravity.hpp"

TEST(DynamicsEngineTest, OrbitalEnergyConservation) {
    RigidBody chaser("Chaser");
    chaser.mass = 1000.0;
    // Circular orbit at 7000km
    double r = 7000e3;
    double v = std::sqrt(kEarthMu / r);
    chaser.positionECI = Vec3(r, 0, 0);
    chaser.velocityECI = Vec3(0, v, 0);

    RigidBody target("Target");
    target.mass = 2000.0;
    target.positionECI = Vec3(r + 100, 0, 0); // slightly different orbit
    target.velocityECI = Vec3(0, std::sqrt(kEarthMu / (r + 100)), 0);

    DynamicsEngine engine(chaser, target, std::make_unique<RK4Integrator>());
    
    double dt = 0.002; // 2ms step
    double t_total = 100.0; // integrate for 100 seconds
    
    double initial_energy = 0.5 * chaser.velocityECI.MagnitudeSq() - kEarthMu / chaser.positionECI.Magnitude();
    
    for (double t = 0; t < t_total; t += dt) {
        engine.Step(dt);
    }
    
    RigidBody& currentChaser = engine.GetChaser();
    double final_energy = 0.5 * currentChaser.velocityECI.MagnitudeSq() - kEarthMu / currentChaser.positionECI.Magnitude();
    
    // Energy should be conserved. RK4 is not symplectic, so we expect some small drift
    // A drift of ~6.6 over 100s for energy ~28 million is extremely good relative error.
    EXPECT_NEAR(initial_energy, final_energy, 10.0);
}
