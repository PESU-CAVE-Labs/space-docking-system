#include <gtest/gtest.h>
#include "DynamicsEngine.hpp"
#include "contact/ContactSpringModel.hpp"

TEST(ContactEngineTest, ValidationScenarioHighMass) {
    // SOW Figure 7 Validation setup
    RigidBody chaser("Chaser");
    chaser.mass = 5000.0; // High mass
    chaser.positionECI = Vec3(0, 0, -0.06); // Just outside engagement (0.05m)
    chaser.velocityECI = Vec3(0, 0, 0.1); // Moving at 10cm/s towards target
    chaser.attitude = Quaternion(); // Identity
    chaser.dockFrameOffsetBody = Vec3(0, 0, 0);

    RigidBody target("Target");
    target.mass = 5000.0;
    target.positionECI = Vec3(0, 0, 0);
    target.velocityECI = Vec3(0, 0, 0);
    target.attitude = Quaternion(); // Identity
    target.dockFrameOffsetBody = Vec3(0, 0, 0);

    DynamicsEngine engine(chaser, target, std::make_unique<RK4Integrator>());
    engine.EnableDisturbances(false); // disable for pure contact test
    
    double dt = 0.001; // 1ms step
    
    bool engaged = false;
    double max_force = 0;
    
    for (double t = 0; t < 2.0; t += dt) {
        engine.Step(dt);
        SimState s = engine.GetSnapshot();
        double dist = s.relativeDockState.position.Magnitude();
        if (dist < 0.05) {
            engaged = true;
            // Target force Z component is negative since chaser pushes it
            double force_z = std::abs(s.targetForceECI.z);
            if (force_z > max_force) max_force = force_z;
        }
    }
    
    EXPECT_TRUE(engaged);
    EXPECT_GT(max_force, 0.0); // Should register a contact force spike
}
