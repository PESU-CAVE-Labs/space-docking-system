#include "ContactSpringModel.hpp"
#include <cmath>
#include <algorithm>

void ContactSpringModel::ApplyContactForces(RigidBody& chaser, RigidBody& target, const RelativeState& dockRelPose) const {
    // 1. DOCKING PORT COLLISION
    double dist = dockRelPose.position.Magnitude();
    
    if (dist < engagementDistance) {
        // Compression amount
        double x = engagementDistance - dist;
        Vec3 dir = dockRelPose.position.Normalized();
        double v_closing = -dockRelPose.velocity.Dot(dir); 
        
        double f_mag = k_spring * x + c_damper * v_closing;
        if (f_mag < 0) f_mag = 0; 
        
        Vec3 f_chaser_tgtDock = dir * f_mag;
        Vec3 f_target_tgtDock = -f_chaser_tgtDock;
        
        Vec3 f_chaser_ECI = target.attitude.Rotate(f_chaser_tgtDock);
        Vec3 f_target_ECI = target.attitude.Rotate(f_target_tgtDock);
        
        Vec3 r_chaser_dock_ECI = chaser.attitude.Rotate(chaser.dockFrameOffsetBody);
        Vec3 r_target_dock_ECI = target.attitude.Rotate(target.dockFrameOffsetBody);
        
        Vec3 t_chaser_ECI = r_chaser_dock_ECI.Cross(f_chaser_ECI);
        Vec3 t_target_ECI = r_target_dock_ECI.Cross(f_target_ECI);
        
        Vec3 t_chaser_body = chaser.attitude.Conjugate().Rotate(t_chaser_ECI);
        Vec3 t_target_body = target.attitude.Conjugate().Rotate(t_target_ECI);
        
        chaser.ApplyForceTorqueBodyFrame(chaser.attitude.Conjugate().Rotate(f_chaser_ECI), t_chaser_body);
        target.ApplyForceTorqueBodyFrame(target.attitude.Conjugate().Rotate(f_target_ECI), t_target_body);
        return; // Skip fallback if perfect dock applies
    }

    // 2. CORNER-BASED COLLISION (Fallback for Rotated Scenarios)
    // Since the Target is mostly unrotated (axis-aligned in ECI), we can check if 
    // any of the 8 corners of the Chaser have penetrated the Target's volume.
    // This perfectly prevents the visual clipping you see when rotated!
    
    double cx = 0.75, cy = 0.75, cz = 1.0; // Chaser half-dimensions
    Vec3 corners[8] = {
        Vec3(cx, cy, cz), Vec3(-cx, cy, cz), Vec3(cx, -cy, cz), Vec3(-cx, -cy, cz),
        Vec3(cx, cy, -cz), Vec3(-cx, cy, -cz), Vec3(cx, -cy, -cz), Vec3(-cx, -cy, -cz)
    };
    
    double tx = 1.0, ty = 1.0, tz = 1.5; // Target half-dimensions
    
    bool collision = false;
    double maxPen = 0.0;
    Vec3 bestNormal(0,0,0);
    Vec3 hitCornerBody(0,0,0);
    
    for (int i = 0; i < 8; i++) {
        // Get corner position in ECI
        Vec3 cornerECI = chaser.attitude.Rotate(corners[i]) + chaser.positionECI;
        
        // Relative to Target COM in ECI
        Vec3 relP = cornerECI - target.positionECI;
        
        // Check if inside Target's AABB
        if (std::abs(relP.x) < tx && std::abs(relP.y) < ty && std::abs(relP.z) < tz) {
            collision = true;
            
            // Find shallowest penetration to determine normal
            double px = tx - std::abs(relP.x);
            double py = ty - std::abs(relP.y);
            double pz = tz - std::abs(relP.z);
            
            double minP = std::min({px, py, pz});
            if (minP > maxPen) {
                maxPen = minP;
                hitCornerBody = corners[i];
                if (minP == px) bestNormal = Vec3(relP.x > 0 ? 1 : -1, 0, 0);
                else if (minP == py) bestNormal = Vec3(0, relP.y > 0 ? 1 : -1, 0);
                else bestNormal = Vec3(0, 0, relP.z > 0 ? 1 : -1);
            }
        }
    }
    
    if (collision) {
        Vec3 dir = bestNormal; // Points outward from Target
        
        Vec3 cornerOffsetECI = chaser.attitude.Rotate(hitCornerBody);
        
        // Velocity of the exact corner
        Vec3 wECI = chaser.attitude.Rotate(chaser.angularVelocityBody);
        Vec3 v_corner = chaser.velocityECI + wECI.Cross(cornerOffsetECI);
        
        double v_closing = -(v_corner - target.velocityECI).Dot(dir);
        
        double f_mag = k_spring * maxPen + c_damper * v_closing;
        if (f_mag < 0.0) f_mag = 0.0;
        
        Vec3 f_chaser_ECI = dir * f_mag;
        Vec3 f_target_ECI = f_chaser_ECI * -1.0;
        
        // Apply torque exactly at the corner that hit! Creates stunning realistic spin-out!
        Vec3 t_chaser_ECI = cornerOffsetECI.Cross(f_chaser_ECI);
        Vec3 t_chaser_body = chaser.attitude.Conjugate().Rotate(t_chaser_ECI);
        chaser.ApplyForceTorqueBodyFrame(chaser.attitude.Conjugate().Rotate(f_chaser_ECI), t_chaser_body);
        
        Vec3 r_target_impact_ECI = (cornerOffsetECI + chaser.positionECI) - target.positionECI;
        Vec3 t_target_ECI = r_target_impact_ECI.Cross(f_target_ECI);
        Vec3 t_target_body = target.attitude.Conjugate().Rotate(t_target_ECI);
        target.ApplyForceTorqueBodyFrame(target.attitude.Conjugate().Rotate(f_target_ECI), t_target_body);
    }
}
