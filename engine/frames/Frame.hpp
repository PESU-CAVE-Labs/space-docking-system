#pragma once

#include "SimState.hpp"
#include "math/Mat3.hpp"
#include "math/Vec3.hpp"
#include "math/Quaternion.hpp"

enum class FrameId {
    ECI,
    LVLH_Chaser,
    LVLH_Target,
    Body_Chaser,
    Body_Target,
    Dock_Chaser,
    Dock_Target
};

class ReferenceFrame {
public:
    virtual ~ReferenceFrame() = default;

    FrameId id;
    
    // Pose of this frame relative to ECI at a given time
    virtual Vec3 OriginInECI(const SimState& state) const = 0;
    virtual Mat3 DCMFromECI(const SimState& state) const = 0;
    virtual Vec3 VelocityInECI(const SimState& state) const = 0;
    virtual Vec3 AngularVelocityECI(const SimState& state) const = 0; // Angular velocity vector of this frame, in ECI
};

// Implementations for the specific frames

class ECIFrame : public ReferenceFrame {
public:
    ECIFrame() { id = FrameId::ECI; }
    
    Vec3 OriginInECI(const SimState&) const override { return Vec3(0, 0, 0); }
    Mat3 DCMFromECI(const SimState&) const override { return Mat3(); } // Identity
    Vec3 VelocityInECI(const SimState&) const override { return Vec3(0, 0, 0); }
    Vec3 AngularVelocityECI(const SimState&) const override { return Vec3(0, 0, 0); }
};

class BodyFrame : public ReferenceFrame {
public:
    BodyFrame(bool isChaser) { id = isChaser ? FrameId::Body_Chaser : FrameId::Body_Target; }
    
    const RelativeState& GetState(const SimState& s) const {
        return (id == FrameId::Body_Chaser) ? s.chaserECI : s.targetECI;
    }

    Vec3 OriginInECI(const SimState& state) const override { 
        return GetState(state).position; 
    }
    Mat3 DCMFromECI(const SimState& state) const override { 
        // Quaternion is ECI->Body
        return GetState(state).attitude.ToDCM(); 
    }
    Vec3 VelocityInECI(const SimState& state) const override { 
        return GetState(state).velocity; 
    }
    Vec3 AngularVelocityECI(const SimState& state) const override { 
        // Convert body angular velocity to ECI
        Mat3 dcm = DCMFromECI(state);
        return dcm.Transposed() * GetState(state).angularVelocity; 
    }
};

class DockFrame : public ReferenceFrame {
public:
    Vec3 offsetBody; // lever arm to docking face
    bool isChaser;
    
    DockFrame(bool isChaser, const Vec3& offset) : offsetBody(offset), isChaser(isChaser) {
        id = isChaser ? FrameId::Dock_Chaser : FrameId::Dock_Target;
    }
    
    const RelativeState& GetState(const SimState& s) const {
        return isChaser ? s.chaserECI : s.targetECI;
    }

    Vec3 OriginInECI(const SimState& state) const override { 
        const auto& bs = GetState(state);
        // r_dock = r_body + R_body->ECI * offset_body
        return bs.position + bs.attitude.Conjugate().Rotate(offsetBody);
    }
    Mat3 DCMFromECI(const SimState& state) const override { 
        // Orientation matches body frame
        return GetState(state).attitude.ToDCM(); 
    }
    Vec3 VelocityInECI(const SimState& state) const override { 
        const auto& bs = GetState(state);
        // v_dock = v_body + omega_ECI x r_offset_ECI
        Vec3 r_offset_ECI = bs.attitude.Conjugate().Rotate(offsetBody);
        Vec3 omega_ECI = bs.attitude.Conjugate().Rotate(bs.angularVelocity);
        return bs.velocity + omega_ECI.Cross(r_offset_ECI);
    }
    Vec3 AngularVelocityECI(const SimState& state) const override { 
        const auto& bs = GetState(state);
        return bs.attitude.Conjugate().Rotate(bs.angularVelocity);
    }
};

class LVLHFrame : public ReferenceFrame {
public:
    bool isChaser;
    
    LVLHFrame(bool isChaser) : isChaser(isChaser) {
        id = isChaser ? FrameId::LVLH_Chaser : FrameId::LVLH_Target;
    }
    
    const RelativeState& GetState(const SimState& s) const {
        return isChaser ? s.chaserECI : s.targetECI;
    }

    Vec3 OriginInECI(const SimState& state) const override { 
        return GetState(state).position;
    }
    
    Mat3 DCMFromECI(const SimState& state) const override { 
        const auto& bs = GetState(state);
        Vec3 r = bs.position;
        Vec3 v = bs.velocity;
        
        // Z = -r_hat (Nadir)
        Vec3 z_axis = (-r).Normalized();
        // Y = -(h x r)_hat = (r x v)_hat x r_hat = -Orbit Normal
        Vec3 h = r.Cross(v);
        Vec3 y_axis = (-h).Normalized();
        // X = Y x Z
        Vec3 x_axis = y_axis.Cross(z_axis).Normalized();
        
        // DCM rows are the axes
        return Mat3(
            x_axis.x, x_axis.y, x_axis.z,
            y_axis.x, y_axis.y, y_axis.z,
            z_axis.x, z_axis.y, z_axis.z
        );
    }
    
    Vec3 VelocityInECI(const SimState& state) const override { 
        return GetState(state).velocity; 
    }
    
    Vec3 AngularVelocityECI(const SimState& state) const override { 
        const auto& bs = GetState(state);
        Vec3 r = bs.position;
        Vec3 v = bs.velocity;
        Vec3 h = r.Cross(v);
        // Orbital rate: omega = h / r^2 in orbit normal direction (which is -y_axis)
        // More generally, omega_lvlh_ECI = (r x v) / |r|^2 = h / |r|^2
        double r2 = r.Dot(r);
        if (r2 > 1e-12) {
            return h / r2;
        }
        return Vec3(0,0,0);
    }
};
