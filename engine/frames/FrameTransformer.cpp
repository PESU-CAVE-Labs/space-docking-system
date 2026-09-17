#include "FrameTransformer.hpp"

RelativeState FrameTransformer::Transform(const ReferenceFrame& target, const ReferenceFrame& reference, const SimState& state) {
    RelativeState rel;
    
    // Position of target w.r.t reference, in ECI
    Vec3 r_ECI = target.OriginInECI(state) - reference.OriginInECI(state);
    // Velocity of target w.r.t reference, in ECI
    Vec3 v_ECI = target.VelocityInECI(state) - reference.VelocityInECI(state);
    
    // DCMs
    Mat3 ref_DCM = reference.DCMFromECI(state);
    Mat3 tgt_DCM = target.DCMFromECI(state);
    
    // Relative position and velocity expressed in the reference frame
    rel.position = ref_DCM * r_ECI;
    
    // Coriolis and transport terms for velocity in rotating frame:
    // v_rel = R_ref * (v_ECI - omega_ref_ECI x r_ECI)
    Vec3 omega_ref = reference.AngularVelocityECI(state);
    rel.velocity = ref_DCM * (v_ECI - omega_ref.Cross(r_ECI));
    
    // Relative attitude (from Reference to Target)
    // ref_DCM: ECI -> Ref
    // tgt_DCM: ECI -> Tgt
    // rel_DCM = tgt_DCM * ref_DCM^T
    Mat3 rel_DCM = tgt_DCM * ref_DCM.Transposed();
    rel.attitude = Quaternion::FromDCM(rel_DCM);
    
    // Relative angular velocity (expressed in Target frame per typical convention, or Reference frame depending on use)
    // SOW Y vector expects states "of Chaser Dock w.r.t Target Dock".
    // Angular velocity omega_rel = omega_tgt - omega_ref (all in ECI)
    // Expressed in Target frame:
    Vec3 omega_tgt = target.AngularVelocityECI(state);
    Vec3 omega_rel_ECI = omega_tgt - omega_ref;
    rel.angularVelocity = tgt_DCM * omega_rel_ECI;
    
    return rel;
}
