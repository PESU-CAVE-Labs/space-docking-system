#include "DataLogger.hpp"
#include "../math/Euler.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

DataLogger::DataLogger(const std::string& filename) : m_filename(filename) {
    // Start with logging ready (can be enabled via toggle or default)
    StartLogging(filename);
}

DataLogger::~DataLogger() {
    StopLogging();
}

void DataLogger::StartLogging(const std::string& filename) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_file.is_open()) {
        m_file.flush();
        m_file.close();
    }
    m_filename = filename;
    m_file.open(filename, std::ios::out | std::ios::trunc);
    if (m_file.is_open()) {
        m_file << std::fixed << std::setprecision(6);
        WriteHeader();
        m_sampleCount = 0;
        m_firstTimestamp = -1.0;
        m_lastTimestamp = 0.0;
        m_enabled = true;
        std::cout << "[DataLogger] Started logging 6-DOF pose data to: " << filename << std::endl;
    } else {
        std::cerr << "[DataLogger] Error: Failed to open log file: " << filename << std::endl;
        m_enabled = false;
    }
}

void DataLogger::StopLogging() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_enabled = false;
    if (m_file.is_open()) {
        m_file.flush();
        m_file.close();
        std::cout << "[DataLogger] Stopped logging. Total samples recorded: " << m_sampleCount << std::endl;
    }
}

void DataLogger::SetEnabled(bool enable) {
    if (enable && !m_enabled) {
        if (!m_file.is_open()) {
            StartLogging(m_filename);
        } else {
            m_enabled = true;
        }
    } else if (!enable && m_enabled) {
        m_enabled = false;
        Flush();
    }
}

bool DataLogger::IsEnabled() const {
    return m_enabled.load();
}

size_t DataLogger::GetSampleCount() const {
    return m_sampleCount.load();
}

double DataLogger::GetLoggedDurationS() const {
    if (m_firstTimestamp < 0.0 || m_sampleCount == 0) return 0.0;
    return m_lastTimestamp - m_firstTimestamp;
}

std::string DataLogger::GetFilename() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_filename;
}

void DataLogger::WriteHeader() {
    if (!m_file.is_open()) return;
    m_file << "Time_s,"
           // Chaser Pose
           << "Chaser_X_m,Chaser_Y_m,Chaser_Z_m,"
           << "Chaser_Vx_mps,Chaser_Vy_mps,Chaser_Vz_mps,"
           << "Chaser_q1,Chaser_q2,Chaser_q3,Chaser_q4,"
           << "Chaser_Roll_deg,Chaser_Pitch_deg,Chaser_Yaw_deg,"
           << "Chaser_wx_radps,Chaser_wy_radps,Chaser_wz_radps,"
           // Target Pose
           << "Target_X_m,Target_Y_m,Target_Z_m,"
           << "Target_Vx_mps,Target_Vy_mps,Target_Vz_mps,"
           << "Target_q1,Target_q2,Target_q3,Target_q4,"
           << "Target_Roll_deg,Target_Pitch_deg,Target_Yaw_deg,"
           << "Target_wx_radps,Target_wy_radps,Target_wz_radps,"
           // Relative Dock Pose
           << "Rel_Dock_X_m,Rel_Dock_Y_m,Rel_Dock_Z_m,"
           << "Rel_Dock_Vx_mps,Rel_Dock_Vy_mps,Rel_Dock_Vz_mps,"
           << "Rel_Dock_Dist_m,Rel_Dock_Speed_mps,"
           << "Rel_Dock_q1,Rel_Dock_q2,Rel_Dock_q3,Rel_Dock_q4,"
           << "Rel_Dock_Roll_deg,Rel_Dock_Pitch_deg,Rel_Dock_Yaw_deg,"
           << "Rel_Dock_wx_radps,Rel_Dock_wy_radps,Rel_Dock_wz_radps\n";
}

void DataLogger::LogState(const SimState& state) {
    if (!m_enabled.load()) return;

    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_file.is_open()) return;

    const double radToDeg = 180.0 / M_PI;

    // Convert Chaser attitude quaternion to Euler angles
    EulerAngles chaserEuler = DCMToEuler(state.chaserECI.attitude.ToDCM());
    // Convert Target attitude quaternion to Euler angles
    EulerAngles targetEuler = DCMToEuler(state.targetECI.attitude.ToDCM());
    // Convert Relative Dock attitude quaternion to Euler angles
    EulerAngles relEuler = DCMToEuler(state.relativeDockState.attitude.ToDCM());

    double relDist = state.relativeDockState.position.Magnitude();
    double relSpeed = state.relativeDockState.velocity.Magnitude();

    if (m_firstTimestamp < 0.0) {
        m_firstTimestamp = state.timestampS;
    }
    m_lastTimestamp = state.timestampS;

    m_file << state.timestampS << ","
           // Chaser Pose
           << state.chaserECI.position.x << ","
           << state.chaserECI.position.y << ","
           << state.chaserECI.position.z << ","
           << state.chaserECI.velocity.x << ","
           << state.chaserECI.velocity.y << ","
           << state.chaserECI.velocity.z << ","
           << state.chaserECI.attitude.q1 << ","
           << state.chaserECI.attitude.q2 << ","
           << state.chaserECI.attitude.q3 << ","
           << state.chaserECI.attitude.q4 << ","
           << chaserEuler.roll * radToDeg << ","
           << chaserEuler.pitch * radToDeg << ","
           << chaserEuler.yaw * radToDeg << ","
           << state.chaserECI.angularVelocity.x << ","
           << state.chaserECI.angularVelocity.y << ","
           << state.chaserECI.angularVelocity.z << ","
           // Target Pose
           << state.targetECI.position.x << ","
           << state.targetECI.position.y << ","
           << state.targetECI.position.z << ","
           << state.targetECI.velocity.x << ","
           << state.targetECI.velocity.y << ","
           << state.targetECI.velocity.z << ","
           << state.targetECI.attitude.q1 << ","
           << state.targetECI.attitude.q2 << ","
           << state.targetECI.attitude.q3 << ","
           << state.targetECI.attitude.q4 << ","
           << targetEuler.roll * radToDeg << ","
           << targetEuler.pitch * radToDeg << ","
           << targetEuler.yaw * radToDeg << ","
           << state.targetECI.angularVelocity.x << ","
           << state.targetECI.angularVelocity.y << ","
           << state.targetECI.angularVelocity.z << ","
           // Relative Dock Pose
           << state.relativeDockState.position.x << ","
           << state.relativeDockState.position.y << ","
           << state.relativeDockState.position.z << ","
           << state.relativeDockState.velocity.x << ","
           << state.relativeDockState.velocity.y << ","
           << state.relativeDockState.velocity.z << ","
           << relDist << ","
           << relSpeed << ","
           << state.relativeDockState.attitude.q1 << ","
           << state.relativeDockState.attitude.q2 << ","
           << state.relativeDockState.attitude.q3 << ","
           << state.relativeDockState.attitude.q4 << ","
           << relEuler.roll * radToDeg << ","
           << relEuler.pitch * radToDeg << ","
           << relEuler.yaw * radToDeg << ","
           << state.relativeDockState.angularVelocity.x << ","
           << state.relativeDockState.angularVelocity.y << ","
           << state.relativeDockState.angularVelocity.z << "\n";

    m_sampleCount++;
}

void DataLogger::Flush() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_file.is_open()) {
        m_file.flush();
    }
}
