#pragma once

#include "../SimState.hpp"
#include <string>
#include <fstream>
#include <mutex>
#include <atomic>

class DataLogger {
public:
    DataLogger(const std::string& filename = "simulation_pose_2ms.csv");
    ~DataLogger();

    // Start or stop recording
    void StartLogging(const std::string& filename);
    void StopLogging();
    void SetEnabled(bool enable);
    bool IsEnabled() const;

    // Log the current 6-DOF state (called every 2ms / 500Hz from physics thread)
    void LogState(const SimState& state);

    // Flush file buffer to disk
    void Flush();

    // Statistics
    size_t GetSampleCount() const;
    double GetLoggedDurationS() const;
    std::string GetFilename() const;

private:
    void WriteHeader();

    std::ofstream m_file;
    std::string m_filename;
    mutable std::mutex m_mutex;
    std::atomic<bool> m_enabled{false};
    std::atomic<size_t> m_sampleCount{0};
    double m_firstTimestamp{-1.0};
    double m_lastTimestamp{0.0};
};
