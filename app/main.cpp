#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#include "engine/DynamicsEngine.hpp"
#include "engine/contact/ContactSpringModel.hpp"
#include "engine/dynamics/Gravity.hpp"
#include "engine/dynamics/disturbances/AtmosphericDrag.hpp"
#include "engine/dynamics/disturbances/GravityGradient.hpp"
#include "engine/dynamics/disturbances/J2Perturbation.hpp"

#include "engine/logging/DataLogger.hpp"
#include "render/MeshFactory.hpp"
#include "render/Renderer.hpp"
#include "render/SceneObject.hpp"
#include "render/Shader.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

// ─── Shared simulation state ──────────────────────────────────────────────────
std::atomic<bool>  g_SimRunning{true};
std::atomic<bool>  g_SimPaused{false};
std::mutex         g_StateMutex;
SimState           g_CurrentState;

std::mutex                    g_EngineMutex;
std::unique_ptr<DynamicsEngine> g_Engine;

std::atomic<bool> g_PoseLoggingEnabled{false};

// ─── Engine factory ───────────────────────────────────────────────────────────
DynamicsEngine* CreateEngine(int scenario, double approachSpeed = 2.0) {
    RigidBody chaser("Chaser");
    chaser.mass = 500.0;
    chaser.dockFrameOffsetBody = Vec3(0, 0, 1.0);

    RigidBody target("Target");
    target.mass = 2000.0;
    target.dockFrameOffsetBody = Vec3(0, 0, -1.5);

    double r = 7000e3;
    double v = std::sqrt(kEarthMu / r);
    target.positionECI = Vec3(r, 0, 0);
    target.velocityECI = Vec3(0, v, 0);

    if (scenario == 0) {
        chaser.positionECI = Vec3(r, 0, -20);
        chaser.velocityECI = Vec3(0, v, approachSpeed);
    } else if (scenario == 1) {
        chaser.positionECI = Vec3(r + 1.2, 0, -20);
        chaser.velocityECI = Vec3(0, v, approachSpeed);
    } else if (scenario == 2) {
        chaser.positionECI = Vec3(r + 0.8, 0, -20);
        chaser.velocityECI = Vec3(0, v, approachSpeed);
        chaser.attitude = Quaternion(0.0, 0.38268, 0.0, 0.92388);
    }

    auto* engine = new DynamicsEngine(chaser, target, std::make_unique<RK4Integrator>());
    engine->AddDisturbance(BodySelector::Chaser, std::make_unique<AtmosphericDragModel>());
    engine->AddDisturbance(BodySelector::Target, std::make_unique<AtmosphericDragModel>());
    engine->AddDisturbance(BodySelector::Chaser, std::make_unique<GravityGradientTorqueModel>());
    engine->AddDisturbance(BodySelector::Target, std::make_unique<GravityGradientTorqueModel>());
    engine->AddDisturbance(BodySelector::Chaser, std::make_unique<J2PerturbationModel>());
    engine->AddDisturbance(BodySelector::Target, std::make_unique<J2PerturbationModel>());
    return engine;
}

// ─── Simulation thread ────────────────────────────────────────────────────────
void SimulationThread(double dt, DataLogger* logger) {
    auto target_dt = std::chrono::duration<double>(dt);

    while (g_SimRunning) {
        auto start = std::chrono::steady_clock::now();

        if (!g_SimPaused.load()) {
            SimState snapshot;
            {
                std::lock_guard<std::mutex> lock(g_EngineMutex);
                if (g_Engine) {
                    g_Engine->Step(dt);
                    snapshot = g_Engine->GetSnapshot();
                    {
                        std::lock_guard<std::mutex> stateLock(g_StateMutex);
                        g_CurrentState = snapshot;
                    }
                }
            }
            if (g_PoseLoggingEnabled.load()) {
                logger->LogState(snapshot);
            }
        }

        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < target_dt)
            std::this_thread::sleep_for(target_dt - elapsed);
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "Starting Spacecraft Docking Simulator" << std::endl;

    {
        std::lock_guard<std::mutex> lock(g_EngineMutex);
        g_Engine.reset(CreateEngine(0));
    }

    DataLogger logger("simulation_pose_2ms.csv");
    logger.SetEnabled(false);

    double sim_dt = 0.002;
    std::thread simThread(SimulationThread, sim_dt, &logger);

    // ── Renderer / window ───────────────────────────────────────────────────
    Renderer renderer(1280, 720, "Spacecraft Docking Simulator VDE");
    GLContext& ctx = renderer.GetContext();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // ── Style: clean dark theme ─────────────────────────────────────────────
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding    = 6.0f;
    style.FrameRounding     = 4.0f;
    style.ItemSpacing       = ImVec2(8, 6);
    style.FramePadding      = ImVec2(6, 4);
    style.WindowPadding     = ImVec2(12, 10);
    style.ScrollbarRounding = 4.0f;
    style.GrabRounding      = 4.0f;
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]       = ImVec4(0.10f, 0.10f, 0.14f, 0.95f);
    colors[ImGuiCol_Header]         = ImVec4(0.20f, 0.40f, 0.70f, 0.50f);
    colors[ImGuiCol_HeaderHovered]  = ImVec4(0.25f, 0.50f, 0.85f, 0.70f);
    colors[ImGuiCol_HeaderActive]   = ImVec4(0.25f, 0.50f, 0.85f, 1.00f);
    colors[ImGuiCol_Button]         = ImVec4(0.20f, 0.35f, 0.60f, 0.80f);
    colors[ImGuiCol_ButtonHovered]  = ImVec4(0.28f, 0.48f, 0.80f, 1.00f);
    colors[ImGuiCol_ButtonActive]   = ImVec4(0.18f, 0.30f, 0.55f, 1.00f);
    colors[ImGuiCol_FrameBg]        = ImVec4(0.16f, 0.16f, 0.22f, 1.00f);
    colors[ImGuiCol_SliderGrab]     = ImVec4(0.35f, 0.60f, 1.00f, 0.80f);
    colors[ImGuiCol_CheckMark]      = ImVec4(0.35f, 0.80f, 0.35f, 1.00f);
    colors[ImGuiCol_TitleBg]        = ImVec4(0.08f, 0.08f, 0.12f, 1.00f);
    colors[ImGuiCol_TitleBgActive]  = ImVec4(0.14f, 0.20f, 0.40f, 1.00f);
    colors[ImGuiCol_Separator]      = ImVec4(0.30f, 0.30f, 0.45f, 0.80f);

    ImGui_ImplGlfw_InitForOpenGL(ctx.Handle(), true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // ── Shaders ─────────────────────────────────────────────────────────────
    auto litShader  = std::make_shared<Shader>("../render/shaders/lit.vert",  "../render/shaders/lit.frag");
    auto lineShader = std::make_shared<Shader>("../render/shaders/line.vert", "../render/shaders/line.frag");

    // ── Meshes ───────────────────────────────────────────────────────────────
    auto chaserMesh        = MeshFactory::MakeSquareBox(1.5, 2.0);
    auto targetMesh        = MeshFactory::MakeSquareBox(2.0, 3.0);
    auto chaserBodyArrows  = MeshFactory::MakeCoordinateArrows(3.0, 0.6, 0.05, 0.14);
    auto targetBodyArrows  = MeshFactory::MakeCoordinateArrows(3.0, 0.6, 0.05, 0.14);
    auto chaserDockArrows  = MeshFactory::MakeCoordinateArrows(1.8, 0.36, 0.03, 0.09);
    auto targetDockArrows  = MeshFactory::MakeCoordinateArrows(1.8, 0.36, 0.03, 0.09);

    // ── Scene objects ────────────────────────────────────────────────────────
    auto chaserObj = std::make_shared<SceneObject>(std::move(chaserMesh), litShader);
    chaserObj->SetColor(Vec3(0.85f, 0.25f, 0.20f));

    auto targetObj = std::make_shared<SceneObject>(std::move(targetMesh), litShader);
    targetObj->SetColor(Vec3(0.55f, 0.55f, 0.60f));

    // Triads — uColor(0,0,0) makes shader use per-vertex colors
    auto chaserBodyTriad = std::make_shared<SceneObject>(std::move(chaserBodyArrows), lineShader);
    chaserBodyTriad->SetColor(Vec3(0, 0, 0));

    auto targetBodyTriad = std::make_shared<SceneObject>(std::move(targetBodyArrows), lineShader);
    targetBodyTriad->SetColor(Vec3(0, 0, 0));

    auto chaserDockTriad = std::make_shared<SceneObject>(std::move(chaserDockArrows), lineShader);
    chaserDockTriad->SetColor(Vec3(0, 0, 0));

    auto targetDockTriad = std::make_shared<SceneObject>(std::move(targetDockArrows), lineShader);
    targetDockTriad->SetColor(Vec3(0, 0, 0));

    renderer.AddObject(chaserObj);
    renderer.AddObject(targetObj);
    renderer.AddObject(chaserBodyTriad);
    renderer.AddObject(targetBodyTriad);
    renderer.AddObject(chaserDockTriad);
    renderer.AddObject(targetDockTriad);

    // ── Camera ───────────────────────────────────────────────────────────────
    Camera cam(Vec3(20, 20, 20), Vec3(0, 0, 0), Vec3(0, 0, 1), 45.0f, 1280.0f / 720.0f);
    float camDist      = 30.0f;
    float camAzimuth   = -0.8f;
    float camElevation = 0.5f;

    // ── UI state ─────────────────────────────────────────────────────────────
    bool showBodyTriads  = true;
    bool showDockTriads  = false;   // off by default — avoids the "5 arrows" confusion
    float coordScale     = 1.0f;

    bool  poseLogging    = false;
    static char logFilename[256] = "simulation_pose_2ms.csv";
    bool  simPaused      = false;
    int   activeScenario = 0;
    float approachSpeed  = 2.0f;

    // Dock offsets must match CreateEngine()
    const Vec3 chaserDockOffset(0, 0,  1.0);
    const Vec3 targetDockOffset(0, 0, -1.5);

    // Plot data
    std::vector<double> timeData, distData, velData;

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (!ctx.ShouldClose()) {
        ctx.PollEvents();

        // ── Dynamic viewport & aspect ratio (fixes fullscreen) ────────────────
        int fbW = 1280, fbH = 720;
        ctx.GetFramebufferSize(fbW, fbH);
        glViewport(0, 0, fbW, fbH);
        if (fbH > 0)
            cam.SetAspect(static_cast<float>(fbW) / static_cast<float>(fbH));

        // ── Camera orbit input ────────────────────────────────────────────────
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                ImVec2 delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
                camAzimuth   -= delta.x * 0.005f;
                camElevation += delta.y * 0.005f;
                camElevation  = std::max(-1.5f, std::min(1.5f, camElevation));
                ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);
            }
            camDist -= io.MouseWheel * 2.0f;
            camDist  = std::max(5.0f, std::min(200.0f, camDist));
        }

        // ── Read simulation state ─────────────────────────────────────────────
        SimState state;
        {
            std::lock_guard<std::mutex> lock(g_StateMutex);
            state = g_CurrentState;
        }

        // ── Update spacecraft positions ───────────────────────────────────────
        Vec3 chaserRelPos = state.chaserECI.position - state.targetECI.position;

        targetObj->SetPosition(Vec3(0, 0, 0));
        targetObj->SetRotation(state.targetECI.attitude);

        chaserObj->SetPosition(chaserRelPos);
        chaserObj->SetRotation(state.chaserECI.attitude);

        // ── Update coordinate triads ──────────────────────────────────────────
        Vec3 scaleOn (coordScale, coordScale, coordScale);
        Vec3 scaleOff(0, 0, 0);

        // Body triads
        if (showBodyTriads) {
            chaserBodyTriad->SetPosition(chaserRelPos);
            chaserBodyTriad->SetRotation(state.chaserECI.attitude);
            chaserBodyTriad->SetScale(scaleOn);

            targetBodyTriad->SetPosition(Vec3(0, 0, 0));
            targetBodyTriad->SetRotation(state.targetECI.attitude);
            targetBodyTriad->SetScale(scaleOn);
        } else {
            chaserBodyTriad->SetScale(scaleOff);
            targetBodyTriad->SetScale(scaleOff);
        }

        // Dock port triads (offset from body CoM along the body Z axis)
        if (showDockTriads) {
            Vec3 cDockRel = chaserRelPos + state.chaserECI.attitude.Rotate(chaserDockOffset);
            chaserDockTriad->SetPosition(cDockRel);
            chaserDockTriad->SetRotation(state.chaserECI.attitude);
            chaserDockTriad->SetScale(scaleOn);

            Vec3 tDockRel = state.targetECI.attitude.Rotate(targetDockOffset);
            targetDockTriad->SetPosition(tDockRel);
            targetDockTriad->SetRotation(state.targetECI.attitude);
            targetDockTriad->SetScale(scaleOn);
        } else {
            chaserDockTriad->SetScale(scaleOff);
            targetDockTriad->SetScale(scaleOff);
        }

        // ── Camera ────────────────────────────────────────────────────────────
        float cx = camDist * cosf(camElevation) * cosf(camAzimuth);
        float cy = camDist * cosf(camElevation) * sinf(camAzimuth);
        float cz = camDist * sinf(camElevation);
        cam.SetTarget(Vec3(0, 0, 0));
        cam.SetPosition(Vec3(cx, cy, cz));

        // ── 3-D render ────────────────────────────────────────────────────────
        renderer.Render(cam);

        // ── ImGui ─────────────────────────────────────────────────────────────
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Single anchored left sidebar — full height, fixed width 300 px
        const float SIDEBAR_W = 300.0f;
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(SIDEBAR_W, static_cast<float>(fbH)), ImGuiCond_Always);
        ImGui::Begin("##sidebar", nullptr,
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize   |
            ImGuiWindowFlags_NoMove     |
            ImGuiWindowFlags_NoScrollbar|
            ImGuiWindowFlags_NoSavedSettings);

        // ── App title ─────────────────────────────────────────────────────────
        ImGui::TextColored(ImVec4(0.4f, 0.75f, 1.0f, 1.0f), "SPACECRAFT DOCKING SIM");
        ImGui::Spacing();

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 1 — Simulation Control
        // ══════════════════════════════════════════════════════════════════════
        if (ImGui::CollapsingHeader("  Simulation Control", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Spacing();

            // Scenario buttons
            ImGui::TextDisabled("Scenario");
            ImGui::PushStyleColor(ImGuiCol_Button,
                activeScenario == 0 ? ImVec4(0.20f,0.55f,0.20f,0.90f) : ImVec4(0.20f,0.35f,0.60f,0.80f));
            if (ImGui::Button("Head-On", ImVec2(-1, 0))) {
                activeScenario = 0;
                std::lock_guard<std::mutex> lk(g_EngineMutex);
                g_Engine.reset(CreateEngine(0, approachSpeed));
                timeData.clear(); distData.clear(); velData.clear();
            }
            ImGui::PopStyleColor();
            ImGui::PushStyleColor(ImGuiCol_Button,
                activeScenario == 1 ? ImVec4(0.20f,0.55f,0.20f,0.90f) : ImVec4(0.20f,0.35f,0.60f,0.80f));
            if (ImGui::Button("Off-Center", ImVec2(-1, 0))) {
                activeScenario = 1;
                std::lock_guard<std::mutex> lk(g_EngineMutex);
                g_Engine.reset(CreateEngine(1, approachSpeed));
                timeData.clear(); distData.clear(); velData.clear();
            }
            ImGui::PopStyleColor();
            ImGui::PushStyleColor(ImGuiCol_Button,
                activeScenario == 2 ? ImVec4(0.20f,0.55f,0.20f,0.90f) : ImVec4(0.20f,0.35f,0.60f,0.80f));
            if (ImGui::Button("Off-Center + Rotated", ImVec2(-1, 0))) {
                activeScenario = 2;
                std::lock_guard<std::mutex> lk(g_EngineMutex);
                g_Engine.reset(CreateEngine(2, approachSpeed));
                timeData.clear(); distData.clear(); velData.clear();
            }
            ImGui::PopStyleColor();

            ImGui::Spacing();
            ImGui::TextDisabled("Approach Speed");
            ImGui::SetNextItemWidth(-1);
            if (ImGui::SliderFloat("##approachSpeed", &approachSpeed, 0.1f, 10.0f, "%.2f m/s")) {
                std::lock_guard<std::mutex> lk(g_EngineMutex);
                if (g_Engine) {
                    auto& chaser = g_Engine->GetChaser();
                    const auto& target = g_Engine->GetTarget();
                    chaser.velocityECI.z = target.velocityECI.z + approachSpeed;
                }
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Start / Stop / Reset row
            float btnW = (SIDEBAR_W - style.WindowPadding.x * 2 - style.ItemSpacing.x * 2) / 3.0f;

            // Start
            ImGui::PushStyleColor(ImGuiCol_Button,
                !simPaused ? ImVec4(0.15f,0.50f,0.15f,0.90f) : ImVec4(0.20f,0.35f,0.60f,0.80f));
            if (ImGui::Button("Start", ImVec2(btnW, 30))) {
                simPaused = false;
                g_SimPaused.store(false);
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();

            // Stop
            ImGui::PushStyleColor(ImGuiCol_Button,
                simPaused ? ImVec4(0.60f,0.20f,0.15f,0.90f) : ImVec4(0.20f,0.35f,0.60f,0.80f));
            if (ImGui::Button("Stop", ImVec2(btnW, 30))) {
                simPaused = true;
                g_SimPaused.store(true);
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();

            // Reset
            if (ImGui::Button("Reset", ImVec2(btnW, 30))) {
                simPaused = false;
                g_SimPaused.store(false);
                std::lock_guard<std::mutex> lk(g_EngineMutex);
                g_Engine.reset(CreateEngine(activeScenario, approachSpeed));
                timeData.clear(); distData.clear(); velData.clear();
            }

            // Status badge
            ImGui::Spacing();
            if (simPaused)
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.1f, 1.0f), "  ⏸  PAUSED");
            else
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "  ▶  RUNNING");

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Live telemetry
            ImGui::TextDisabled("Telemetry");
            ImGui::Text("Time        %.3f s", state.timestampS);
            double relDist = state.relativeDockState.position.Magnitude();
            double relVel  = state.relativeDockState.velocity.Magnitude();
            ImGui::Text("Dock Dist   %.3f m",   relDist);
            ImGui::Text("Dock Speed  %.3f m/s", relVel);
            ImGui::Spacing();
            ImGui::TextDisabled("Chaser attitude (q1..q4)");
            ImGui::Text("%.3f  %.3f  %.3f  %.3f",
                state.chaserECI.attitude.q1, state.chaserECI.attitude.q2,
                state.chaserECI.attitude.q3, state.chaserECI.attitude.q4);
            ImGui::TextDisabled("Target attitude (q1..q4)");
            ImGui::Text("%.3f  %.3f  %.3f  %.3f",
                state.targetECI.attitude.q1, state.targetECI.attitude.q2,
                state.targetECI.attitude.q3, state.targetECI.attitude.q4);
        }

        ImGui::Spacing();

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 2 — Coordinate System
        // ══════════════════════════════════════════════════════════════════════
        if (ImGui::CollapsingHeader("  Coordinate System", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Spacing();

            ImGui::Checkbox("Body Frames  (CoM)", &showBodyTriads);
            ImGui::Checkbox("Dock Port Frames",   &showDockTriads);
            ImGui::Spacing();
            ImGui::SliderFloat("Scale", &coordScale, 0.3f, 5.0f, "%.1fx");
            ImGui::Spacing();

            // Color legend
            float sq = 12.0f;
            ImGui::TextColored(ImVec4(1.0f,0.2f,0.2f,1.0f), "■"); ImGui::SameLine();
            ImGui::TextUnformatted("+X  Forward / Roll");
            ImGui::TextColored(ImVec4(0.2f,1.0f,0.2f,1.0f), "■"); ImGui::SameLine();
            ImGui::TextUnformatted("+Y  Pitch");
            ImGui::TextColored(ImVec4(0.3f,0.5f,1.0f,1.0f), "■"); ImGui::SameLine();
            ImGui::TextUnformatted("+Z  Yaw");
            (void)sq;
        }

        ImGui::Spacing();

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 3 — Pose Logger
        // ══════════════════════════════════════════════════════════════════════
        if (ImGui::CollapsingHeader("  Pose Logger  (2 ms / 500 Hz)")) {
            ImGui::Spacing();

            ImGui::SetNextItemWidth(-1);
            ImGui::InputText("##filename", logFilename, sizeof(logFilename));
            ImGui::Spacing();

            // Record / Stop buttons
            float half = (SIDEBAR_W - style.WindowPadding.x * 2 - style.ItemSpacing.x) / 2.0f;

            ImGui::PushStyleColor(ImGuiCol_Button,
                poseLogging ? ImVec4(0.15f,0.50f,0.15f,0.90f) : ImVec4(0.20f,0.35f,0.60f,0.80f));
            if (ImGui::Button("Record", ImVec2(half, 28))) {
                if (!poseLogging) {
                    poseLogging = true;
                    g_PoseLoggingEnabled.store(true);
                    logger.StartLogging(std::string(logFilename));
                }
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();

            ImGui::PushStyleColor(ImGuiCol_Button,
                !poseLogging ? ImVec4(0.60f,0.20f,0.15f,0.90f) : ImVec4(0.20f,0.35f,0.60f,0.80f));
            if (ImGui::Button("Stop & Save", ImVec2(half, 28))) {
                if (poseLogging) {
                    poseLogging = false;
                    g_PoseLoggingEnabled.store(false);
                    logger.StopLogging();
                }
            }
            ImGui::PopStyleColor();

            ImGui::Spacing();
            if (poseLogging)
                ImGui::TextColored(ImVec4(0.2f,0.9f,0.3f,1.0f), "● RECORDING");
            else
                ImGui::TextColored(ImVec4(0.5f,0.5f,0.5f,1.0f), "○ Stopped");

            ImGui::Text("Samples  %zu",  logger.GetSampleCount());
            ImGui::Text("Duration %.2f s", logger.GetLoggedDurationS());
            ImGui::Text("File: %s", logger.GetFilename().c_str());

            ImGui::Spacing();
            if (ImGui::Button("Flush to Disk", ImVec2(-1, 0)))
                logger.Flush();
        }

        ImGui::Spacing();

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 4 — Hint
        // ══════════════════════════════════════════════════════════════════════
        ImGui::TextDisabled("Camera: drag=orbit  scroll=zoom");

        ImGui::End(); // sidebar

        // ── Plot data accumulation (keep last 30 s) ────────────────────────
        double relDist = state.relativeDockState.position.Magnitude();
        double relVel  = state.relativeDockState.velocity.Magnitude();
        if (timeData.empty() || state.timestampS - timeData.back() > 0.05) {
            timeData.push_back(state.timestampS);
            distData.push_back(relDist);
            velData.push_back(relVel);
            if (timeData.size() > 600) {
                timeData.erase(timeData.begin());
                distData.erase(distData.begin());
                velData.erase(velData.begin());
            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        ctx.SwapBuffers();
    }

    g_SimRunning = false;
    simThread.join();

    logger.StopLogging();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    return 0;
}
