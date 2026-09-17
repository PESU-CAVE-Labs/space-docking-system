# Spacecraft Docking Simulator VDE

This is a comprehensive software-in-the-loop (SIL) 6-DOF orbital dynamics simulator for spacecraft docking, completely replacing the original hardware-in-the-loop (HIL) air-bearing table constraint.

The simulator features a robust physics engine running in a dedicated background thread with a real-time native OpenGL Visualization & Debugging Environment (VDE).

## Architecture & Features

### Phase 1: Core Mathematics & Frames
- Fully custom rigid body math utility library: `Vec3`, `Mat3` (DCM), `Quaternion` (scalar-last), `Euler` (3-2-1 sequence).
- `Frame` and `FrameTransformer` architectures to derive Relative Docking States between two bodies in ECI (Earth-Centered Inertial) frame.

### Phase 2: Core Dynamics Engine & Integrator
- Complete 6-DOF `RigidBody` implementation.
- Customizable `IIntegrator` with a verified `RK4Integrator` implementation handling high-precision state integration at 500Hz.
- `TwoBodyGravity` implementation using a precise `kEarthMu`.

### Phase 3: Perturbation & Disturbance Models
The environment is thoroughly modelled with multiple selectable disturbance models extending the `IDisturbanceModel` interface:
- **Atmospheric Drag** (`AtmosphericDragModel`)
- **Gravity Gradient Torque** (`GravityGradientTorqueModel`)
- **J2 Perturbations** (`J2PerturbationModel`)

### Phase 4: Contact Dynamics (Docking Mechanism)
- A highly stiff `ContactSpringModel` (Linear Spring-Damper) handles the physical collision between spacecraft docking frames, ensuring they physically interact (and bounce) without clipping.

### Phase 5 & 6: VDE (Visualization & Debugging Environment)
- Replaced the dependency on massive 3rd party engines (like Unity) by building a native OpenGL 3.3 Core Profile renderer from scratch.
- Features `GLContext`, `Shader`, `MeshFactory` for drawing the Chaser and Target geometry.
- Smooth orbital camera controls (Mouse drag to pan, Scroll to zoom).

### Phase 7 & 8: Real-Time Architecture & Telemetry Logging
- Architecture decoupled into a Physics Thread (running at a fixed 500Hz timestep, throttled to real-time) and a main GUI Thread.
- Thread-safe `SimState` snapshotting.
- Live HUD displaying critical state telemetry.
- **DataLogger**: Automatically records high-frequency telemetry across the entire simulation run to `build/simulation_data.csv` for post-flight analysis.

## Build and Run Instructions

### Prerequisites
- CMake 3.15+
- C++17 Compiler (Clang/GCC)
- macOS (Supports OpenGL Forward-Compat mode)

### Compiling
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Running
```bash
./docking_sim
```

### Controls
- **Left-Click + Drag**: Pan the camera orbit around the Target spacecraft.
- **Scroll Wheel**: Zoom the camera in and out.
- **CSV Data**: Upon exiting the simulation, you will find `simulation_data.csv` populated with your run's telemetry data in the `build/` directory.
