# Rigid Body Collision & Bungee Physics Simulator

An interactive browser-based simulator for demonstrating **rigid-body motion, collision response, rotational dynamics, quaternions, bungee/rope physics, and coordinate-frame transformations**.

The simulator contains two cubes:

* 🔵 **Blue Cube** — suspended from a fixed bungee/rope attachment point.
* 🟡 **Yellow Cube** — user-controlled rigid body that can be dragged into the blue cube.

When the cubes collide, the simulator calculates a collision impulse and updates both **linear and angular motion**. The resulting pose is integrated using quaternions.

The simulator also displays dynamically changing position information in:

* **Relative coordinates**
* **ECEF — Earth-Centered, Earth-Fixed**
* **ECI — Earth-Centered Inertial**

---

## Features

### 1. Interactive Rigid Bodies

Both cubes are modeled as rigid bodies with:

* Position
* Orientation
* Linear velocity
* Angular velocity
* Mass
* Moment of inertia

The yellow cube can be directly manipulated using the mouse.

---

### 2. Bungee / Rope Physics

The blue cube is attached to a fixed anchor point.

The bungee behaves as an elastic rope:

$$
L = \|\mathbf{x}_{anchor}-\mathbf{x}_{cube}\|
$$

where \(L\) is the instantaneous rope length.

The rope has a natural length \(L_0\).

If:

$$
L \leq L_0
$$

the rope is slack and no tension is applied.

If:

$$
L > L_0
$$

the rope produces a restoring force.

The extension is:

$$
\Delta L = L-L_0
$$

Using Hooke's law:

$$
F_s=k\Delta L
$$

The force direction is along the rope:

$$
\hat{\mathbf{d}} =
\frac{\mathbf{x}_{anchor}-\mathbf{x}_{cube}}
{\|\mathbf{x}_{anchor}-\mathbf{x}_{cube}\|}
$$

Therefore:

$$
\mathbf{F}_{spring} =
k\Delta L\hat{\mathbf{d}}
$$

---

### 3. Bungee Damping

Damping is applied along the rope direction.

The velocity component along the rope is:

$$
v_r=\mathbf{v}\cdot\hat{\mathbf{d}}
$$

The damping force is:

$$
\mathbf{F}_{damping}=
-cv_r\hat{\mathbf{d}}
$$

The total bungee force becomes:

$$
\boxed{
\mathbf{F}=
(k\Delta L-cv_r)\hat{\mathbf{d}}
}
$$

The elasticity and damping values can be controlled through the user interface.

---

## Collision Detection

The cubes are oriented rigid bodies, so collision detection uses the **Oriented Bounding Box (OBB) Separating Axis Test (SAT)** rather than a simple axis-aligned bounding box.

This allows collision detection to continue working even when either cube has rotated.

### Separating Axis Theorem

For two convex objects, if a separating axis exists, the objects do not intersect.

For two boxes, candidate axes include:

1. Three local axes of the first box
2. Three local axes of the second box
3. Nine cross products between the two sets of axes

Therefore, the candidate set contains up to:

$$
3+3+9=15
$$

axes.

For each candidate axis, the projection radius of each box is calculated.

If:

$$
|C_2-C_1| > R_1+R_2
$$

then the boxes are separated along that axis.

If no separating axis exists, the boxes are colliding.

---

# Collision Response

Once a collision is detected, the simulator calculates a collision impulse.

The collision normal points from the blue cube toward the yellow cube.

---

## Contact Point

A contact point is estimated from the supporting points of the two OBBs.

The vectors from each cube's center of mass to the contact point are:

$$
\mathbf{r}_1=
\mathbf{x}_{contact}-\mathbf{x}_1
$$

$$
\mathbf{r}_2=
\mathbf{x}_{contact}-\mathbf{x}_2
$$

These vectors are important because an impulse applied away from the center of mass generates rotation.

---

# Contact Velocity

The velocity at a point on a rigid body is:

$$
\boxed{
\mathbf{v}_p=
\mathbf{v}_{CM} + \boldsymbol{\omega}\times\mathbf{r}
}
$$

For the blue cube:

$$
\mathbf{v}_1=
\mathbf{v}_{blue} + \boldsymbol{\omega}_{blue}
\times
\mathbf{r}_1
$$

For the yellow cube:

$$
\mathbf{v}_2 = \mathbf{v}_{yellow} + \boldsymbol{\omega}_{yellow}
\times
\mathbf{r}_2
$$

The relative contact velocity is:

$$
\mathbf{v}_{rel} = \mathbf{v}_2-\mathbf{v}_1
$$

The velocity along the collision normal is:

$$
v_n=
\mathbf{v}_{rel}\cdot\mathbf{n}
$$

A collision impulse is required when:

$$
v_n<0
$$

meaning that the objects are moving toward one another at the contact point.

---

# Collision Impulse

The impulse magnitude is calculated using:

$$
j=
-\frac{(1+e)v_n}
{
m_1^{-1}
+
m_2^{-1}
+
I_1^{-1}|\mathbf{r}_1\times\mathbf{n}|^2
+
I_2^{-1}|\mathbf{r}_2\times\mathbf{n}|^2
}
$$

where:

* \(j\) = impulse magnitude
* \(e\) = coefficient of restitution / elasticity
* \(m\) = mass
* \(I\) = moment of inertia
* \(\mathbf{r}\) = center-of-mass to contact vector
* \(\mathbf{n}\) = collision normal

The impulse vector is:

$$
\boxed{
\mathbf{J}=j\mathbf{n}
}
$$

---

# Linear Velocity Update

The impulse changes the linear velocities.

For the blue cube:

$$
\mathbf{v}_1' = \mathbf{v}_1 - \frac{\mathbf{J}}{m_1}
$$

For the yellow cube:

$$
\mathbf{v}_2' = \mathbf{v}_2 + \frac{\mathbf{J}}{m_2}
$$

Thus, momentum is transferred between the two bodies.

---

# Angular Velocity Update

An impulse applied away from the center of mass produces torque-like rotational change.

The change in angular momentum is:

$$
\Delta\mathbf{L} = \mathbf{r}\times\mathbf{J}
$$

Using:

$$
\mathbf{L}=I\boldsymbol{\omega}
$$

the change in angular velocity is:

$$
\boxed{
\Delta\boldsymbol{\omega} = I^{-1} (\mathbf{r}\times\mathbf{J})
}
$$

For the blue cube:

$$
\boldsymbol{\omega}_1' = \boldsymbol{\omega}_1 + I_1^{-1} (\mathbf{r}_1\times\mathbf{J})
$$

For the yellow cube:

$$
\boldsymbol{\omega}_2' = \boldsymbol{\omega}_2 - I_2^{-1} (\mathbf{r}_2\times\mathbf{J})
$$

This is what allows a collision to change both **position and orientation**.

---

# Moment of Inertia

For a uniform cube:

$$
I_x=I_y=I_z
$$

For a cube with side length \(a\):

$$
\boxed{
I=\frac{m(a^2+a^2)}{12}
}
$$

or:

$$
\boxed{
I=\frac{ma^2}{6}
}
$$

The simulator uses this simplified scalar moment of inertia because the objects are cubes with uniform mass distribution.

---

# Quaternion Orientation

The simulator uses **quaternions** instead of Euler angles to represent orientation.

A quaternion is represented as:

$$
q=(x,y,z,w)
$$

where:

* \(x,y,z\) represent the vector component
* \(w\) represents the scalar component

Quaternions avoid the gimbal-lock problem associated with Euler-angle representations.

---

## Quaternion Angular Velocity

Angular velocity is represented as a pure quaternion:

$$
\omega_q=(\omega_x,\omega_y,\omega_z,0)
$$

The quaternion derivative is:

$$
\boxed{
\dot q= \frac{1}{2}q\otimes\omega_q
}
$$

where \(\otimes\) represents quaternion multiplication.

The simulator implements this using Three.js quaternion multiplication.

Conceptually:

```text
angular velocity
       ↓
pure quaternion
       ↓
q × ω
       ↓
multiply by 1/2
       ↓
q̇
       ↓
integrate
       ↓
new orientation
```

The quaternion is normalized after integration:

$$
q \leftarrow \frac{q}{\|q\|}
$$

This prevents numerical drift from causing the quaternion to cease being a unit quaternion.

---

# Numerical Integration

The simulator uses a simple explicit integration scheme.

For position:

$$
\mathbf{x}_{t+\Delta t} = \mathbf{x}_t + \mathbf{v}_t\Delta t
$$

For linear velocity:

$$
\mathbf{v}_{t+\Delta t} = \mathbf{v}_t + \mathbf{a}_t\Delta t
$$

For orientation:

$$
q_{t+\Delta t} = q_t+\dot q_t\Delta t
$$

followed by quaternion normalization.

---

# Angular Damping

Collision can introduce angular velocity.

A small amount of angular damping is applied to prevent persistent numerical rotation:

$$
\boldsymbol{\omega}_{new} = \boldsymbol{\omega}_{old} e^{-k_d\Delta t}
$$

where \(k_d\) is the angular damping coefficient.

This allows the cubes to gradually stop rotating after a collision.

---

# Positional Correction

When two bodies overlap, simply applying an impulse may leave some penetration because of numerical integration.

The simulator therefore applies a positional correction.

Conceptually:

$$
\Delta x \propto \text{penetration depth}
$$

The correction is divided between the two cubes to reduce overlap before the next simulation step.

This prevents the cubes from remaining embedded inside each other.

---

# Coordinate Systems

The simulator demonstrates the same physical state in different coordinate frames.

## 1. Relative Frame

The yellow cube's position can be expressed relative to the blue cube:

$$
\boxed{
\mathbf{p}_{rel} = \mathbf{p}_{yellow} - \mathbf{p}_{blue}
}
$$

This describes where the yellow cube is with respect to the blue cube.

---

## 2. ECEF

ECEF stands for:

**Earth-Centered, Earth-Fixed**

The coordinate system:

* Origin is at Earth's center.
* Rotates with Earth.
* Coordinates remain fixed relative to Earth's surface.

The simulator transforms the local simulation coordinates into an ECEF representation using a reference ECEF position and orientation.

---

## 3. ECI

ECI stands for:

**Earth-Centered Inertial**

The ECI frame is treated as non-rotating with respect to distant space.

The simulator demonstrates the transformation between ECEF and ECI using a time-varying Earth rotation angle.

For a simplified rotation about the Z-axis:

$$
\begin{bmatrix}
x_{ECI}\\
y_{ECI}\\
z_{ECI}
\end{bmatrix} =
\begin{bmatrix}
\cos\theta &-\sin\theta&0\\
\sin\theta &\cos\theta&0\\
0&0&1
\end{bmatrix}
\begin{bmatrix}
x_{ECEF}\\
y_{ECEF}\\
z_{ECEF}
\end{bmatrix}
$$

where \(\theta\) is the simulated Earth rotation angle.

> **Note:** The ECI/ECEF transformation in this educational simulator is a simplified visualization model and is not intended to replace a full astronomical Earth-orientation model.

---

# User Interface

The simulator provides controls for:

### Bungee Elasticity

Controls the strength of the spring.

```text
0%   → bungee disabled
100% → maximum configured elasticity
```

### Bungee Damping

Controls how quickly oscillations are dissipated.

```text
0%   → no damping
100% → maximum configured damping
```

### Rotation

Controls the visual/simulation rotation rate used for the ECI/ECEF demonstration.

---

# Mouse Controls

## Yellow Cube

The yellow cube can be dragged directly with the mouse.

```text
Left mouse button + drag on yellow cube
                ↓
        Move yellow cube
```

Collision detection remains active while dragging.

---

## Camera

The simulator uses orbit-style camera controls.

| Action                   | Function         |
| ------------------------ | ---------------- |
| Left drag on empty space | Orbit camera     |
| Mouse wheel              | Zoom             |
| Right drag               | Pan              |
| Left drag on yellow cube | Move yellow cube |

The camera interaction is disabled temporarily while the yellow cube is being dragged.

---

# Keyboard Controls

## Reset Yellow Cube

Press:

```text
R
```

The yellow cube is reset to:

$$
\boxed{
(0,0,0)
}
$$

Its orientation is reset to the identity quaternion and its linear and angular velocities are set to zero.

Both:

```text
R
```

and:

```text
r
```

are supported.

---

# Simulation Architecture

The simulation loop can be conceptually represented as:

```text
                 ┌──────────────────┐
                 │ User Interaction │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │ Bungee / Rope    │
                 │ Force            │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │ Linear Dynamics  │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │ OBB Collision    │
                 │ Detection        │
                 └────────┬─────────┘
                          │
                     Collision?
                      /       \
                    No         Yes
                    │           │
                    │           ▼
                    │    ┌───────────────┐
                    │    │ Impulse       │
                    │    │ Response      │
                    │    └───────┬───────┘
                    │            │
                    └─────┬──────┘
                          ▼
                 ┌──────────────────┐
                 │ Angular Dynamics │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │ Quaternion       │
                 │ Integration      │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │ Coordinate       │
                 │ Transformations  │
                 └────────┬─────────┘
                          │
                          ▼
                 ┌──────────────────┐
                 │ Three.js Render  │
                 └──────────────────┘
```

---

# Technology Stack

The simulator is implemented as a client-side web application.

### Core technologies

* HTML5
* JavaScript
* Three.js
* Three.js OrbitControls
* Tailwind CSS

No server-side physics engine is required.

---

# Running the Simulator

The simulator can be run directly in a modern web browser.

### Option 1 — Local file

Open the HTML file directly in a browser.

### Option 2 — Local HTTP server

For a more reliable development environment, serve the directory using a local HTTP server.

For example:

```bash
python -m http.server 8000
```

Then open:

```text
http://localhost:8000
```

---

# Suggested Repository Structure

```text
rigid-body-bungee-simulator/
│
├── index.html
├── README.md
└── assets/
    └── ...
```
# Physics Parameters

The current implementation uses simplified parameters suitable for an interactive educational demonstration.

Important parameters include:

| Parameter              | Description                     |
| ----------------------- | -------------------------------- |
| Mass                   | Mass of each cube               |
| Cube size              | Physical dimensions of the cube |
| Spring stiffness \(k\) | Bungee restoring force          |
| Damping \(c\)          | Bungee energy dissipation       |
| Rest length \(L_0\)    | Natural bungee length            |
| Elasticity \(e\)       | Collision restitution control   |
| Angular damping        | Numerical rotational damping     |
| Time step \(\Delta t\) | Simulation integration interval |

---

# Educational Objectives

This simulator can be used to demonstrate the relationship between several concepts that are often taught separately.

### Translational Dynamics

$$
\mathbf{F}=m\mathbf{a}
$$

### Rotational Dynamics

$$
\boldsymbol{\tau}=I\boldsymbol{\alpha}
$$

### Angular Momentum

$$
\mathbf{L}=I\boldsymbol{\omega}
$$

### Impulse

$$
\mathbf{J} =
\int \mathbf{F}\,dt
$$

### Collision Response

$$
\mathbf{v}'=\mathbf{v}+\frac{\mathbf{J}}{m}
$$

### Rotational Collision Response

$$
\Delta\boldsymbol{\omega} =
I^{-1}
(\mathbf{r}\times\mathbf{J})
$$

### Quaternion Orientation

$$
\dot q =
\frac{1}{2}q\otimes\omega_q
$$

### Spring Physics

$$
F=k\Delta L
$$

### Damping

$$
F_d=-cv
$$

### Coordinate Transformation

$$
\mathbf{p}_{ECI} =
R(\theta)\mathbf{p}_{ECEF}
$$

The simulator therefore provides a single interactive environment connecting **Newtonian mechanics, rigid-body dynamics, collision mechanics, quaternion mathematics, and spatial coordinate transformations**.

---

# Limitations

This simulator is intended primarily for **education and visualization**, rather than high-fidelity physical simulation.

The following simplifications are currently made:

1. Both bodies are uniform cubes.
2. The moment of inertia is simplified to a scalar value.
3. Collision contact geometry is approximated.
4. Friction and tangential collision impulses are not currently modeled.
5. The bungee is modeled as a spring-damper system and does not model material deformation in detail.
6. The bungee force is currently applied through the blue cube's center of mass, so it does not independently generate torque.
7. ECI/ECEF conversion is simplified for visualization.
8. Numerical integration is explicit and intended for interactive demonstration.
9. The simulation does not currently implement a complete constraint solver.

---

# Possible Extensions

The simulator can be extended with:

* Coulomb friction
* Tangential collision impulses
* Full inertia tensors
* World-space inertia tensor transformation
* Multiple contact points
* Contact manifolds
* More accurate box-box collision resolution
* True rope constraints
* Rope attachment points offset from the center of mass
* Torque generated by off-center rope attachment
* Gravity
* Air resistance
* Multiple rigid bodies
* Sphere-box and mesh collision
* Energy and momentum graphs
* Real astronomical GMST/GAST calculations
* Full ECI/ECEF conversion using Earth orientation parameters
* Verlet or semi-implicit Euler integration
* Constraint stabilization
* Energy conservation analysis
---
