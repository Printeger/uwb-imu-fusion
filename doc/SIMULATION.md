# UWB-IMU Fusion — Simulation Pipeline Documentation

> **Last updated**: 2026-06-15  
> **Codebase**: `src/uwb-imu-fusion/simulator/`  
> **Status**: Verified against current code.

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Trajectory Generation](#3-trajectory-generation)
4. [Quadrotor Dynamics Simulation](#4-quadrotor-dynamics-simulation)
5. [SO(3) Geometric Controller](#5-so3-geometric-controller)
6. [IMU Sensor Simulation](#6-imu-sensor-simulation)
7. [UWB TWR Ranging Simulation](#7-uwb-twr-ranging-simulation)
8. [Data Formats \& Rosbag Topics](#8-data-formats--rosbag-topics)
9. [Launch \& Configuration](#9-launch--configuration)
10. [End-to-End Workflow](#10-end-to-end-workflow)

---

## 1. Overview

The simulation pipeline generates fully synthetic sensor data (IMU, UWB) with realistic
error models, enabling controlled validation of the UWB-IMU FGO system against known
ground truth.

**Key features**:

| Feature | Implementation |
|---------|---------------|
| Dynamics | 6-DOF rigid-body quadrotor, Boost `odeint` (RK-Dopri5) |
| Control | SO(3) geometric tracking controller (Lee et al., CDC 2010) |
| IMU | Specific force + angular velocity, Z-up convention matching Livox |
| UWB | TDMA DS-TWR scheduling, clock drift, per-link bias, NLOS, RSSI |
| Message format | Nooploop LinkTrack `LinktrackNodeframe3` (hardware-compatible) |
| Anchor layout | 8 anchors at 10m cube vertices, z ∈ {1, 5} m |
| Output rates | IMU/Odom: 200 Hz, UWB: 20 Hz |
| Visualization | RViz: anchors (spheres), ranging lines (color-coded), trajectory path |

---

## 2. System Architecture

```
circle_sim.launch
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│  trajectory_circle.py ──/position_cmd──→ so3_control (nodelet)  │
│  (100 Hz)                                     │                 │
│                                               │ /so3_cmd        │
│                                        ┌──────▼──────────────┐  │
│                                        │ quadrotor_sim_so3   │  │
│                                        │                     │  │
│                                        │  odeint (1000 Hz)   │  │
│                                        │  → R, v, ω update   │  │
│                                        │  → motor RPM lag    │  │
│                                        │                     │  │
│                                        │  Output (200 Hz):   │  │
│                                        │   /sim/odom (GT)    │  │
│                                        │   /sim/imu          │  │
│                                        └──────┬──────────────┘  │
│                                               │ /sim/odom       │
│                                        ┌──────▼──────────────┐  │
│                                        │ uwb_twr_sim         │  │
│                                        │                     │  │
│                                        │  TDMA (80 Hz slots) │  │
│                                        │  + error models     │  │
│                                        │  Output (20 Hz):    │  │
│                                        │   /nlink_linktrack  │  │
│                                        │   _nodeframe3       │  │
│                                        └─────────────────────┘  │
│                                                                  │
│  rosbag record -a  →  sim_circle.bag  →  uwb_imu_fgo batch FGO │
└──────────────────────────────────────────────────────────────────┘
```

**Node summary**:

| Node | Language | Internal Rate | Output Rate | Topics Published |
|------|----------|--------------|-------------|-----------------|
| `trajectory_generator` | Python | — | 100 Hz | `/position_cmd` |
| `so3_control` | C++ (nodelet) | — | 200 Hz | `/so3_cmd` |
| `quadrotor_simulator_so3` | C++ | 1000 Hz | 200 Hz | `/sim/imu`, `/sim/odom` |
| `uwb_twr_sim` | C++ | 80 Hz (slots) | 20 Hz | `/nlink_linktrack_nodeframe3`, `/uwb_sim/*` |
| `odom_to_path` | Python | — | 10 Hz | `/sim/path` |

---

## 3. Trajectory Generation

**Source**: `simulator/scripts/trajectory_circle.py`

### 3.1 Phase Sequence

| # | Phase | Duration | Description |
|---|-------|----------|-------------|
| 1 | Takeoff | 3.0 s | Cubic smooth-step $(0,0,0) \to (0,0,2.0)$ m |
| 2 | Hold | 2.0 s | Hover at $(0,0,2.0)$ m |
| 3 | Circle | 60.0 s | 3 loops, radius 3.0 m, period 20 s/loop |
| 4 | Return | 5.0 s | Smooth-step to $(0,0,2.0)$ m |
| 5 | Land | 3.0 s | Cubic descent to $(0,0,0)$ m |

Total: $\sim$73 s of flight.

### 3.2 Circle Phase Parameterization

$$
\begin{aligned}
x(t) &= R \cos(\omega t), & y(t) &= R \sin(\omega t), & z &= H \\[4pt]
\dot{x}(t) &= -R\omega \sin(\omega t), & \dot{y}(t) &= R\omega \cos(\omega t) \\[4pt]
\psi(t) &= \omega t + \pi/2, & \dot{\psi}(t) &= \omega
\end{aligned}
$$

with $R = 3.0$ m, $H = 2.0$ m, $\omega = 2\pi / 20$ rad/s.

### 3.3 Smooth Transitions

Cubic Hermite spline: $p(s) = p_0 + (p_1 - p_0)(3s^2 - 2s^3)$, $s = t/T \in [0,1]$.

### 3.4 ROS Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `takeoff_height` | 2.0 | Hover altitude (m) |
| `takeoff_duration` | 3.0 | Takeoff time (s) |
| `hold_duration` | 2.0 | Pre-circle hover (s) |
| `circle_radius` | 3.0 | Circle radius (m) |
| `circle_period` | 20.0 | Time per loop (s) |
| `num_circles` | 3 | Number of loops |
| `return_duration` | 5.0 | Return time (s) |
| `land_duration` | 3.0 | Landing time (s) |
| `rate` | 100 | Publish rate (Hz) |

### 3.5 Output Message

`uwb_imu_fgo/PositionCommand` with fields:

| Field | Description |
|-------|-------------|
| `position` | Desired world-frame position |
| `velocity` | Desired world-frame velocity |
| `acceleration` | Desired acceleration (unused) |
| `yaw` / `yaw_dot` | Desired heading and rate |
| `kx`, `kv` | Per-axis gain overrides (unused → controller defaults) |

---

## 4. Quadrotor Dynamics Simulation

**Source**: `simulator/quadrotor_dynamics/`

### 4.1 Physical Model

The quadrotor state is 22-dimensional (for the ODE integrator):

$$
\mathbf{s} = [\underbrace{x,y,z}_{3},\; \underbrace{v_x,v_y,v_z}_{3},\; \underbrace{R_{11},\ldots,R_{33}}_{9},\; \underbrace{\omega_x,\omega_y,\omega_z}_{3},\; \underbrace{\text{rpm}_{1\ldots4}}_{4}]
$$

**Equations of motion**:

| Quantity | Equation |
|----------|----------|
| Position | $\dot{\mathbf{x}} = \mathbf{v}$ |
| Velocity | $\dot{\mathbf{v}} = \frac{1}{m}\left(\mathbf{R}\,\mathbf{F}_{\text{thrust}} + \mathbf{F}_{\text{ext}}\right) + \mathbf{g}$ |
| Rotation | $\dot{\mathbf{R}} = \mathbf{R}\,[\boldsymbol{\omega}]_{\times}$ |
| Angular rate | $\mathbf{J}\,\dot{\boldsymbol{\omega}} = -\boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega}) + \mathbf{M}_{\text{thrust}} + \mathbf{M}_{\text{ext}}$ |
| Motor RPM | $\dot{\text{rpm}}_i = k_m\,(\text{rpm}_{\text{cmd},i} - \text{rpm}_i)$ |

where $\mathbf{g} = (0,\,0,\,-9.81)$ m/s².

**Thrust and moment from motors**:

$$
\begin{aligned}
F_i &= k_f \cdot \text{rpm}_i^2, \quad M_i = k_m \cdot \text{rpm}_i^2 \\[4pt]
\mathbf{F}_{\text{thrust}} &= \begin{bmatrix}0 \\ 0 \\ \sum_{i=1}^4 F_i\end{bmatrix} \\[4pt]
\mathbf{M}_{\text{thrust}} &= \begin{bmatrix}
d\cdot(F_4 - F_2) \\
d\cdot(F_3 - F_1) \\
M_1 + M_3 - M_2 - M_4
\end{bmatrix}
\end{aligned}
$$

### 4.2 Rotor Layout

```
        *1*    Front (CW)
      3     4
         2        Rear (CW)

Motors 1 & 2: Clockwise
Motors 3 & 4: Counter-Clockwise
```

### 4.3 Physical Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | $m$ | 0.98 | kg |
| Gravity | $g$ | 9.81 | m/s² |
| Roll inertia | $I_{xx}$ | $2.64\times10^{-3}$ | kg·m² |
| Pitch inertia | $I_{yy}$ | $2.64\times10^{-3}$ | kg·m² |
| Yaw inertia | $I_{zz}$ | $4.96\times10^{-3}$ | kg·m² |
| Arm length | $d$ | 0.26 | m |
| Propeller radius | $r$ | 0.062 | m |
| Thrust coefficient | $k_f$ | $8.98132\times10^{-9}$ | N/rpm² |
| Moment coefficient | $k_m$ | $0.07\cdot(3r)\cdot k_f$ | N·m/rpm² |
| Motor time constant | $\tau_m$ | 1/30 | s |
| Min RPM | — | 1200 | rpm |
| Max RPM | — | 35000 | rpm |

### 4.4 Numerical Integration

| Property | Value |
|----------|-------|
| Integrator | Boost `odeint` with default stepper (Runge-Kutta-Dopri5, adaptive) |
| Internal step | 1000 Hz (parameter `rate/simulation`) |
| Output rate | 200 Hz (parameter `rate/odom`) |
| SO(3) maintenance | Polar decomposition orthonormalization after each step |
| Floor constraint | $z \ge 0$, clamp $v_z \ge 0$ at contact |
| NaN guard | Rollback to previous state if NaN detected |

### 4.5 External Disturbances

Two optional subscribers allow injecting external force and moment:

| Topic | Type | Description |
|-------|------|-------------|
| `force_disturbance` | `geometry_msgs/Vector3` | External force in world frame (N) |
| `moment_disturbance` | `geometry_msgs/Vector3` | External moment in body frame (N·m) |

These are zero by default (no disturbances).

---

## 5. SO(3) Geometric Controller

**Source**: `simulator/so3_control/`  
**Reference**: T. Lee, M. Leok, N. H. McClamroch, *"Geometric Tracking Control of a Quadrotor UAV on SE(3)"*, CDC 2010.

### 5.1 Architecture

```
/position_cmd ──→ SO3ControlNodelet ──→ /so3_cmd (SO3Command)
       /sim/odom ──→     │
       /sim/imu  ──→ (yaw only)
```

The nodelet subscribes to position commands and odometry, then produces
SO(3) attitude commands consumed by the quadrotor simulator.

### 5.2 Control Law

**Position/Velocity error**:

$$
\mathbf{e}_p = \mathbf{p}_{\text{des}} - \mathbf{p}, \quad
\mathbf{e}_v = \mathbf{v}_{\text{des}} - \mathbf{v}, \quad
\mathbf{e}_a = \mathbf{a}_{\text{des}} - \mathbf{a}
$$

**Force command** (world frame):

$$
\mathbf{F} = m\,g\,\mathbf{e}_z + \mathbf{K}_p\,\mathbf{e}_p + \mathbf{K}_v\,\mathbf{e}_v + m\,\mathbf{K}_a\,\mathbf{e}_a + m\,\mathbf{a}_{\text{des}}
$$

$\mathbf{K}_a$ is adaptive: $\mathbf{K}_a = \text{diag}(0.2 \cdot |\mathbf{e}_p + \mathbf{e}_v + \mathbf{e}_a|)$, capped at zero when any axis error exceeds 3 m.

**Attitude command** (from thrust direction + desired yaw):

$$
\mathbf{b}_{3d} = \frac{\mathbf{F}}{\|\mathbf{F}\|}, \quad
\mathbf{b}_{2d} = \frac{\mathbf{b}_{3d} \times [\cos\psi,\;\sin\psi,\;0]^T}{\|\cdots\|}, \quad
\mathbf{b}_{1d} = \mathbf{b}_{2d} \times \mathbf{b}_{3d}
$$

**Tilt limit**: $\theta_{\max} = 45^\circ$. If the thrust direction exceeds this angle
relative to vertical, the force vector is re-projected.

### 5.3 Control Gains

**From `gains.yaml` (overridden by launch params for Z-axis)**:

| Axis | Position $k_p$ | Velocity $k_v$ | Rotation $k_R$ | Angular $k_{\Omega}$ |
|------|---------------|---------------|---------------|---------------------|
| X | 2.0 | 1.8 | 1.0 | 0.07 |
| Y | 2.0 | 1.8 | 1.0 | 0.07 |
| Z | 3.5 → **1.0** | 2.0 | 0.3 → **1.0** | 0.02 → **0.1** |

Values after "→" are overrides from `circle_sim.launch` (`gains/rot/z` and `gains/ang/z`).

**Corrections** (`corrections.yaml`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `z` | 0.0 | Thrust coefficient correction |
| `r` | 0.0 | Roll angle correction |
| `p` | 0.0 | Pitch angle correction |

### 5.4 SO3Command Message

| Field | Description |
|-------|-------------|
| `force` | Desired total thrust in world frame (N) |
| `orientation` | Desired attitude as quaternion |
| `kR[3]` | Rotation feedback gains |
| `kOm[3]` | Angular rate feedback gains |
| `aux.kf_correction` | Thrust coefficient adjustment |
| `aux.use_external_yaw` | Enable external yaw source |

---

## 6. IMU Sensor Simulation

**Source**: `quadrotor_simulator_so3.cpp`, function `quadToImuMsg()`  
**Output**: `sensor_msgs/Imu` on `/sim/imu` at 200 Hz.

### 6.1 Measurement Model

The simulated IMU produces **specific force** (proper acceleration + gravity in body frame)
and **angular velocity**, with a Z-axis sign flip to match the Livox IMU convention (Z-up).

**Angular velocity** (body frame → Z-up convention):

$$
\boldsymbol{\omega}_{\text{imu}} = \begin{bmatrix} \omega_x \\ \omega_y \\ -\omega_z \end{bmatrix}
$$

**Specific force** (body frame → Z-up convention):

$$
\mathbf{a}_{\text{body}} = \mathbf{a}_{\text{proper}} - \mathbf{R}^T\,\mathbf{g}_{\text{world}}
$$

$$
\mathbf{a}_{\text{imu}} = \begin{bmatrix} a_{\text{body},x} \\ a_{\text{body},y} \\ -a_{\text{body},z} \end{bmatrix}
$$

where $\mathbf{g}_{\text{world}} = (0,\,0,\,+9.81)$ is the gravity vector in world frame,
and $\mathbf{R}^T$ is the world-to-body rotation.

### 6.2 Hover Verification

In perfect hover ($\mathbf{R} = \mathbf{I}$, $\mathbf{a}_{\text{proper}} = \mathbf{0}$):

$$
\mathbf{a}_{\text{imu}} = \begin{bmatrix} 0 \\ 0 \\ -(-9.81) \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ +9.81 \end{bmatrix}\ \text{m/s}^2
$$

A positive $+9.81$ m/s² on the Z-axis at hover — matching the Livox convention.

> ⚠️ **IMU acceleration is in m/s²**, NOT g-units. The FGO config must set  
> `imu_acc_in_g: false` for simulation data (vs. `true` for real Livox IMU).

### 6.3 IMU Message Fields

| Field | Content | Unit |
|-------|---------|------|
| `linear_acceleration.x/y/z` | Specific force (Z-up body frame) | m/s² |
| `angular_velocity.x/y/z` | Body angular rate (Z-up) | rad/s |
| `orientation.x/y/z/w` | Ground-truth body-to-world quaternion | — |
| `header.frame_id` | `"simulator"` | — |
| `header.stamp` | `ros::Time::now()` | — |

---

## 7. UWB TWR Ranging Simulation

**Source**: `simulator/src/uwb_twr_sim.cpp`  
**Design document**: `doc/uwb_sim.tex`

### 7.1 TDMA Protocol

Time-Division Multiple Access with a repeating superframe:

| Parameter | Symbol | Value | Derivation |
|-----------|--------|-------|------------|
| Publish rate | $f_{\text{pub}}$ | **20 Hz** | User-configured |
| Superframe period | $T_{\text{sf}}$ | $1 / f_{\text{pub}} = \mathbf{0.05}$ s | Auto-aligned |
| Number of anchors | $N$ | **8** | Configurable |
| Slot duration | $\tau$ | $T_{\text{sf}} / N = \mathbf{6.25}$ ms | Per anchor |
| Message air time | $t_{\text{msg}}$ | **1.5** ms | DS-TWR = 3 msgs |
| Messages per ranging | $m$ | **3** (DS-TWR) | Poll → Response → Final |
| Anchors per slot | $k_{\max}$ | $\lfloor \tau / (m \cdot t_{\text{msg}}) \rfloor = \mathbf{1}$ | Capacity limit |

**DS-TWR (Double-Sided Two-Way Ranging) sequence per slot**:

```
Tag  ──── Poll ────→  Anchor
Tag  ←── Response ──  Anchor
Tag  ──── Final ───→  Anchor
```

Each slot, the tag ranges with $k_{\max}$ anchors. All measurements are accumulated
in a `pending_map_` (deduplicated by anchor ID). At the end of each superframe, the
accumulated data is published as a single `LinktrackNodeframe3` at 20 Hz.

Under LOS conditions with $k_{\max}=1$, all 8 anchors appear in every published frame.

### 7.2 Ranging Measurement Model

The complete measurement model for anchor $j$ in slot $i$:

$$
\boxed{\;\tilde{d}_{ij} = d_{ij} \cdot (1 + \varepsilon_{\text{clock}}) \;+\; b_0 \;+\; b_{ij}^{\text{link}} \;+\; \mathbf{1}_{\text{NLOS}}\!\cdot\!\eta_{\text{NLOS}} \;+\; \mathcal{N}(0,\,\sigma^2)\;}
$$

where $d_{ij} = \|\mathbf{p}_{\text{tag}} - \mathbf{p}_{\text{anchor},j}\|$ is the true Euclidean distance.

### 7.3 Error Components

#### 7.3.1 Clock Drift

Each anchor accumulates independent clock drift per slot:

$$
c_k \;\leftarrow\; c_k + \text{ppm} \times 10^{-6} \times \tau
$$

The relative clock error between the tag (anchor $i$) and ranging anchor $j$:

$$
\varepsilon_{\text{clock}} = (c_i - c_j) \cdot c_{\text{mode}}
$$

| Mode | $c_{\text{mode}}$ | Description |
|------|-------------------|-------------|
| DS-TWR | **0.05** | Compensated — ~5% residual |
| SS-TWR | 2.0 | Uncompensated — full clock error |

#### 7.3.2 Per-Link Bias

$$
b_{ij}^{\text{link}} \sim \mathcal{U}(-b_{\max},\, +b_{\max})
$$

Generated once at startup for each $(i,j)$ pair, held constant throughout the run.

#### 7.3.3 NLOS Bias

$$
\eta_{\text{NLOS}} \sim \text{Exp}(\lambda), \quad \lambda = 1 / \mu_{\text{NLOS}}
$$

Always **positive** — NLOS only extends the measured distance. Triggered randomly
with probability $P_{\text{NLOS}}$ at each measurement.

#### 7.3.4 Gaussian Noise

$$
\mathcal{N}(0,\, \sigma^2), \quad \sigma = 0.10\ \text{m (configurable)}
$$

White Gaussian noise added to every LOS measurement.

#### 7.3.5 Packet Loss

Each measurement is independently dropped with probability $P_{\text{loss}}$. Dropped
measurements are simply skipped.

### 7.4 Error Model Parameters

| Parameter | YAML Key | Default | Description |
|-----------|----------|---------|-------------|
| **Noise std** | `range_noise_std` | **0.10 m** | LOS Gaussian σ |
| **Global bias** | `range_bias_const` | **0.0 m** | Constant offset $b_0$ |
| **Per-link bias amplitude** | `per_link_bias_max` | **0.0 m** | Link bias range $b_{\max}$ |
| **Clock drift** | `clock_ppm` | **20.0 ppm** | Frequency error |
| **Packet loss** | `packet_loss_prob` | **0.02** | Per-measurement prob. |
| **NLOS probability** | `nlos_probability` | **0.05** | Random NLOS trigger |
| **NLOS bias mean** | `nlos_bias_mean` | **0.40 m** | $\mu_{\text{NLOS}}$ for Exp(λ) |
| **Max range** | `max_range` | **80.0 m** | Measurements beyond dropped |
| **Odom timeout** | `odom_timeout` | **0.20 s** | Staleness threshold |

### 7.5 RSSI Model

#### 7.5.1 Received RSSI

Log-distance path loss with log-normal shadowing:

$$
\text{RSSI} = P_{\text{tx}} - 10\,n\,\log_{10}\!\left(\frac{d}{d_0}\right) + \mathcal{N}(0,\,\sigma_{\text{sh}}^2)
$$

#### 7.5.2 First-Path RSSI

$$
\text{fp\_rssi} = \text{RSSI} - \Delta_{\text{fp}}
$$

$$
\Delta_{\text{fp}} = \begin{cases}
\Delta_{\text{LOS}} & \text{(LOS)} \\[4pt]
\Delta_{\text{LOS}} + \Delta_{\text{NLOS}} & \text{(NLOS)}
\end{cases}
$$

The difference $\text{rx\_rssi} - \text{fp\_rssi}$ is used at the FGO side for NLOS
detection (threshold: 6 dB).

#### 7.5.3 RSSI Parameters

| Parameter | YAML Key | Default | Description |
|-----------|----------|---------|-------------|
| TX power | `rssi_tx_power` | **−45.0 dBm** | $P_{\text{tx}}$ |
| Path loss exponent | `rssi_path_loss_n` | **2.2** | $n$ |
| Reference distance | `rssi_ref_dist` | **1.0 m** | $d_0$ |
| Shadowing std | `rssi_shadowing_std` | **2.0 dB** | $\sigma_{\text{sh}}$ |
| FP offset (LOS) | `fp_offset_los` | **3.0 dB** | $\Delta_{\text{LOS}}$ |
| FP offset (NLOS) | `fp_offset_nlos` | **8.0 dB** | $\Delta_{\text{NLOS}}$ |

### 7.6 Anchor Layout

8 anchors at the vertices of a 10 m cube, with $z \in \{1.0,\;5.0\}$ m:

| ID | X (m) | Y (m) | Z (m) |
|----|-------|-------|-------|
| 1 | −5.0 | −5.0 | 1.0 |
| 2 | −5.0 | −5.0 | 5.0 |
| 3 | −5.0 | +5.0 | 1.0 |
| 4 | −5.0 | +5.0 | 5.0 |
| 5 | +5.0 | −5.0 | 1.0 |
| 6 | +5.0 | −5.0 | 5.0 |
| 7 | +5.0 | +5.0 | 1.0 |
| 8 | +5.0 | +5.0 | 5.0 |

### 7.7 Visualization

Two RViz marker topics are published:

| Topic | Type | Rate | Content |
|-------|------|------|---------|
| `/uwb_sim/anchor_markers` | `Marker` (SPHERE + TEXT) | 10 Hz, latched | Orange spheres + ID labels |
| `/uwb_sim/range_lines` | `Marker` (LINE_LIST) | 10 Hz | Color-coded lines (green=LOS, blue=NLOS) |

---

## 8. Data Formats & Rosbag Topics

### 8.1 Rosbag Topic Summary

| Topic | Message Type | Rate | Source Node |
|-------|-------------|------|-------------|
| `/sim/imu` | `sensor_msgs/Imu` | 200 Hz | `quadrotor_simulator_so3` |
| `/sim/odom` | `nav_msgs/Odometry` | 200 Hz | `quadrotor_simulator_so3` |
| `/nlink_linktrack_nodeframe3` | `LinktrackNodeframe3` | 20 Hz | `uwb_twr_sim` |
| `/position_cmd` | `PositionCommand` | 100 Hz | `trajectory_circle.py` |
| `/so3_cmd` | `SO3Command` | 200 Hz | `so3_control` |
| `/sim/path` | `nav_msgs/Path` | — | `odom_to_path.py` |
| `/uwb_sim/anchor_markers` | `Marker` | 10 Hz (latched) | `uwb_twr_sim` |
| `/uwb_sim/range_lines` | `Marker` | 10 Hz | `uwb_twr_sim` |
| `/tf` | `TFMessage` | 200 Hz | `quadrotor_simulator_so3` |

### 8.2 IMU (`/sim/imu`)

| Field | Value | Unit |
|-------|-------|------|
| `linear_acceleration` | Specific force, Z-up body frame | **m/s²** |
| `angular_velocity` | Body rate, Z-up | **rad/s** |
| `orientation` | GT body-to-world quaternion | — |
| `header.frame_id` | `"simulator"` | — |

Hover check: `acc.z ≈ +9.81` m/s².

### 8.3 Ground Truth (`/sim/odom`)

| Field | Content |
|-------|---------|
| `pose.pose.position` | World-frame position |
| `pose.pose.orientation` | World-frame attitude (quaternion) |
| `twist.twist.linear` | World-frame velocity |
| `twist.twist.angular` | Body-frame angular velocity |
| `header.frame_id` | `"world"` |
| `child_frame_id` | `"quadrotor"` |

### 8.4 UWB (`/nlink_linktrack_nodeframe3`)

| Field | Type | Description |
|-------|------|-------------|
| `header.stamp` | `time` | Publish timestamp (20 Hz) |
| `id` | `uint8` | Tag ID (= 1) |
| `role` | `uint8` | Node role (= 0) |
| `local_time` | `uint64` | Local time (µs) |
| `system_time` | `uint32` | System time (ms) |
| `voltage` | `float32` | Battery (4.10 V) |
| `nodes[i].id` | `uint16` | Anchor ID |
| `nodes[i].dis` | `float32` | Range measurement (m) |
| `nodes[i].fp_rssi` | `float32` | First-path RSSI (dBm) |
| `nodes[i].rx_rssi` | `float32` | Received RSSI (dBm) |

### 8.5 TUM Output Files (FGO stage)

| File | Format | Content |
|------|--------|---------|
| `trajectory.txt` | `t x y z qx qy qz qw` | FGO estimated trajectory |
| `groundtruth.txt` | `t x y z qx qy qz qw` | GT from `/sim/odom` |

---

## 9. Launch & Configuration

### 9.1 Launching the Simulation

```bash
roslaunch uwb_imu_fgo circle_sim.launch [init_x:=0] [init_y:=0] [init_z:=0] [rviz:=true]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `init_x/y/z` | 0.0 | Initial drone position (m) |
| `rviz` | true | Enable RViz visualization |

### 9.2 UWB Simulator Config

**File**: `simulator/config/uwb_twr_sim.yaml`

Key parameters (see §7.4 and §7.5 for details):

```yaml
num_anchors: 8
pub_rate: 20.0
ranging_mode: "DS"
range_noise_std: 0.10
clock_ppm: 20.0
packet_loss_prob: 0.02
nlos_probability: 0.05
nlos_bias_mean: 0.40
max_range: 80.0
```

### 9.3 Controller Gains

**File**: `simulator/so3_control/gains.yaml`

```yaml
gains:
  pos: {x: 2.0, y: 2.0, z: 3.5}
  vel: {x: 1.8, y: 1.8, z: 2.0}
  rot: {x: 1.0, y: 1.0, z: 0.3}
  ang: {x: 0.07, y: 0.07, z: 0.02}
```

Z-axis override in `circle_sim.launch`: `gains/rot/z=1.0`, `gains/ang/z=0.1`.

### 9.4 FGO Processing Config

**File**: `config/sim_circle.yaml`

| Critical Parameter | Value | Reason |
|--------------------|-------|--------|
| `topics.imu` | `/sim/imu` | Simulation IMU topic |
| `topics.uwb` | `/nlink_linktrack_nodeframe3` | Simulation UWB topic |
| `topics.gt_odom` | `/sim/odom` | GT source for evaluation |
| `imu.imu_acc_in_g` | **`false`** | IMU already in m/s² |
| `keyframe.step` | 10 | ~2 Hz keyframe rate |
| `keyframe.yaw_align_frames` | 15 | Yaw grid search |
| `uwb.sigma_range` | 0.10 | Matches simulation noise |

---

## 10. End-to-End Workflow

### Step 1: Build

```bash
cd ~/ws_fusion_uwb
catkin build uwb_imu_fgo --cmake-args -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM
source devel/setup.bash
```

### Step 2: Launch Simulation

```bash
roslaunch uwb_imu_fgo circle_sim.launch
```

### Step 3: Record Rosbag (separate terminal)

```bash
cd src/uwb-imu-fusion/data
rosbag record -a -o ./sim_circle.bag
# Wait ~85 s, then Ctrl+C
```

### Step 4: Configure & Run FGO

Edit `config/sim_circle.yaml` — update `bag.path` to the newly recorded file.

```bash
roslaunch uwb_imu_fgo offline_with_viz.launch
```

### Step 5: Analyze

```bash
python3 tools/analyze_log.py
# Open logs/latest/report.md
```

### Expected Results (8-anchor, full LOS, $\sigma=0.10$ m)

| Metric | Typical Value |
|--------|---------------|
| ATE RMSE | < 0.10 m |
| UWB Residual RMSE | ~0.087 m |
| UWB Inlier Ratio | 100% |
| Keyframes | 160 |
| Total Factors | ~1357 |
| Batch Runtime | ~73 s |
