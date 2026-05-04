# Piper Arm Research Simulation

Research-grade robot arm simulation in C++ using MuJoCo and the AgileX Piper 6-DOF arm.  
Designed for studying **disturbance estimation and rejection** at the torque level.

---

## Overview

The simulator provides a modular pipeline where every layer (disturbance injection, sensor noise, observer, controller) is independently configurable through a single YAML file.

```
MuJoCo (ground truth physics)
  │
  ├─ Mismatched disturbance   ← external forces, mass/gravity perturbation
  │
  ├─ Sensor layer             ← Gaussian noise, bias, drift, delay
  │
  ├─ Observer layer           ← DOB  /  Momentum Observer  /  Force Observer
  │
  ├─ PD Controller            ← Kp·e + Kd·ė  −  K_obs·d̂
  │
  ├─ Matched disturbance      ← actuator noise, constant offset, chirp
  │
  └─ Motor torque → MuJoCo
```

**Key properties**

- **Torque-controlled** — motor actuators, not position actuators; `ctrl[i]` = torque in Nm
- **Gravity-compensated internally** — `gravcomp="1"` on all links so the observer model reduces to `M(q)·q̈ = τ`
- **Three observer implementations** selectable at runtime via YAML:
  - `dob` — Q-filter Disturbance Observer (diagonal nominal inertia model)
  - `momentum` — Momentum Observer using full `M(q)` from MuJoCo (De Luca & Mattone 2003)
  - `force` — Force Observer (model-free low-pass filter on torque residual)
- All experiment parameters live in one YAML file — no recompilation needed to change disturbances, gains, or observers

---

## Dependencies

| Library | Version tested | Install |
|---|---|---|
| [MuJoCo](https://github.com/google-deepmind/mujoco) | 3.7.0 | see below |
| Eigen3 | 3.4.0 | `sudo apt install libeigen3-dev` |
| yaml-cpp | 0.7.0 | `sudo apt install libyaml-cpp-dev` |
| GLFW3 | 3.3.6 | `sudo apt install libglfw3-dev` |
| CMake | ≥ 3.18 | `sudo apt install cmake` |
| GCC | ≥ 11 (C++17) | `sudo apt install build-essential` |
| Python | ≥ 3.8 | for offline analysis (optional) |
| pandas | - | `pip install pandas` (for analysis) |
| matplotlib | - | `pip install matplotlib` (for plots) |

### Install system packages

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev libglfw3-dev
```

### Install MuJoCo from source

```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$HOME/mujoco/install
cmake --build build -j$(nproc)
cmake --install build
```

The CMakeLists.txt expects MuJoCo at `$HOME/mujoco/install`.  
Use `-DMUJOCO_ROOT=<path>` to override.

---

## Build

```bash
cd prj_arm

cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

cmake --build build -j$(nproc)
```

Output binary: `build/arm_sim`

---

## Run

### Build

Use VS Code's task system (Ctrl+Shift+B) or command line:

```bash
# Terminal command
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
```

Or invoke from VS Code:
- **Ctrl+Shift+B** — Run default build task (CMake: Configure + Build)
- **Ctrl+Shift+P → Run Task** — Select specific task

Available tasks:
- `CMake: Configure` — Run cmake configure step
- `CMake: Build` — Build executable (depends on Configure)
- `Simulator: Run (default config)` — Run with `config/piper_arm.yaml`
- `Simulator: Run (headless)` — Run without display server
- `Analysis: Run on latest log` — Run offline analysis with plots

### Run simulator

```bash
./build/arm_sim config/piper_arm.yaml
```

Press **Escape** in the visualizer window to stop early.

To edit parameters, modify `config/piper_arm.yaml` and re-run.

### Offline analysis

After simulation completes, analyze the CSV:

```bash
# Analyze with default parameters (plots joint 0 in detail)
python3 scripts/analyze_log.py

# Specify input CSV and output directory
python3 scripts/analyze_log.py --input data/logs/piper_arm.csv --output-dir data/results/offline_analysis

# Plot a specific joint (e.g., joint 3)
python3 scripts/analyze_log.py --joint 3
```

Or use the **Analysis: Run on latest log** task in VS Code (Ctrl+Shift+P).

**Output:**
- `summary.txt` — RMS tracking error, max error, disturbance estimate magnitude per joint
- `joint_N_analysis.png` — Time-domain plot (position, error, control torque, disturbance estimate)
- `joint_N_phase.png` — Phase plot (desired vs actual position)

### 3D end-effector trajectory visualization

After simulation, visualize the end-effector path in 3D space:

```bash
python3 scripts/analyze_3d_trajectory.py
```

**Output:**
- `ee_trajectory_3d.png` — 3D plot of actual (red) and desired (green) end-effector paths
- `ee_trajectory_error.png` — 3D trajectory colored by tracking error magnitude
- `ee_trajectory_projections.png` — XY, XZ, YZ 2D projections with overlaid paths

The script computes tracking statistics (max/avg position error) and displays them on screen.

---

## Output

Simulation data is written to `data/logs/piper_arm.csv` (path set in YAML, directory created automatically).

CSV columns per timestep:

| Group | Columns |
|---|---|
| Time | `t` |
| True joint position | `q0` – `q5` |
| Desired joint position | `q_des0` – `q_des5` |
| True joint velocity | `dq0` – `dq5` |
| Controller torque | `tau_ctrl0` – `tau_ctrl5` |
| Disturbance estimate | `d_hat0` – `d_hat5` |
| End-effector position (actual) | `ee_pos_x`, `ee_pos_y`, `ee_pos_z` |
| End-effector position (desired) | `ee_des_pos_x`, `ee_des_pos_y`, `ee_des_pos_z` |

---

---

## Configuration

`config/sim/piper_arm.yaml` — all parameters, no recompilation required.

### Switch active observer

```yaml
active_observer: dob        # "dob" | "momentum" | "force"

controller:
  use_observer: true         # false = pure PD (d̂ ignored)
  observer_gain: 0.8         # feedforward scale on d̂  (1.0 = full cancellation)
```

### Enable matched disturbance (input channel)

```yaml
matched_disturbance:
  enabled: true
  constant:     [0.5, 0, 0, 0, 0, 0]               # constant offset per joint [Nm]
  noise_std:    [0.05, 0.05, 0.05, 0.02, 0.02, 0.02]  # Gaussian noise std [Nm]
  chirp_amp:    0.3                                  # sinusoidal sweep amplitude [Nm]
  chirp_freq_hz: 0.5
```

### Enable mismatched disturbance (physics layer)

```yaml
mismatched_disturbance:
  enabled: true
  mass_scale:    1.3          # inflate all body masses by 30 %
  force_body:    link6        # body to receive external force
  ext_force:     [0, 0, -3]   # world-frame external force [N]
  gravity_scale: 0.9          # scale effective gravity
```

### Controller gains

```yaml
controller:
  Kp:      [80, 80, 60, 40, 20, 20]     # [Nm/rad]
  Kd:      [5.0, 5.0, 4.0, 3.0, 1.5, 1.5]  # [Nm·s/rad]
  tau_max: [18, 18, 18, 9, 7, 7]        # torque saturation [Nm]
```

### Trajectory

```yaml
trajectory:
  amplitude_rad: 0.15     # 0 = hold home (pure disturbance rejection test)
  frequency_hz:  0.2
```

Each joint moves with a staggered phase offset (`i × π/3`) so joints move independently.

### Friction (true plant, unknown to controller)

```yaml
friction:
  enabled: true
  viscous: [0.08, 0.08, 0.06, 0.04, 0.02, 0.02]  # Bv [Nm·s/rad]
  coulomb: [0.20, 0.20, 0.15, 0.10, 0.06, 0.06]  # Bc [Nm]
```

Friction is applied to the true plant but is **not** modeled by the controller.  
The observers are expected to estimate and reject it.

---

## Project Structure

```
prj_arm/
├── include/arm/
│   ├── types.hpp                   # VecN (VectorXd), MatN (MatrixXd), ArmState, ArmConfig
│   ├── disturbance.hpp             # DisturbanceManager, MatchedConfig, MismatchedConfig
│   ├── sensor.hpp                  # JointSensor — noise, bias, drift, delay buffer
│   ├── friction.hpp                # FrictionModel — viscous·dq + coulomb·sign(dq)
│   ├── observer/
│   │   ├── observer_base.hpp       # pure virtual interface: update / estimate / reset
│   │   ├── dob.hpp                 # Q-filter DOB
│   │   ├── momentum_observer.hpp   # Momentum Observer (De Luca & Mattone)
│   │   └── force_observer.hpp      # Force Observer (model-free LPF)
│   └── controller/
│       └── pd_controller.hpp       # PD + optional observer feedforward
│
├── src/
│   ├── main.cpp                    # simulation loop
│   ├── disturbance.cpp
│   ├── sensor.cpp
│   ├── friction.cpp
│   ├── observer/
│   │   ├── dob.cpp
│   │   ├── momentum_observer.cpp
│   │   └── force_observer.cpp
│   └── controller/
│       └── pd_controller.cpp
│
├── sim/
│   ├── mujoco_env.{hpp,cpp}        # ArmEnv: getState, applyTorques, getMassMatrix
│   └── visualizer.{hpp,cpp}        # GLFW 3D view + real-time joint-0 plot + text overlay
│
├── model/
│   ├── piper_torque.xml            # Piper MJCF — motor actuators, gravcomp=1
│   └── piper_torque_scene.xml      # scene (floor, lights, sky)
│
├── asset/agilex_piper/             # STL/OBJ mesh assets
├── config/sim/piper_arm.yaml       # all experiment parameters
├── data/logs/                      # CSV output (auto-created at runtime)
└── CMakeLists.txt
```

---

## Observer Details

### DOB — Disturbance Observer

Q-filter on torque residual using a diagonal nominal inertia model:

```
residual[k] = τ_meas[k] − M_n · q̈_est[k] − Bv_n · q̇[k]
d̂[k]       = α · d̂[k−1] + (1−α) · residual[k]
α           = exp(−2π · f_cutoff · dt)
```

Tuning: `dob.inertia_nominal`, `dob.damping_nominal`, `dob.cutoff_hz`

### Momentum Observer

Uses the full MuJoCo mass matrix `M(q)` (computed by `mj_fullM` each step):

```
p[k]     = M(q[k]) · q̇[k]          (measured momentum)
r[k]     = Ko ⊙ (p[k] − p̂[k])     (residual = disturbance estimate)
p̂[k+1]  = p̂[k] + dt · (τ_cmd[k] + r[k])
```

Tuning: `momentum_observer.Ko`

### Force Observer

Model-free — no nominal inertia required:

```
d̂[k] = α · d̂[k−1] + (1−α) · (τ_meas[k] − τ_cmd[k])
```

Tuning: `force_observer.cutoff_hz`

---

## Disturbance Taxonomy

| Type | Injection point | Typical sources | Config key |
|---|---|---|---|
| **Matched** | Input channel | actuator noise, torque offset, vibration | `matched_disturbance` |
| **Mismatched** | Physics layer | payload force, mass uncertainty, gravity change | `mismatched_disturbance` |
| **Sensor** | Measurement layer | encoder quantization, cable flex, drift | `sensor` |
| **Friction** | True plant (hidden) | viscous + Coulomb, unknown to controller | `friction` |

Matched disturbances lie in the same subspace as the control input — observers can cancel them directly.  
Mismatched disturbances enter through the dynamics — full cancellation requires knowledge of `M(q)`.
