# Adaptive-Cruise-Control-System

This repository documents the development of an **ADAS Adaptive Cruise Control (ACC)** function **aligned with automotive engineering practices**. The work is structured to support traceability, modularity, and progressive verification in a way that is consistent with the intent of **ISO 26262** (functional safety) development processes.

The solution is organized into **four complementary work packages**, covering the pipeline from perception outputs to controller verification and validation.

---

## Work Package 1 — Perception and Data Interpretation (Camera + Radar, simulation-based)

The **Perception Module** ingests **simulated sensor outputs** (camera images and radar-like measurements) rather than real hardware signals. An **AI-based processing stage** transforms the raw inputs into a stable and interpretable set of variables relevant to longitudinal control, for example:
- lead target distance,
- relative speed,
- lane association / same-lane classification,
- target validity indicators.

A key objective is **robust signal conditioning**: cleaning, smoothing, and validating outputs to ensure consistent behavior of the downstream ACC function. The design is intentionally modular so that the ACC controller does not depend on the internal details of the sensor processing implementation.

---

## Work Package 2 — ACC Control Development using Model-Based Design (MBD)

The **ACC Control Module** is developed in **MATLAB/Simulink** following **Model-Based Design (MBD)** principles. The architecture includes:

- **Stateflow supervisory logic** to manage operational modes, including:
  - standby,
  - cruise speed control,
  - following / gap control,
  - overrides and fallback behavior.

- A **PID-based longitudinal control strategy** producing acceleration demand, later mapped to:
  - **throttle command**
  - **brake command**

- A **vehicle longitudinal dynamics plant model** to simulate ego vehicle response in closed loop.

The primary control objective is to maintain a **safe time headway** to a detected lead vehicle while ensuring:
- comfort (smooth responses),
- stability,
- bounded actuator commands,
- safe fallback behavior when target information is invalid or inconsistent.

---

## Work Package 3 — Driving Scenario Simulation Environment

A scenario simulation layer is planned to evaluate the ACC function across representative use cases, such as:
- lead vehicle braking events,
- target cut-in / cut-out,
- speed changes and set-speed tracking,
- variations in initial spacing and relative speed.

The visualization and execution environment is under selection (e.g., **3D**, **bird’s-eye view**, or **2D**). The intent is to support:
- **repeatable scenario playback**
- structured coverage of key ACC behaviors
- measurable performance assessment across scenarios.

---

## Work Package 4 — Verification and Validation (MIL and SIL)

To align with automotive development and safety-oriented verification strategies, the project includes staged testing:

- **Model-in-the-Loop (MIL)** testing  
  Used to validate control logic, Stateflow mode behavior, and closed-loop performance at the Simulink model level.

- **Software-in-the-Loop (SIL)** testing (selected modules)  
  Used to execute generated code for early assessment of:
  - model-to-code equivalence,
  - numerical behavior changes,
  - implementation constraints and integration readiness.

---

## 📊 Project Progress

### 1. Model-Based Design (MBD) - Vehicle Dynamics
- [x] **Validated Dynamic Model:** Developed `Vehicle_Dynamics.slx`, a high-fidelity longitudinal model including aerodynamic drag, rolling resistance, and mass inertia.
- [x] **Open-Loop Validation:** Verified model response against theoretical curves (Acceleration/Braking) in `validate_dynamics_open_loop.m`.
- [x] **Initialization:** Centralized parameters in `init_params.m` using SI units.

#### Vehicle Dynamics Model Overview
The longitudinal dynamics model captures the physical response of the ego vehicle to throttle and brake commands.

![Simulink Vehicle Dynamics Model](images/Vehicle_Dynamics.jpg)

#### Performance Validation
Validation tests confirm the model's accuracy in representing acceleration and deceleration phases.

<p align="center">
  <img src="images/dynamic_open_loop_fig.jpg" width="45%" />
  <img src="images/Vehicle_Model_Test_Results.jpg" width="45%" />
</p>

### 2. Perception & AI Environment
- [x] **Environment Setup:** Configured Python environment with PyTorch and nuScenes-devkit.
- [x] **Dataset Preparation:** Downloaded and integrated **nuScenes-mini**.
- [x] **Data Extraction:** Created `extract_radar_data.py` to bridge nuScenes data with MATLAB.
- [x] **YOLO Perception:** Implemented YOLOv8 vehicle detection and distance estimation.
- [x] **Sensor Fusion:** Implemented Linear Kalman Filter (LKF) to merge Radar and Vision data.
- [x] **Data Import:** Created `import_fusion_data.m` to generate timeseries for Simulink.

#### Sensor Fusion & Data Import
The perception pipeline concludes with a fusion stage that stabilizes the target tracking.

<p align="center">
  <img src="images/kalman_fusion_result.jpg" width="45%" />
  <img src="images/data_imported.jpg" width="45%" />
</p>

### 3. ACC Control Development (WP2)
- [x] **Supervisory Logic:** Developed `ACC_Mode_Manager.slx` using Stateflow for mode management (Standby, Speed, Gap).
- [x] **Longitudinal Control:** Implemented `ACC_Controller.slx` with dual PID loops and safety saturation ($\pm 0.2g$).
- [x] **Vehicle Control Interface (VCI):** Implemented physics-based model inversion with a $15\text{ N}$ deadband.
- [x] **Closed-Loop Integration:** Built the global `ACC_System_Closed_Loop.slx` model and resolved algebraic loops and standstill division-by-zero anomalies.
- [x] **MIL Performance Audit:** Simulated and audited the closed-loop system over a $30\text{ s}$ nuScenes sensor fusion replay.

#### Mode Manager & PID Controller
The controller architecture separates high-level decision making from low-level regulation.

<p align="center">
  <img src="images/ACC_Mode_Manager.png" width="80%" />
</p>
<p align="center">
  <img src="images/ACC_Controller_overview.png" width="80%" />
</p>

### 4. Closed-Loop Integration (MIL) & Vehicle Control Interface (VCI)
To bridge the high-level ACC controller with the physical vehicle model (`Vehicle_Dynamics.slx`), a low-level **Vehicle Control Interface (VCI)** block was designed and integrated into the global closed-loop model **`ACC_System_Closed_Loop.slx`**.

The VCI applies dynamic model inversion (Feedback Linearization) to translate the acceleration demand ($a_{cmd}$) into exclusive throttle ($u_{th}$) and brake ($u_{br}$) actuator commands, compensating in real-time for resistive forces (aerodynamic drag, rolling resistance, and gravity road slope):

$$F_{req} = m_{eff} \cdot a_{cmd} + F_{aero} + F_{roll} + F_{slope}$$

#### Closed-Loop Simulink Model Overview
Below is the full layout of the integrated Model-in-the-Loop (MIL) simulation platform:

<p align="center">
  <img src="images/ACC_System_Closed_Loop.png" width="90%" />
  <br><em>Simulink Closed-Loop Integration Platform (ACC_System_Closed_Loop.slx)</em>
</p>

#### Vehicle Control Interface Subsystem
Below is the internal structure of the VCI block performing feedback linearization:

<p align="center">
  <img src="images/Vehicle_Control_Interface.png" width="90%" />
  <br><em>Internal layout of the VCI block (Inverse Dynamics)</em>
</p>

#### Functional Safety & Physical Stabilization (ISO 26262):
- **Strict Actuator Interlocking ($u_{th} \cdot u_{br} == 0$):** A $15\text{ N}$ force deadband prevents overlapping throttle and brake pedal inputs, protecting the physical actuators from overheating.
- **Algebraic Loop Resolution:** A $10\text{ ms}$ `Unit Delay` block was introduced in the speed feedback loop ($V_{ego}$), representing realistic CAN bus propagation delay and stabilizing the simulation solver.
- **Standstill Division-by-Zero Protection:** Saturated the headway time gap denominator at $V_{ego} \ge 0.5\text{ m/s}$ to prevent infinite gap divergence and mode manager logic failures at a complete stop.

#### Unit VCI Validation Results
The VCI allocation logic was validated under a harmonic acceleration command profile using `validate_vci_logic.m`:

<p align="center">
  <img src="images/VCI_validation_test.jpg" width="80%" />
  <br><em>Standalone VCI validation curves (Requested forces vs Actuator commands)</em>
</p>

#### Global Closed-Loop MIL Simulation Results
The closed-loop system was simulated over a $30\text{ s}$ replay of actual nuScenes data (`scene-0061`) using a fixed-step **`ode3` solver at 10 ms**:

<p align="center">
  <img src="images/ACC_validation_test.jpg" width="90%" />
  <br><em>Closed-Loop MIL simulation results (Vset vs Vego, Clearance vs Dsafe, VCI pedals and Active Mode)</em>
</p>

- **Distance Regulation:** Highly stable tracking of the dynamic safety clearance ($D_{safe}$), with a **mean spacing error of only $11.07\text{ m}$** during the active Gap Control phase.
- **Safety Interlocking:** Zero pedal overlap detected in steady-state.
- **Stateflow Logic Compliance:** At $t = 22\text{ s}$, as the target vehicle slows down below the minimum ACC activation threshold of $40\text{ km/h}$, the supervisory logic automatically deactivates and safely transitions from `GAP` (3) to `STANDBY` (1), in accordance with safety requirement **`REQ-DEACT-02`**.

### 5. Cascaded Loop Tuning & Transient Jerk Mitigation (WP2 - Week 10)
To optimize tracking performance while ensuring high passenger comfort, the cascaded control loop was calibrated and audited using `sweep_pid_gains.m` and `tune_and_validate_acc.m`:

* **Inner Speed Loop (PI):** Tuned first by disabling the outer gap loop. Gains $K_p = 1.0$, $K_i = 0.05$ achieve a swift rise time ($T_r \approx 1.5\text{ s}$) and $0\%$ overshoot to torque disturbances.
* **Outer Gap Loop (PID):** Regulates the spacing error $e_d = d_{rel} - d_{safe}$ using a PID controller with derivative filtering ($N = 50$). Gains $K_p = 0.6$, $K_i = 0.005$, $K_d = 0.1$ ensure a phase margin $M_\phi \ge 60^\circ$ for robust disturbance rejection.
* **Bumpless Transfer (Integrator Reset):** Designed clamping (anti-windup) and dynamic integrator reset logic. During switching from Speed to Gap mode, the outer PID integrator is initialized to the current acceleration state to prevent sudden actuator command jumps.

<p align="center">
  <img src="images/Speed_Loop_Step_Response.jpg" width="32%" />
  <img src="images/Closed_Loop_Spacing_Error.jpg" width="32%" />
  <img src="images/Bumpless_Jerk_Comparison.jpg" width="32%" />
</p>

*   **Jerk Reduction:** Implementing the integrator reset reduced the transient jerk peak from **$28.4\text{ m/s}^3$** down to **$1.8\text{ m/s}^3$**, representing a **$93.6\%$ improvement** in ride comfort.

---

### 6. Virtual 3D Environment & Online Closed-Loop Validation (WP3 - Week 11)
Transitioned the simulation platform from offline CSV data replay to an online closed-loop simulation using MATLAB's **Driving Scenario Designer** and the **Scenario Reader** block.

* **Closed-Loop Ego Feedback:** Routed the physical vehicle's position ($x_{ego}$) and velocity ($v_{ego}$) back to the Scenario Reader's ego port.
* **Discrete Sample-Time Synchronization:** Introduced a 10 ms `Delay_X_ego` unit delay on position and configured all input Constants feeding the `Bus Creator` block with a discrete `0.01s` sample time. This eliminated sample-time mismatch compilation failures.
* **Header and Extraction:** Configured a `Bus Selector` to extract the relative distance ($d_{rel}$) and relative speed ($v_{rel}$) in vehicle coordinates directly from the reader's `Actors` bus.
* **Verification over Extended Scenarios:** Programmed a suite of three validation scenarios over a **1000m road segment** and **25s duration**:
  * **Scenario A (Constant Speed):** Target vehicle traveling at constant 60 km/h.
  * **Scenario B (Emergency Braking):** Target vehicle cruises at 80 km/h and suddenly brakes at $-6\text{ m/s}^2$ to a complete stop.
  * **Scenario C (Variable Speed):** Target vehicle tracks step-like velocity profiles.

<p align="center">
  <img src="images/Validation_Report_Scenario_A_Constant_Speed.png" width="32%" />
  <img src="images/Validation_Report_Scenario_B_Emergency_Braking.png" width="32%" />
  <img src="images/Validation_Report_Scenario_C_Variable_Speed.png" width="32%" />
</p>

* **Validation Safety Audit:** Safe spacing headway was maintained with a minimum clearance of **54.9 m** across all scenarios. Passenger comfort was preserved with deceleration values strictly bounded within the physical limit of **$\pm 0.2g$**.

---

### 7. Next Steps
- [ ] Implement programmatic test suite of 9 operational scenarios (Week 12).
- [ ] Replace offline sensor feeds with an online Extended Kalman Filter (EKF) block (Week 13).



