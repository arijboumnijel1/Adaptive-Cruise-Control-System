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

