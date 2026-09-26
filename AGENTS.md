# Agent context: autonomous-systems-vehicle

This file gives AI coding agents enough project context to work in this repository without treating sibling repos or the `core` submodule as code owned here.

## Project identity

- **Team:** [Técnico Fuel Cell](https://github.com/TecnicoFuelCell) (TFC) — **student team** at Instituto Superior Técnico; the autonomy stack is developed by the **Autonomous Systems** group within the team.
- **Vehicle:** Battery-electric prototype autonomous car (Ackermann-style steering); monocular camera, no lidar.
- **This repository:** Real-car **vehicle setup** — hardware I/O on the car (CAN firmware + Jetson ROS 2 bridge). It is **not** the autonomy algorithms repo and **not** the simulator stack.
- **Simulation (elsewhere):** The team uses **Gazebo** for simulation. Code that talks to the simulator (worlds, plugins, virtual sensors/actuators, sim bringup) lives in **autonomous-systems-simulation**, not here.

### Actuators (two)

The car has two actuators this repo drives and reads back over CAN (via **PCToCAN** / **CANToPC** and the ROS bridge):

1. **Propulsion (drive motor)** — Controlled by a **VESC** (motor controller) on the CAN bus. A dedicated ESP32 node (`embedded/CAN/PedalToVESC/`) can also interact with the VESC (pedal input, local debug). The Jetson sends throttle/brake as UART lines to **PCToCAN** (`R2:` motor current, `L2:` brake current), which translates them into VESC CAN commands (`vesc_set_current`, `vesc_set_brake_current`). Telemetry (temperature, currents, duty cycle, RPM, input voltage, watt-hours) is published on the bus, aggregated by **CANToPC** as `VESC:` CSV lines, and exposed on ROS as **`/vesc_data`** (`car_msgs/VescData`) by `vesc_pub`. Wheel odometry in the core stack uses RPM from the VESC together with steering feedback.

2. **Steering (direction motor)** — A **stepper motor** on the front axle, driven by an ESP32-C3 in `embedded/CAN/Direction/` (step/dir outputs on `D5`/`D6`). **PCToCAN** accepts `Dir: <angle>` from the Jetson (from teleop or autonomy via `uart_writer` and `car_msgs/ImSpeed`), clamps the command, and sends the target on CAN; the Direction node steps the motor to match and periodically reports the current steering angle on CAN. **CANToPC** forwards feedback as `Dir:` UART lines; **`dir_pub`** publishes **`/dir_data`** (`car_msgs/Dir`, field `dir` as a signed steering value). Command and feedback use the CAN message layouts in `embedded/src/canIds.h` (steering angle is scaled on the bus; UART feedback is reported in the same units the ROS nodes expect).

Teleop today maps joystick axes to `Dir:` plus `R2:`/`L2:` lines; autonomy uses the same serial path for steering and drive commands.

## Sibling repositories

| Repository | Role |
|------------|------|
| [autonomous-systems-core](https://github.com/TecnicoFuelCell/autonomous-systems-core) | Autonomous core logic (SLAM, perception, MPC, etc.) |
| [autonomous-systems-io](https://github.com/TecnicoFuelCell/autonomous-systems-io) | Shared ROS 2 I/O contract (topics, messages) used by vehicle and simulation |
| [autonomous-systems-vehicle](https://github.com/TecnicoFuelCell/autonomous-systems-vehicle) | **This repo** — implements the I/O contract on physical sensors and actuators |
| [autonomous-systems-simulation](https://github.com/TecnicoFuelCell/autonomous-systems-simulation) | **Gazebo** simulation — ROS 2 code that interacts with the simulator and implements the same I/O contract on virtual sensors and actuators |

Do not implement core logic, simulation drivers, or the shared message package in this repo unless the user explicitly asks for a vehicle-side integration only. If the work belongs elsewhere, **tell the user to open and change the intended repository** instead of patching it here.

## `core/` submodule in this workspace

The `core/` directory is a **git submodule** pointing at autonomous-systems-core. It is included so this workspace can build against shared types (e.g. `car_msgs`) and align with the stack. Treat it as **read-only context** for vehicle work: do not refactor SLAM, YOLO, MPC, or bringup inside `core/` when the task is vehicle setup. Advise the user to clone or work in **autonomous-systems-core** and commit there.

## Hardware and data path (high level)

- Multiple **ESP32-C3** boards with **MCP2515** on a **CAN bus** (sensors and actuators: pedal/VESC, steering, IMU, magnetometer, deadman, etc.).
- The onboard computer is an **NVIDIA Jetson Orin (64 GB)**. It is **not** a CAN node. Two dedicated boards bridge UART and CAN:
  - **CANToPC** — read-only: CAN → UART text lines to the Jetson.
  - **PCToCAN** — write-only: UART commands from the Jetson → CAN (traction and steering).
- **Camera** and **GPS** connect to the Jetson over USB/serial; they do not use the CAN bridge for payload data.
- **Safety:** A deadman path on CAN can disarm command output when the watchdog signal goes stale.

## Onboard runtime (ROS 2)

- **Host OS:** Ubuntu **24.04 LTS** on the Jetson Orin.
- **ROS distribution:** **ROS 2 Jazzy** (target the Jazzy APIs and packages when changing `ros2-vehicle/` or launch files).
- **Execution model:** ROS 2 code from this repo is meant to run **on the car’s Jetson**, inside a **Docker container** that bundles ROS 2 and the project dependencies/libraries. Treat device paths (e.g. `/dev/ttyACM*`, `/dev/video0`) as passed through from the host into the container unless the user says otherwise.

Embedded firmware (`embedded/`) is flashed to ESP32 boards and does **not** run in this container.

## Software path (high level)

Inside the onboard container, ROS 2 nodes in this repo:

1. Read UART from CANToPC and fan out raw lines (e.g. `/vehicle_internal/serial/*`).
2. Decode into typed sensor topics for the data I/O interface / core stack.
3. Subscribe to motion commands (e.g. teleop today; autonomy later) and write UART to PCToCAN.

Autonomy logic talks **ROS 2**, not CAN directly.

## Repository layout (roles; folders may be reorganized)

| Area | Typical path | Purpose |
|------|----------------|---------|
| Embedded firmware | `embedded/` | Arduino sketches for CAN nodes and shared CAN helpers |
| Onboard ROS bridge | `ros2-vehicle/` | UART, sensor publishers, command writer, camera, GPS, joystick, deadman |
| Bringup | `launch/` | Launch files for vehicle-side nodes (may move under ROS tree) |
| Calibration | `calibration/` | Camera, magnetometer, odometry-related calibration (may move) |
| Parameters | `config/` | Shared YAML params for vehicle nodes |

Prefer describing **roles** over assuming permanent folder names.

## Agent rules

### Audience (student maintainers)

Contributors are **engineering students** on a mixed team: some with a **computer science** background, some from **electrical / electrotechnical** engineering, and others with **little or no formal informatics training**. When you explain changes in chat, define acronyms and stack concepts briefly (ROS 2, CAN, UART, nodes/topics) instead of assuming everyone has the same baseline. In code and comments, favor clarity over cleverness so both software- and hardware-oriented teammates can follow the change. Do not talk down; keep explanations proportional to the task.

### Language

- Use **English** in chat with the user.
- Write **all code in English**: identifiers, string literals meant for logs/docs, commit messages you suggest, and **comments**.
- Do not add Portuguese (or other languages) in new code or comments unless the user explicitly requests it (legacy firmware may contain Portuguese; do not spread it in new changes).

### Code style and comments

- **No emojis** in source code, comments, or log messages.
- Comments should be **concise**, **technical**, and **easy for a student** to follow (see **Audience** above): explain non-obvious intent, hardware quirks, or safety behavior — not restate what the code already says.
- Match existing conventions in the file and package (naming, ROS 2 patterns, Arduino style). Keep diffs minimal unless the task requires broader change.

### Scope and accuracy

- Do not invent CAN IDs, UART line formats, or ROS topic names; verify in `embedded/`, `uart_reader`, and sibling `autonomous-systems-io` / `car_msgs` as needed.
- **Do not change CAN IDs** (`embedded/src/canIds.h` or per-sketch definitions) as a shortcut to fix unrelated bugs (timing, parsing, wiring, ROS topics, deadman logic, etc.). IDs are shared across every node on the bus; renumbering breaks firmware, bridges, and any recorded traces. Fix the root cause instead, and only change IDs when the user explicitly requests a coordinated bus-wide update (all affected sketches and decoders updated together).
- Do not add runbooks (install, launch commands, device paths) to this file; the README and launch files are the human entry points unless the user asks for documentation.
- Do not implement SLAM, neural detectors, MPC, or Gazebo simulation in this repo.
- Do not commit secrets (keys, tokens, `.env` with credentials).

### Cross-repository changes

If the requested change clearly belongs in another repo, **do not implement it in this workspace** (including edits under `core/`). Instead:

1. **Name the correct repository** (core, io, simulation, or another sibling).
2. **Explain why** it belongs there (e.g. shared message definitions → **autonomous-systems-io**; SLAM/MPC → **autonomous-systems-core**; Gazebo worlds, plugins, or sim I/O → **autonomous-systems-simulation**).
3. **Advise the user** to make the change in that repository’s clone and follow that repo’s workflow. Only add or change code **here** when it is genuinely vehicle I/O (firmware, UART bridge, Jetson nodes, vehicle bringup/calibration).

Exception: a small vehicle-only adapter (e.g. subscribing to an existing io topic) is fine in this repo when the user explicitly wants vehicle integration.

### When unsure

- Ask the user before changing the CAN/UART protocol, **CAN ID assignments**, deadman behavior, or anything that affects on-car safety.
- If ownership of a change is unclear, ask which repo should own it before editing.

## Human-oriented docs

- **README.md** — short introduction, stack diagram, and structure for visitors.
- **Firmware** — `embedded/src/canIds.h` and sketches are the source of truth for on-bus message layout; agents should read them rather than duplicating full tables in docs.
