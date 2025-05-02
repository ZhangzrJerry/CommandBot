# 3, 2, 1, Let's Dive ~

### A Command-Based Robot Framework for 2025 FRC REEFSCAPE

<img width=100% src="./assets/screenshot.png">

![](./assets/reefscape.svg) ![](./assets/java.svg) ![](./assets/akit.svg) ![](./assets/ascope.svg)

Command Bot represents a paradigm shift in FRC robotics development, introducing a revolutionary hardware abstraction layer and modern control architecture. By redefining the interaction between software and hardware components, this framework eliminates legacy boilerplate while introducing powerful new capabilities.

#### 🧠 **Intelligent Hardware Abstraction**

- **Unified Hardware Interface**: Consolidates 20+ legacy interfaces (`GenericElevatorIO`, `GenericArmIO`, etc.) into a single **`DCMotorIO`** interface
- **Automated Unit Conversion**: Seamlessly handles conversions between radians (simulation), rotations (Phoenix), degrees, and meters (subsystem)
- **Universal Simulation Support**: Single simulation interface compatible with all mechanism types (arms, elevators, flywheels, etc.)
- **Advanced Hardware Integration**: Native support for TalonFX follower mode and CANCoder fusion mode

#### ⚙️ **Modern Architecture**

- **True Hardware Independence**: Identical code paths for both real hardware and simulation environments
- **Factory-Patterned Subsystems**: Effortless hardware swapping without logic modifications
- **Visual Debugging Suite**: Real-time mechanism tracing and analysis tools
- **Command-Based Control Flow**: Robust and maintainable control architecture

_Note: Command Bot is currently in **beta** as we refine the architecture and expand hardware support. While not yet optimized for competition use, we encourage teams to explore the codebase and adapt these patterns to their robots._

## Environment Setup

1. **Prerequisites**

   - Clone this repository
   - Install FRC Game Tools

2. **WPILib Configuration**

   - Download [WPILib v2025.3.2](https://github.com/wpilibsuite/allwpilib/releases/tag/v2025.3.2)
   - Initialize project: `.\gradlew`
   - View available tasks: `.\gradlew tasks`

3. **Advantage Scope Setup**

   - Install [Advantage Scope v4.1.6](https://github.com/Mechanical-Advantage/AdvantageScope/releases/tag/v4.1.6)
   - Import layout: `File -> Import Layout` (select `.\.ascope\layout.json`)
   - Configure assets: Copy `.\.ascope\Robot_Sideway` to assets folder
   - Network configuration:
     - Set `Live Source` to `NetworkTables 4`
     - Configure `roboRIO Address` (default: `10.te.am.2`)
     - Set log directory: `/home/lvuser/logs`

4. **Elastic Dashboard** (Optional)
   - Install [Elastic v2025.2.2](https://github.com/Gold872/elastic-dashboard/releases/tag/v2025.2.2)
   - Load layout: `File -> Open Layout` (select `.\src\main\deploy\elastic-layout.json`)

## Quick Start Guide

1. Launch Simulation:

   - Open WPILib VS Code
   - Run `WPILib: Simulate Robot Code`

2. Configure Controls:

   - Open FRC Driver Station
   - Connect and enable joystick

3. Monitor Robot State:
   - Launch Advantage Scope
   - Connect to simulator: `File -> Connect to Simulator`

## Controller Button Mapping

| Button     | Primary Function                           | Secondary Function                             |
| ---------- | ------------------------------------------ | ---------------------------------------------- |
| `A`        | **Idle Arm** (short press)                 | **Home Arm** (long press)                      |
| `B`        | **Algae Processor Score**                  | -                                              |
| `X`        | **Algae Ground Pick** (hold)               | -                                              |
| `X + RT`   | **Algae Ground Eject** (hold)              | -                                              |
| `Y`        | **Algae Reef Collect** (no algae)          | **Algae Net Score** (with algae)               |
| `LB`       | **Coral Left Station Collect** (no coral)  | **Coral Score with NodeSelector** (with coral) |
| `RB`       | **Coral Right Station Collect** (no coral) | **Coral Score with POV** (with coral)          |
| `LB + RB`  | **Toggle Arm Forced Elevate Mode**         | -                                              |
| `LT`       | **Score Coral** (score position)           | **Eject Coral** (other positions)              |
| `RT`       | **Score Algae** (score position)           | **Eject Algae** (other positions)              |
| `Back`     | **Enter Climbing Mode**                    | -                                              |
| `Start`    | **Home Gyro**                              | -                                              |
| `POV Down` | **Climbing** (climbing mode only)          | -                                              |
