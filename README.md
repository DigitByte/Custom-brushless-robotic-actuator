# Custom Brushless Robotic Actuator Library

Core repository for gearbox and actuator component development. This project is the primary source of mechanical assets, electronics placeholders, and ROS2 control integration for a modular brushless actuator platform.

## Purpose

This repo is organized as a reusable **component library** for:
- Planetary gearbox parts and integrated actuator CAD.
- Actuator subsystem bring-up and ROS2 control integration.
- Future electronics hardware and firmware associated with this actuator family.

## Repository Structure

```text
custom-brushless-robotic-actuator/
├── docs/
│   ├── overview.md
│   └── repository-map.md
├── mechanical/
│   ├── README.md
│   └── cad/
│       ├── actuator/
│       └── planetary-gearbox/
├── electronics/
│   ├── README.md
│   ├── hardware/
│   └── firmware/
└── software/
    ├── README.md
    └── ros2/
        └── odrive-stack/
            └── ws_odrive/
```

## Current Component Coverage

### Mechanical
- `mechanical/cad/planetary-gearbox/`: STL files for planetary transmission components.
- `mechanical/cad/actuator/`: actuator assembly files in STEP/STL.

### Software
- `software/ros2/odrive-stack/ws_odrive/`: ROS2 workspace for ODrive-oriented control stack.

### Electronics
- `electronics/hardware/`: reserved for schematics, PCB, BOM, and wiring references.
- `electronics/firmware/`: reserved for embedded actuator control firmware.

## Typical Workflow

1. Select gearbox/actuator CAD from `mechanical/cad/`.
2. Manufacture or print components and build prototype hardware.
3. Integrate control stack from `software/ros2/odrive-stack/ws_odrive/`.
4. Add board/firmware artifacts under `electronics/` as designs mature.

## Related Repositories

This repo is the central actuator library. Supporting repositories in this workspace provide encoder bring-up, PID tuning, and application-level demonstrations (arm, gimbal, IMU).

## Status

Active development. Structure is stable and intended for long-term expansion as a reusable actuator component library.
