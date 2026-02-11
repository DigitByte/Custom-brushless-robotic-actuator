# Robotic Gearboxes & Actuators

**Click on the links below for detailed information**

## Gearbox Designs

| Planetary Gearbox | Cycloidal Drive | Harmonic Drive | Custom Reduction |
|-------------------|-----------------|----------------|------------------|
| [![Planetary](docs/images/placeholder_planetary.jpg)](#planetary-gearbox) | [![Cycloidal](docs/images/placeholder_cycloidal.jpg)](#cycloidal-drive) | [![Harmonic](docs/images/placeholder_harmonic.jpg)](#harmonic-drive) | [![Custom](docs/images/placeholder_custom.jpg)](#custom-reduction) |

## Actuator Modules

| Integrated Actuator | Modular Design | High-Torque Unit | Compact Unit |
|---------------------|----------------|------------------|--------------|
| [![Integrated](docs/images/placeholder_integrated.jpg)](#integrated-actuator) | [![Modular](docs/images/placeholder_modular.jpg)](#modular-design) | [![High-Torque](docs/images/placeholder_high_torque.jpg)](#high-torque-unit) | [![Compact](docs/images/placeholder_compact.jpg)](#compact-unit) |

## CAD Files & Manufacturing

| STEP Files | STL Files | Manufacturing Guide | Assembly Drawings |
|------------|-----------|---------------------|-------------------|
| [![STEP](docs/images/placeholder_step_files.jpg)](#step-files) | [![STL](docs/images/placeholder_stl_files.jpg)](#stl-files) | [![Manufacturing](docs/images/placeholder_manufacturing.jpg)](#manufacturing-guide) | [![Assembly](docs/images/placeholder_assembly_dwg.jpg)](#assembly-drawings) |

| 3D Printed Parts | Machined Components | Hardware BOM | Tolerance Specifications |
|------------------|---------------------|--------------|-------------------------|
| [![3D Prints](docs/images/placeholder_3d_prints.jpg)](#3d-printed-parts) | [![Machined](docs/images/placeholder_machined_parts.jpg)](#machined-components) | [![BOM](docs/images/placeholder_bom.jpg)](#hardware-bom) | [![Tolerances](docs/images/placeholder_tolerances.jpg)](#tolerance-specifications) |

## Control & Software

| Sample Code | Control Algorithms | Performance Testing | Characterization |
|-------------|-------------------|---------------------|------------------|
| [![Sample Code](docs/images/placeholder_sample_code.jpg)](#sample-code) | [![Algorithms](docs/images/placeholder_algorithms.jpg)](#control-algorithms) | [![Testing](docs/images/placeholder_perf_testing.jpg)](#performance-testing) | [![Characterization](docs/images/placeholder_characterization.jpg)](#characterization) |

## Assembly & Testing

| Gearbox Assembly | Motor Integration | Bearing Installation | Load Testing |
|------------------|-------------------|---------------------|--------------|
| [![Gearbox Assembly](docs/images/placeholder_gearbox_assembly.jpg)](#gearbox-assembly) | [![Motor](docs/images/placeholder_motor_integration.jpg)](#motor-integration) | [![Bearings](docs/images/placeholder_bearings.jpg)](#bearing-installation) | [![Load Test](docs/images/placeholder_load_test.jpg)](#load-testing) |

## Analysis & Documentation

| Gear Ratio Analysis | Efficiency Curves | Backlash Measurement | Design Calculations |
|---------------------|-------------------|---------------------|---------------------|
| [![Gear Ratio](docs/images/placeholder_gear_ratio.jpg)](#gear-ratio-analysis) | [![Efficiency](docs/images/placeholder_efficiency.jpg)](#efficiency-curves) | [![Backlash](docs/images/placeholder_backlash.jpg)](#backlash-measurement) | [![Calculations](docs/images/placeholder_calculations.jpg)](#design-calculations) |

---

## Project Overview

This repository contains comprehensive designs, manufacturing files, and control software for custom robotic gearboxes and actuator modules. The project emphasizes:

- **Open Source Design**: Complete CAD files in STEP and STL formats
- **Manufacturing Ready**: Detailed specifications and tolerances
- **Tested Performance**: Characterized designs with measured data
- **Modular Architecture**: Scalable and adaptable configurations
- **Control Integration**: Sample code and control libraries

### Design Philosophy

**Modularity**: Interchangeable gearbox and motor combinations
**Performance**: Optimized for robotics applications
**Manufacturability**: Design for both 3D printing and machining
**Reliability**: Proven designs with testing data
**Open Development**: Complete documentation and source files

---

## System Specifications

### Gearbox Performance

| Parameter | Planetary | Cycloidal | Harmonic |
|-----------|-----------|-----------|----------|
| **Reduction Ratio** | 3:1 - 10:1 | 10:1 - 100:1 | 50:1 - 160:1 |
| **Efficiency** | 90-95% | 85-92% | 80-90% |
| **Backlash** | 0.5-2° | <0.1° | <0.05° |
| **Torque Density** | Medium | High | Very High |

### Actuator Configurations

| Configuration | Reduction | Peak Torque | Cont. Torque | Weight |
|--------------|-----------|-------------|--------------|---------|
| **Compact** | 6:1 | 2 Nm | 0.8 Nm | 150g |
| **Standard** | 9:1 | 5 Nm | 2 Nm | 280g |
| **High-Torque** | 15:1 | 12 Nm | 5 Nm | 450g |

---

## Getting Started

### Prerequisites

**For Manufacturing**:
- 3D printer (FDM or SLA for prototypes)
- CNC mill/lathe (for production parts)
- Standard metric hardware (M2-M6)
- Bearings (size-specific, see BOM)

**For Control**:
- Motor controller (compatible with your motor)
- Microcontroller or SBC (Arduino, Raspberry Pi, etc.)
- Power supply (voltage/current dependent on motor)

### Quick Start

1. **Download CAD Files**
   - STEP files: `cad/step/`
   - STL files: `cad/stl/`

2. **Review Manufacturing Guide**
   - See `docs/manufacturing/` for specifications
   - Check tolerance requirements
   - Review assembly sequence

3. **Manufacture or 3D Print Components**
   - Print parts from STL files OR
   - Machine from STEP files

4. **Assemble Gearbox/Actuator**
   - Follow assembly guide in `docs/assembly/`
   - Use specified torque values
   - Check for smooth operation

5. **Test & Control**
   - Use sample code from `examples/`
   - Characterize your build
   - Tune control parameters

### Repository Structure

```
robotic-gearboxes-actuators/
├── cad/                        # CAD files
│   ├── step/                   # STEP format (editable)
│   ├── stl/                    # STL format (3D printing)
│   └── source/                 # Native CAD files (Fusion360, etc.)
├── docs/                       # Documentation
│   ├── design/                 # Design methodology
│   ├── manufacturing/          # Manufacturing specs
│   ├── assembly/               # Assembly instructions
│   └── analysis/               # Performance analysis
├── examples/                   # Sample control code
│   ├── basic/                  # Simple examples
│   ├── characterization/       # Testing scripts
│   └── advanced/               # Complex implementations
├── hardware/                   # Bills of materials
│   ├── bom/                    # Component lists
│   └── sourcing/               # Supplier information
└── simulations/                # FEA and motion simulation
    ├── stress_analysis/
    └── efficiency_models/
```

---

## CAD Files

### Available Formats

**STEP Files** (`cad/step/`):
- Industry-standard format
- Fully editable in any CAD software
- Includes assembly structure
- Parametric where possible

**STL Files** (`cad/stl/`):
- Ready for 3D printing
- Organized by component
- Print-oriented orientation
- Multiple resolution options

**Source Files** (`cad/source/`):
- Native CAD format (Fusion 360, SolidWorks, etc.)
- Fully parametric
- Design history preserved
- Includes sketches and constraints

### File Organization

```
cad/
├── step/
│   ├── planetary_gearbox_6to1.step
│   ├── cycloidal_drive_20to1.step
│   ├── housing_assembly.step
│   └── ...
├── stl/
│   ├── planetary/
│   │   ├── sun_gear.stl
│   │   ├── planet_carrier.stl
│   │   └── ring_gear.stl
│   └── cycloidal/
│       ├── cycloidal_disk.stl
│       └── housing.stl
└── source/
    └── (native CAD files)
```

---

## Sample Code

### Control Examples

All examples are in `examples/` directory:

**Basic Control**:
- Position control
- Velocity control
- Torque control

**Characterization**:
- Efficiency measurement
- Backlash testing
- Thermal characterization
- Load testing

**Advanced**:
- Impedance control
- Force feedback
- Multi-actuator coordination

### Example: Basic Position Control

```python
# See examples/basic/position_control.py for full implementation
import actuator_controller as ac

# Initialize actuator
actuator = ac.Actuator(port='/dev/ttyUSB0', reduction=9.0)

# Command position
actuator.set_position(90.0)  # degrees

# Read actual position
pos = actuator.get_position()
print(f"Current position: {pos}°")
```

---

## Design Documentation

### [Design Principles](docs/design/)

- Gear tooth profiles and geometry
- Load distribution analysis
- Material selection criteria
- Manufacturing considerations

### [Manufacturing Guide](docs/manufacturing/)

- Machining specifications
- 3D printing settings
- Surface finish requirements
- Assembly tolerances

### [Performance Analysis](docs/analysis/)

- Efficiency calculations
- Thermal modeling
- Stress analysis (FEA)
- Backlash characterization

### [Assembly Instructions](docs/assembly/)

- Step-by-step assembly
- Torque specifications
- Alignment procedures
- Testing protocols

---

## Performance Data

### Measured Characteristics

Performance data from tested builds available in `docs/analysis/performance_data/`:

- Efficiency vs. load curves
- Thermal characteristics
- Backlash measurements
- Torque ripple analysis
- Speed-torque curves

### Testing Methodology

All testing procedures documented in `docs/testing/`:

- Test fixture designs
- Measurement protocols
- Data acquisition setup
- Analysis methods

---

## Manufacturing Options

### 3D Printing

**Recommended for**:
- Prototyping
- Low-load applications
- Proof of concept
- Custom modifications

**Materials**:
- PLA: Testing and prototypes
- PETG: Higher strength applications
- Nylon: Wear resistance
- Carbon fiber filled: Stiffness

### CNC Machining

**Recommended for**:
- Production units
- High-load applications
- Precision requirements
- Long-term durability

**Materials**:
- Aluminum 6061-T6: Lightweight, good strength
- Steel (4140): High strength applications
- Brass: Wear-resistant gears
- Delrin: Low-friction components

### Hybrid Approach

Many designs combine:
- 3D printed housings
- Machined gears and shafts
- Off-the-shelf bearings
- Standard hardware

---

## Bill of Materials

Complete BOMs available in `hardware/bom/`:

- Mechanical components
- Bearings and hardware
- Motors and sensors
- Electronic components

Supplier information in `hardware/sourcing/`:

- Recommended vendors
- Part numbers
- Pricing estimates
- Lead times

---

## More Information

[Project Documentation](docs/)
[CAD File Guide](docs/cad_guide.md)
[Manufacturing Guide](docs/manufacturing/)
[Assembly Instructions](docs/assembly/)
[Testing & Characterization](docs/testing/)
[Performance Database](docs/analysis/performance_data/)

---

## Contributing

We welcome contributions! Areas of interest:

- 🔧 **New Gearbox Designs**: Alternative configurations and ratios
- 📐 **Design Improvements**: Optimization and refinements
- 📊 **Performance Data**: Test results from your builds
- 💻 **Control Code**: New algorithms and implementations
- 📝 **Documentation**: Improvements and expansions
- 🎥 **Build Logs**: Photos and videos of your builds

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## Authors

Damien Delgado

## License

BSD 3-Clause License - See [LICENSE](LICENSE) for details

This applies to both:
- Mechanical designs (CAD files)
- Software (control code)

## Copyright

Copyright (c) 2026, [Your Organization/Name]

---

## Acknowledgments

- Open source robotics community
- [Relevant research papers/projects]
- Contributors and testers

---

## Safety & Legal

### Safety Notice

⚠️ **Important**: Gearboxes and actuators can generate significant force:

- Test in controlled environment
- Use proper guards and shielding
- Implement emergency stops
- Follow assembly torque specifications
- Inspect for wear regularly
- Never exceed rated loads

### Patent Notice

These designs are provided for educational and research purposes. Users are responsible for ensuring their use doesn't infringe on existing patents in their jurisdiction.

---

## Status & Roadmap

**Current Status**: 🟢 Active Development
**Version**: 0.2.0
**Last Updated**: January 2026

### Completed

- ✅ Basic planetary gearbox design
- ✅ STEP and STL files
- ✅ Sample control code
- ✅ Initial assembly documentation

### In Progress

- 🟡 Cycloidal drive design refinement
- 🟡 Comprehensive testing data
- 🟡 Video assembly guides

### Planned

- ⬜ Harmonic drive design
- ⬜ Integrated encoder brackets
- ⬜ Force sensor integration
- ⬜ ROS driver packages
- ⬜ FEA simulation files

---

## Contact

For questions, collaboration, or support:
- Open an [Issue](../../issues)
- Join our [Discussions](../../discussions)
- Email: drdelgado9@outlook.com

---

**Tags**: #robotics #gearbox #actuator #opensource #cad #manufacturing #3dprinting #cnc