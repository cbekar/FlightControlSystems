# CLAUDE.md - AI Assistant Guide for FlightControlSystems

## Project Overview

This is an academic flight control system design project (ITU Aerospace Engineering - AFCS Term Project) for a B737 aircraft. It implements a full pipeline from nonlinear aircraft modeling through controller design to closed-loop simulation using MATLAB/Simulink.

The project covers:
- Nonlinear 6-DOF aircraft dynamics (JSBSim format)
- Trimming and linearization across 26 flight levels
- Stability Augmentation System (SAS) design (pitch damper, roll/yaw damper)
- Control Augmentation System (CAS) design (pitch rate controller)
- Autopilot design (Pitch Attitude Hold, Altitude Hold)
- Three control design methods: Classical, Eigenvalue Assignment, and LQR/LQT

**License:** BSD 2-Clause (see `licences/license.txt`)

## Repository Structure

```
FlightControlSystems/
├── src/
│   ├── main.m                    # Main entry point - orchestrates the full pipeline
│   ├── functions/                # Control design and utility functions (15 files)
│   │   ├── getGainLUTs.m         # Gain scheduling: generates LUTs for 26 flight levels
│   │   ├── trim.m                # Trims aircraft at multiple flight conditions (fmincon)
│   │   ├── linearization.m       # Linearizes nonlinear dynamics (linmod)
│   │   ├── SAS_Pitch_EigAsgn.m   # SAS pitch controller via eigenvalue assignment
│   │   ├── SAS_Pitch_LQR.m       # SAS pitch controller via LQR
│   │   ├── SAS_RollYaw_EigAsgn.m # SAS roll/yaw damper via eigenvalue assignment
│   │   ├── SAS_RollYaw_LQR.m     # SAS roll/yaw damper via LQR
│   │   ├── CAS_Q_LQT.m           # CAS pitch rate controller via LQT
│   │   ├── LQR.m                 # Iterative LQR solver
│   │   ├── LQT.m                 # Linear Quadratic Tracker (with feedforward)
│   │   ├── fillSvalues.m         # Populates global S struct with trim/ICs for sim
│   │   ├── atmos.m               # 1976 Standard Atmosphere calculator
│   │   ├── gravity.m             # Gravitational acceleration (lat/alt)
│   │   ├── xml2struct.m          # XML parser for aircraft parameter files
│   │   └── progressbar.m         # Console progress bar utility
│   └── Zcripts/                  # Nonlinear dynamics functions (8 files)
│       ├── Zinit.m               # Initialization: loads aircraft XML, creates global S
│       ├── Zaero_fn.m            # Aerodynamic forces & moments (coefficient tables)
│       ├── Zeom_fn.m             # Equations of motion: 12-state aircraft dynamics
│       ├── ZgetStateDerivativesNonlinearSystem.m  # State derivative orchestrator
│       ├── Zatmosphere_fn.m      # Mach number and dynamic pressure
│       ├── Zengine_fn.m          # Engine thrust (throttle/alt/Mach lookup)
│       ├── Zgravity_fn.m         # Local gravity (NED to geodetic)
│       └── Zcostfn.m             # Trim cost function for optimization
├── models/                       # Simulink models (.slx)
│   ├── B737_JSB.slx              # Nonlinear JSBSim model with animation
│   ├── B737_JSB_0.slx            # Nonlinear JSBSim model without animation
│   ├── B737_PLANT.slx            # Linear plant (state-space)
│   ├── B737_SAS.slx              # Stability Augmentation System
│   ├── B737_CAS.slx              # Control Augmentation System
│   ├── B737_PAH.slx              # Pitch Attitude Hold autopilot
│   ├── B737_AH.slx               # Altitude Hold autopilot
│   └── slprj/                    # Simulink build cache (generated)
├── artifacts/
│   ├── data/                     # Aircraft/engine configuration (XML, BADA tables)
│   │   ├── B737.xml              # B737 airframe parameters (JSBSim format)
│   │   ├── CFM56.xml             # CFM56 engine parameters
│   │   └── B737PDT.txt           # BADA performance data (26 flight levels)
│   ├── mat/                      # Saved controller gains (.mat files)
│   ├── img/                      # Results visualizations (JPG, BMP, PDF)
│   └── prev/                     # Previous versions of artifacts
├── licences/
│   ├── README.md                 # Project description
│   └── license.txt               # BSD 2-Clause license
├── FlightControl.prj             # MATLAB/Simulink project file
└── .SimulinkProject/             # Simulink project metadata
```

## Language and Toolchain

- **Language:** Pure MATLAB (~1,600 lines across 23 .m files)
- **Simulation:** Simulink (7 model files)
- **No build system** - this is a MATLAB/Simulink project, not compiled code

### Required MATLAB Toolboxes

- Control Systems Toolbox (`place`, `ctrb`, `ss`, `tf`, etc.)
- Simulink (model simulation)
- Optimization Toolbox (`fmincon` for trimming)
- Aerospace Toolbox (`ned2geodetic` for coordinate conversion)
- Signal Processing Toolbox (`fixpt_interp1` for interpolation)

## How to Run

1. Open MATLAB and navigate to the project root
2. Open `FlightControl.prj` to initialize the Simulink project (adds paths automatically)
3. Edit configuration in `src/main.m`:
   ```matlab
   airframe = 'B737';   % Aircraft type
   fast = 0;            % 0 = compute from scratch, 1 = load saved gains
   modern = 2;          % 0 = Classical, 1 = EigStructure, 2 = LQR
   oppoint = 20;        % Operating point / flight level index (1-26)
   FC = 'MMCR';         % Flight condition code
   ```
4. Run `src/main.m`

### Pipeline Steps (when `fast = 0`)

1. **Zinit** - Load aircraft XML data, create global state structure `S`
2. **trim** - Trim aircraft at 26 flight levels using `fmincon` optimization
3. **linearization** - Linearize around each operating point using `linmod`
4. **getGainLUTs** - Design controllers and generate gain lookup tables
5. **save** - Persist gains to `artifacts/mat/`
6. **fillSvalues** - Populate simulation initial conditions
7. **sim** - Run Simulink closed-loop simulation

### Fast Mode (`fast = 1`)

Loads previously computed gains from `.mat` files, skipping trim/linearization/design. Useful for re-running simulations without recomputing.

## Architecture and Key Conventions

### Global State Structure

The project uses a single global MATLAB struct `S` as the shared state container:
```matlab
global S;
```
All functions read from and write to `S`. Key fields include:
- `S.AC` - Aircraft parameters (from XML)
- `S.Trim` - Trim conditions at each flight level
- `S.Lin` - Linearized state-space models
- `S.Gains` - Controller gains and lookup tables
- `S.Sim` - Simulation initial conditions

### Naming Conventions

| Pattern | Meaning |
|---------|---------|
| `Z*` prefix (e.g., `Zinit.m`) | Nonlinear dynamics functions in `src/Zcripts/` |
| `SAS_*` | Stability Augmentation System controllers |
| `CAS_*` | Control Augmentation System controllers |
| `*_EigAsgn` | Eigenvalue assignment design method |
| `*_LQR` / `*_LQT` | Optimal control design methods |
| `*LUT` suffix | Lookup table variables |
| `B737_*.slx` | Simulink model files |
| `*_fn.m` | Physics/dynamics computation functions |

### Flight Condition Codes

- `MMCR` - Medium-range cruise
- `MMCL` - Low cruise
- `MMDE` - Descent
- `LMCR` - Long-range cruise

### Control Architecture Hierarchy

```
SAS (innermost) → CAS → Autopilot (outermost)
  Pitch damper       Pitch rate (q)    Pitch Attitude Hold (PAH)
  Roll/Yaw damper                      Altitude Hold (AH)
```

## Testing and Verification

There is no formal test suite or CI/CD pipeline. Verification is done through:
- Simulink simulation of closed-loop models
- Visual inspection of root locus, step response, and impulse response plots
- Comparison of controller gains across flight levels
- Generated visualizations stored in `artifacts/img/`

## File Naming for Saved Gains

Controller gain `.mat` files follow the pattern:
```
artifacts/mat/{airframe}_{flightCondition}_{controlMethod}.mat
```
Example: `B737_MMCR_2.mat` = B737, medium-range cruise, LQR method

## Important Notes for AI Assistants

- **Do not modify `.slx` files** - Simulink models are binary and must be edited in Simulink
- **The global `S` struct is critical** - All functions depend on it; changes to its structure can break the pipeline
- **MATLAB path dependencies** - The Simulink project file (`FlightControl.prj`) manages path setup; `src/functions/` and `src/Zcripts/` must be on the MATLAB path
- **Aircraft data is JSBSim format** - XML files in `artifacts/data/` follow JSBSim conventions for aerodynamic coefficients, engine tables, etc.
- **No `.gitignore` exists** - Be cautious about adding generated files to version control
- **26 flight levels** - The BADA performance data table (`B737PDT.txt`) defines 26 flight levels from FL60 to FL390; all gain scheduling is indexed over these levels
- **`models/slprj/`** is a build cache - It contains generated Simulink artifacts and should not be manually edited
- **Comments are sparse** - Most functions have minimal inline documentation; variable names and function names carry the semantic meaning
