# voyant-perception-toolkit
Open source perception libraries for Voyant Photonics Inc.


## Overview
This repository contains the source code for the Voyant Perception Toolkit, which provides libraries and tools like:
- **Range-Doppler Ambiguity Solver (RDAS)**: A tool for solving range-Doppler ambiguities in lidar data moving at high speeds. Specifically designed for MDL lidar systems.

## Installation

### 1. Install dependencies

Follow the steps [here](https://voyant-photonics.github.io/02_getting-started/installation.html#install-voyant-packages) to install voyant-api.

### 2. Clone the repository

```bash
git clone https://github.com/Voyant-Photonics/voyant-perception-toolkit.git
cd voyant-perception-toolkit
```

### 3. Build the project

```bash
cd src/rd_ambiguity_solver/
mkdir build
cd build
cmake ..
make
```

### 4. Run the RDAS tool

Before running the **RDAS** tool, ensure that the config file is set up correctly. You can find the configuration file in `config/solver_params.yaml`. Adjust the parameters as needed for your specific use case. The executable will be located in the `bin` directory inside the build folder. Read the [README](config/README.md) in the `config` directory for detailed parameter descriptions.

```bash
./bin/rng_dop_amb_solver ../../../config/solver_params.yaml
```
