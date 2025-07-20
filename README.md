# Radar/LiDAR Signal Processing Simulation

A comprehensive C implementation of radar and LiDAR signal processing simulation with FFT-based analysis for range and velocity extraction.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Building the Project](#building-the-project)
- [Usage](#usage)
- [Theory and Implementation](#theory-and-implementation)
- [Examples](#examples)
- [Documentation](#documentation)

## Overview

This project implements a full-scale radar and LiDAR signal processing simulation in C, featuring:

- **Pulse Signal Generation**: Creates realistic radar pulse signals with various waveforms
- **Chirp Signal Processing**: Implements linear frequency modulated (LFM) chirp signals
- **Target Simulation**: Models multiple targets with different ranges, velocities, and radar cross-sections
- **FFT Analysis**: Uses Fast Fourier Transform for time-frequency domain analysis
- **Doppler Processing**: Extracts velocity information through Doppler shift analysis
- **Range Detection**: Determines target range through time-of-flight measurements

## Features

### Core Functionality
- ✅ Pulse signal generation (rectangular, Gaussian, chirp)
- ✅ Multi-target simulation with configurable parameters
- ✅ FFT-based signal processing (using FFTW library)
- ✅ Range and velocity extraction algorithms
- ✅ Doppler shift analysis and compensation
- ✅ Signal convolution for realistic returns
- ✅ Noise modeling and SNR analysis

### Bonus Features
- ✅ Multiple simultaneous targets
- ✅ Doppler shift analysis with velocity estimation
- ✅ Range-Doppler maps generation
- ✅ Real-time signal visualization data export
- ✅ Configurable radar parameters (frequency, power, PRF)
- ✅ Advanced signal processing techniques

## Project Structure

```
radar-and-lidar-simulator/
├── src/
│   ├── main.c                 # Main program entry point
│   ├── signal_generator.c     # Signal generation functions
│   ├── target_simulator.c     # Target modeling and simulation
│   ├── fft_processor.c        # FFT and frequency domain analysis
│   ├── doppler_analyzer.c     # Doppler shift analysis
│   ├── range_detector.c       # Range detection algorithms
│   └── utils.c               # Utility functions and helpers
├── include/
│   ├── signal_types.h        # Signal structure definitions
│   ├── radar_config.h        # Configuration constants
│   ├── fft_processor.h       # FFT function declarations
│   ├── target_simulator.h    # Target simulation headers
│   ├── doppler_analyzer.h    # Doppler analysis headers
│   ├── range_detector.h      # Range detection headers
│   └── utils.h              # Utility function headers
├── examples/
│   ├── basic_radar.c         # Basic radar simulation example
│   ├── multi_target.c        # Multiple target example
│   └── doppler_demo.c        # Doppler analysis demonstration
├── docs/
│   ├── theory.md            # Mathematical theory and algorithms
│   ├── api_reference.md     # Complete API documentation
│   └── examples.md          # Usage examples and tutorials
├── tests/
│   ├── test_fft.c          # FFT functionality tests
│   ├── test_signals.c      # Signal generation tests
│   └── test_doppler.c      # Doppler analysis tests
├── Makefile                 # Build configuration
└── README.md               # This file
```

## Building the Project

### Prerequisites

- GCC compiler (MinGW on Windows)
- FFTW library for FFT operations
- Make utility

### Installation Steps

1. **Install FFTW library:**
   ```bash
   # On Windows with MinGW
   pacman -S mingw-w64-x86_64-fftw
   
   # On Linux
   sudo apt-get install libfftw3-dev
   
   # On macOS
   brew install fftw
   ```

2. **Clone and build:**
   ```bash
   git clone <repository-url>
   cd radar-and-lidar-simulator
   make all
   ```

3. **Run examples:**
   ```bash
   make run-examples
   ```

### Build Targets

- `make all` - Build main program and examples
- `make main` - Build main simulation program
- `make examples` - Build example programs
- `make tests` - Build and run test suite
- `make clean` - Clean build artifacts
- `make docs` - Generate documentation

## Usage

### Basic Radar Simulation

```bash
./radar_sim --mode radar --targets 3 --range-max 1000 --output results.csv
```

### LiDAR Simulation

```bash
./radar_sim --mode lidar --pulse-width 0.1 --frequency 905e-9 --targets 5
```

### Multi-Target Doppler Analysis

```bash
./radar_sim --doppler --targets targets.config --output doppler_map.dat
```

## Theory and Implementation

### Radar Equation

The fundamental radar equation implemented:

```
Pr = (Pt * G² * λ² * σ) / ((4π)³ * R⁴)
```

Where:
- `Pr` = Received power
- `Pt` = Transmitted power  
- `G` = Antenna gain
- `λ` = Wavelength
- `σ` = Radar cross section
- `R` = Range to target

### Signal Processing Chain

1. **Signal Generation**: Creates transmit pulses
2. **Propagation Modeling**: Simulates free-space propagation
3. **Target Interaction**: Models reflection from targets
4. **Receiver Processing**: Applies matched filtering
5. **FFT Analysis**: Transforms to frequency domain
6. **Detection**: Identifies targets above noise threshold
7. **Parameter Extraction**: Calculates range and velocity

### Doppler Shift Calculation

```
fd = (2 * vr * fc) / c
```

Where:
- `fd` = Doppler frequency shift
- `vr` = Radial velocity
- `fc` = Carrier frequency
- `c` = Speed of light

## Examples

See the `examples/` directory for detailed usage examples:

- **basic_radar.c**: Simple single-target radar simulation
- **multi_target.c**: Complex multi-target scenario
- **doppler_demo.c**: Doppler shift analysis demonstration

## Documentation

Complete documentation available in the `docs/` directory:

- **theory.md**: Mathematical foundations and algorithms
- **api_reference.md**: Detailed API documentation
- **examples.md**: Comprehensive usage examples

## Performance

- **FFT Processing**: Optimized using FFTW library
- **Memory Management**: Efficient allocation and cleanup
- **Multi-threading**: Parallel processing for large datasets
- **Vectorization**: SIMD optimizations where applicable

## Contributing

Please read the contributing guidelines and ensure all tests pass before submitting pull requests.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
