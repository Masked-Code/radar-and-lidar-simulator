# Radar/LiDAR Simulation API Reference

## Table of Contents

- [Overview](#overview)
- [Core Data Types](#core-data-types)
- [Signal Types](#signal-types)
- [FFT Processing](#fft-processing)
- [Target Simulation](#target-simulation)
- [Doppler Analysis](#doppler-analysis)
- [Range Detection](#range-detection)
- [Utilities](#utilities)
- [Error Handling](#error-handling)
- [Performance Considerations](#performance-considerations)

## Overview

This document provides a comprehensive reference for all APIs available in the Radar/LiDAR Signal Processing Simulation library. The library is designed to provide high-fidelity simulation capabilities for both radar and LiDAR systems with emphasis on performance, accuracy, and ease of use.

### Library Architecture

The library is organized into several functional modules:

- **Signal Types**: Core data structures and signal representations
- **FFT Processing**: Fast Fourier Transform operations and spectral analysis
- **Target Simulation**: Multi-target environment modeling and signal generation
- **Doppler Analysis**: Velocity estimation and moving target indication
- **Range Detection**: Time-of-flight measurement and target ranging
- **Utilities**: Memory management, mathematical functions, and I/O operations

### Thread Safety

Unless otherwise noted, all functions in this library are **not thread-safe**. If you need to use the library in a multi-threaded environment, you must provide your own synchronization mechanisms.

### Memory Management

The library follows a strict memory management model:
- All allocation functions return `true` on success, `false` on failure
- All structures that allocate memory have corresponding cleanup functions
- It is the caller's responsibility to call cleanup functions to prevent memory leaks
- Cleanup functions are safe to call on uninitialized structures (they will simply return)

## Core Data Types

### ComplexSample

Represents a single complex-valued signal sample.

```c
typedef struct {
    double real;        /**< Real component */
    double imag;        /**< Imaginary component */
} ComplexSample;
```

### SignalBuffer

Primary container for storing signal data with metadata.

```c
typedef struct {
    ComplexSample* samples;     /**< Array of complex signal samples */
    size_t length;              /**< Number of samples in the buffer */
    double sample_rate;         /**< Sampling rate in Hz */
    double center_frequency;    /**< Center frequency in Hz */
    double bandwidth;           /**< Signal bandwidth in Hz */
    double duration;            /**< Signal duration in seconds */
    WaveformType waveform;      /**< Type of waveform stored */
    double timestamp;           /**< Timestamp of signal start (seconds) */
    bool is_allocated;          /**< Flag indicating if memory is allocated */
} SignalBuffer;
```

### DetectionResult

Contains the results of target detection and parameter estimation.

```c
typedef struct {
    uint32_t target_id;         /**< Detected target ID */
    double detected_range;      /**< Estimated range in meters */
    double detected_velocity;   /**< Estimated velocity in m/s */
    double detected_azimuth;    /**< Estimated azimuth in radians */
    double detected_elevation;  /**< Estimated elevation in radians */
    double snr_db;              /**< Detected SNR in dB */
    double confidence;          /**< Detection confidence (0.0 to 1.0) */
    bool is_valid;              /**< Flag indicating valid detection */
    double range_error;         /**< Range estimation error in meters */
    double velocity_error;      /**< Velocity estimation error in m/s */
} DetectionResult;
```

## Signal Types

### Enumerations

#### WaveformType

Defines the various waveform types supported by the system.

```c
typedef enum {
    WAVEFORM_RECTANGULAR,   /**< Rectangular pulse waveform */
    WAVEFORM_GAUSSIAN,      /**< Gaussian-shaped pulse waveform */
    WAVEFORM_CHIRP_LINEAR,  /**< Linear frequency modulated (LFM) chirp */
    WAVEFORM_CHIRP_QUAD,    /**< Quadratic frequency modulated chirp */
    WAVEFORM_BARKER_CODE,   /**< Barker code modulated pulse */
    WAVEFORM_CUSTOM         /**< User-defined custom waveform */
} WaveformType;
```

#### SimulationMode

Defines the operational mode of the simulation system.

```c
typedef enum {
    MODE_RADAR,     /**< Radar simulation mode */
    MODE_LIDAR,     /**< LiDAR simulation mode */
    MODE_HYBRID     /**< Combined radar/LiDAR simulation */
} SimulationMode;
```

#### WindowType

Window function types for FFT processing.

```c
typedef enum {
    WINDOW_RECTANGULAR = 0,     /**< Rectangular (no) window */
    WINDOW_HAMMING,             /**< Hamming window */
    WINDOW_HANNING,             /**< Hanning window */
    WINDOW_BLACKMAN,            /**< Blackman window */
    WINDOW_KAISER,              /**< Kaiser window */
    WINDOW_GAUSSIAN,            /**< Gaussian window */
    WINDOW_TUKEY,               /**< Tukey window */
    WINDOW_BARTLETT,            /**< Bartlett window */
    WINDOW_COUNT                /**< Number of window types */
} WindowType;
```

### Complex Number Utilities

#### complex_magnitude
```c
double complex_magnitude(const ComplexSample sample);
```
**Description**: Calculates the magnitude of a complex sample.
**Parameters**: 
- `sample`: Complex sample input
**Returns**: Magnitude as a double
**Example**:
```c
ComplexSample sample = {3.0, 4.0};
double mag = complex_magnitude(sample); // Returns 5.0
```

#### complex_phase
```c
double complex_phase(const ComplexSample sample);
```
**Description**: Calculates the phase of a complex sample.
**Parameters**: 
- `sample`: Complex sample input
**Returns**: Phase in radians (-π to π)

#### complex_add
```c
ComplexSample complex_add(const ComplexSample a, const ComplexSample b);
```
**Description**: Adds two complex samples.
**Parameters**: 
- `a`: First complex sample
- `b`: Second complex sample
**Returns**: Sum of the two complex samples

#### complex_multiply
```c
ComplexSample complex_multiply(const ComplexSample a, const ComplexSample b);
```
**Description**: Multiplies two complex samples.
**Parameters**: 
- `a`: First complex sample
- `b`: Second complex sample
**Returns**: Product of the two complex samples

## FFT Processing

### Data Structures

#### FFTProcessor

Main structure for FFT operations, containing FFTW plans and workspace memory.

```c
typedef struct {
    fftw_plan forward_plan;         /**< FFTW forward transform plan */
    fftw_plan inverse_plan;         /**< FFTW inverse transform plan */
    fftw_complex* input_buffer;     /**< Input buffer for FFTW */
    fftw_complex* output_buffer;    /**< Output buffer for FFTW */
    double* window_function;        /**< Window function coefficients */
    uint32_t fft_size;              /**< FFT size (must be power of 2) */
    WindowType window_type;         /**< Type of window function applied */
    bool is_initialized;            /**< Initialization status flag */
    double scale_factor;            /**< Scaling factor for normalization */
} FFTProcessor;
```

#### SpectralAnalysis

Contains the results of spectral analysis operations.

```c
typedef struct {
    double* frequency_bins;         /**< Frequency bin values (Hz) */
    double* magnitude_spectrum;     /**< Magnitude spectrum */
    double* phase_spectrum;         /**< Phase spectrum (radians) */
    double* power_spectrum;         /**< Power spectral density */
    uint32_t spectrum_length;       /**< Number of spectral points */
    double frequency_resolution;    /**< Frequency resolution (Hz/bin) */
    double peak_frequency;          /**< Peak frequency (Hz) */
    double peak_magnitude;          /**< Peak magnitude */
    uint32_t peak_bin;              /**< Peak bin index */
    double total_power;             /**< Total signal power */
    double noise_floor;             /**< Estimated noise floor */
} SpectralAnalysis;
```

### Core Functions

#### fft_processor_init
```c
bool fft_processor_init(FFTProcessor* processor, uint32_t fft_size, WindowType window_type);
```
**Description**: Initializes an FFT processor with specified parameters. Creates FFTW plans and allocates memory for FFT operations.

**Parameters**:
- `processor`: Pointer to FFT processor structure
- `fft_size`: Size of FFT (must be power of 2)
- `window_type`: Type of window function to apply

**Returns**: `true` if initialization successful, `false` otherwise

**Thread Safety**: Not thread-safe

**Example**:
```c
FFTProcessor processor;
if (fft_processor_init(&processor, 1024, WINDOW_HAMMING)) {
    // Processor ready for use
    fft_processor_cleanup(&processor);
}
```

#### fft_forward
```c
bool fft_forward(FFTProcessor* processor, const SignalBuffer* input, SignalBuffer* output);
```
**Description**: Computes forward FFT of signal buffer. Applies window function and transforms signal from time domain to frequency domain.

**Parameters**:
- `processor`: Pointer to initialized FFT processor
- `input`: Input signal buffer (time domain)
- `output`: Output signal buffer (frequency domain)

**Returns**: `true` if FFT computation successful, `false` otherwise

**Preconditions**: 
- Processor must be initialized
- Input buffer must be allocated with length ≥ FFT size
- Output buffer must be allocated with length ≥ FFT size

#### spectral_analysis
```c
bool spectral_analysis(FFTProcessor* processor, const SignalBuffer* signal, SpectralAnalysis* analysis);
```
**Description**: Performs comprehensive spectral analysis of signal including magnitude spectrum, phase spectrum, power spectral density, and peak detection.

**Parameters**:
- `processor`: Pointer to initialized FFT processor
- `signal`: Input signal buffer
- `analysis`: Output spectral analysis results

**Returns**: `true` if analysis successful, `false` otherwise

**Note**: Caller must call `spectral_analysis_cleanup()` to free allocated memory.

### Window Functions

#### generate_window
```c
bool generate_window(WindowType window_type, uint32_t window_length, double* coefficients, double beta);
```
**Description**: Generates window function coefficients for the specified window type and length.

**Parameters**:
- `window_type`: Type of window function
- `window_length`: Length of window function
- `coefficients`: Output array for window coefficients (must be pre-allocated)
- `beta`: Kaiser window beta parameter (ignored for other windows)

**Returns**: `true` if window generation successful, `false` otherwise

**Supported Windows**:
- `WINDOW_RECTANGULAR`: Uniform weighting (no windowing)
- `WINDOW_HAMMING`: Hamming window (α = 0.54)
- `WINDOW_HANNING`: Hanning window
- `WINDOW_BLACKMAN`: Blackman window
- `WINDOW_KAISER`: Kaiser window (requires beta parameter)
- `WINDOW_GAUSSIAN`: Gaussian window
- `WINDOW_BARTLETT`: Bartlett (triangular) window

## Target Simulation

### Data Structures

#### TargetParams

Defines the physical and motion characteristics of a simulated target.

```c
typedef struct {
    uint32_t id;                /**< Unique target identifier */
    double range;               /**< Range to target in meters */
    double velocity;            /**< Radial velocity in m/s (positive = approaching) */
    double azimuth;             /**< Azimuth angle in radians */
    double elevation;           /**< Elevation angle in radians */
    double rcs;                 /**< Radar cross section in m² */
    double reflectivity;        /**< LiDAR reflectivity coefficient (0.0 to 1.0) */
    bool is_active;             /**< Flag indicating if target is active */
    double acceleration;        /**< Radial acceleration in m/s² */
    double snr_db;              /**< Signal-to-noise ratio in dB */
} TargetParams;
```

#### SimulationEnvironment

Contains all targets and environmental parameters for a complete simulation scenario.

```c
typedef struct {
    TargetParams* targets;          /**< Array of target parameters */
    TargetMotion* target_motions;   /**< Array of target motion models */
    uint32_t target_count;          /**< Number of active targets */
    uint32_t max_targets;           /**< Maximum number of targets */
    double simulation_time;         /**< Current simulation time */
    double time_step;               /**< Simulation time step */
    bool enable_clutter;            /**< Enable clutter simulation */
    double clutter_density;         /**< Clutter density (targets/km²) */
    double atmospheric_loss_db;     /**< Atmospheric loss in dB/km */
    double rain_rate_mm_hr;         /**< Rain rate in mm/hr */
    double temperature_kelvin;      /**< Environmental temperature */
    double humidity_percent;        /**< Relative humidity percentage */
} SimulationEnvironment;
```

### Core Functions

#### simulation_environment_init
```c
bool simulation_environment_init(SimulationEnvironment* env, uint32_t max_targets, double time_step);
```
**Description**: Initializes a simulation environment with specified parameters and allocates memory for the specified number of targets.

**Parameters**:
- `env`: Pointer to simulation environment structure
- `max_targets`: Maximum number of targets to support
- `time_step`: Simulation time step in seconds

**Returns**: `true` if initialization successful, `false` otherwise

#### add_target
```c
uint32_t add_target(SimulationEnvironment* env, const TargetParams* target_params, 
                   const TargetMotion* motion_params);
```
**Description**: Adds a new target with specified parameters to the simulation. The target is assigned a unique ID and added to the active target list.

**Parameters**:
- `env`: Pointer to simulation environment
- `target_params`: Target parameters structure
- `motion_params`: Initial target motion parameters

**Returns**: Target ID if successful, 0 if failed

**Example**:
```c
TargetParams target = {
    .id = 0,
    .range = 1000.0,
    .velocity = 50.0,
    .azimuth = 0.0,
    .elevation = 0.0,
    .rcs = 1.0,
    .reflectivity = 0.5,
    .is_active = true
};
TargetMotion motion = {0}; // Initialize to zero

uint32_t target_id = add_target(&env, &target, &motion);
if (target_id > 0) {
    // Target added successfully
}
```

#### generate_multi_target_signal
```c
bool generate_multi_target_signal(const SimulationEnvironment* env, const SignalBuffer* tx_signal,
                                 SignalBuffer* rx_signal, const PulseParams* pulse_params);
```
**Description**: Creates a composite received signal containing echoes from all active targets in the simulation environment.

**Parameters**:
- `env`: Pointer to simulation environment
- `tx_signal`: Transmitted signal
- `rx_signal`: Output composite received signal
- `pulse_params`: Radar pulse parameters

**Returns**: `true` if signal generation successful, `false` otherwise

### Propagation Modeling

#### calculate_path_loss
```c
double calculate_path_loss(double range, double frequency);
```
**Description**: Computes the free space path loss between radar and target using the Friis transmission equation.

**Parameters**:
- `range`: Range to target in meters
- `frequency`: Operating frequency in Hz

**Returns**: Path loss in dB

**Formula**: 
```
PL = 20 * log₁₀(4πRf/c)
```
Where R is range, f is frequency, and c is speed of light.

#### calculate_atmospheric_attenuation
```c
double calculate_atmospheric_attenuation(double range, double frequency, double temperature,
                                        double humidity, double pressure);
```
**Description**: Computes atmospheric attenuation due to absorption and scattering based on frequency, range, and atmospheric conditions.

**Parameters**:
- `range`: Range to target in meters
- `frequency`: Operating frequency in Hz
- `temperature`: Temperature in Kelvin
- `humidity`: Relative humidity (0.0 to 1.0)
- `pressure`: Atmospheric pressure in Pa

**Returns**: Atmospheric attenuation in dB

## Doppler Analysis

### Data Structures

#### DopplerParams

Configuration parameters for Doppler analysis and velocity estimation.

```c
typedef struct {
    double carrier_frequency;       /**< Radar carrier frequency (Hz) */
    double prf;                     /**< Pulse repetition frequency (Hz) */
    uint32_t coherent_pulses;       /**< Number of coherent integration pulses */
    uint32_t doppler_fft_size;      /**< FFT size for Doppler processing */
    double max_unambiguous_velocity; /**< Maximum unambiguous velocity (m/s) */
    double velocity_resolution;     /**< Velocity resolution (m/s) */
    bool enable_mti;                /**< Enable moving target indication */
    uint32_t mti_order;             /**< MTI filter order */
    double clutter_threshold_db;    /**< Clutter rejection threshold (dB) */
} DopplerParams;
```

#### DopplerAnalysis

Contains the results of Doppler processing including velocity estimates and confidence metrics.

```c
typedef struct {
    double* velocity_bins;          /**< Velocity bin values (m/s) */
    double* doppler_spectrum;       /**< Doppler spectrum magnitude */
    double* doppler_phase;          /**< Doppler spectrum phase */
    uint32_t spectrum_length;       /**< Length of Doppler spectrum */
    double peak_velocity;           /**< Peak velocity estimate (m/s) */
    double peak_magnitude;          /**< Peak magnitude */
    uint32_t peak_bin;              /**< Peak bin index */
    double velocity_spread;         /**< Velocity spread estimate (m/s) */
    double snr_db;                  /**< Signal-to-noise ratio (dB) */
    double confidence;              /**< Velocity estimate confidence */
    bool velocity_ambiguous;        /**< Flag indicating velocity ambiguity */
    uint32_t num_peaks_detected;    /**< Number of detected velocity peaks */
} DopplerAnalysis;
```

### Core Functions

#### doppler_params_init
```c
bool doppler_params_init(DopplerParams* params, double carrier_freq, double prf, uint32_t num_pulses);
```
**Description**: Sets up Doppler processing parameters based on radar system configuration.

**Parameters**:
- `params`: Pointer to Doppler parameters structure
- `carrier_freq`: Radar carrier frequency (Hz)
- `prf`: Pulse repetition frequency (Hz)
- `num_pulses`: Number of pulses for coherent processing

**Returns**: `true` if initialization successful, `false` otherwise

#### velocity_to_doppler
```c
double velocity_to_doppler(double velocity, double carrier_frequency);
```
**Description**: Converts radial velocity to Doppler frequency shift using the radar equation.

**Parameters**:
- `velocity`: Radial velocity (m/s, positive = approaching)
- `carrier_frequency`: Radar carrier frequency (Hz)

**Returns**: Doppler frequency shift (Hz)

**Formula**: 
```
fd = 2 * v * fc / c
```

#### doppler_to_velocity
```c
double doppler_to_velocity(double doppler_freq, double carrier_frequency);
```
**Description**: Converts Doppler frequency shift to radial velocity.

**Parameters**:
- `doppler_freq`: Doppler frequency shift (Hz)
- `carrier_frequency`: Radar carrier frequency (Hz)

**Returns**: Radial velocity (m/s, positive = approaching)

#### analyze_doppler
```c
bool analyze_doppler(const SignalBuffer* pulse_train, uint32_t num_pulses, 
                     const DopplerParams* params, DopplerAnalysis* analysis);
```
**Description**: Analyzes a coherent pulse train to extract Doppler information and estimate target velocities.

**Parameters**:
- `pulse_train`: Array of received pulse signals
- `num_pulses`: Number of pulses in the train
- `params`: Doppler processing parameters
- `analysis`: Output Doppler analysis results

**Returns**: `true` if analysis successful, `false` otherwise

**Note**: Caller must call `doppler_analysis_cleanup()` to free allocated memory.

## Range Detection

### Data Structures

#### RangeParams

Configuration parameters for range detection and measurement algorithms.

```c
typedef struct {
    double sample_rate;             /**< Signal sampling rate (Hz) */
    double pulse_width;             /**< Transmitted pulse width (s) */
    double chirp_bandwidth;         /**< Chirp bandwidth for pulse compression (Hz) */
    double range_resolution;        /**< Desired range resolution (m) */
    double max_range;               /**< Maximum detection range (m) */
    uint32_t num_range_gates;       /**< Number of range gates */
    double gate_spacing;            /**< Range gate spacing (m) */
    double detection_threshold_db;  /**< Detection threshold above noise (dB) */
    bool enable_pulse_compression;  /**< Enable pulse compression processing */
    bool enable_cfar;               /**< Enable CFAR detection */
} RangeParams;
```

#### RangeMeasurement

Contains the results of range detection and measurement operations.

```c
typedef struct {
    double* range_bins;             /**< Range bin values (m) */
    double* range_profile;          /**< Range profile magnitude */
    double* range_phase;            /**< Range profile phase */
    uint32_t profile_length;        /**< Length of range profile */
    double detected_range;          /**< Primary detected range (m) */
    double range_accuracy;          /**< Range measurement accuracy (m) */
    double peak_magnitude;          /**< Peak detection magnitude */
    uint32_t peak_bin;              /**< Peak range bin index */
    double snr_db;                  /**< Signal-to-noise ratio (dB) */
    uint32_t num_detections;        /**< Number of range detections */
    double* detection_ranges;       /**< Array of detected ranges (m) */
    double* detection_magnitudes;   /**< Array of detection magnitudes */
} RangeMeasurement;
```

### Core Functions

#### range_params_init
```c
bool range_params_init(RangeParams* params, double sample_rate, double pulse_width, double max_range);
```
**Description**: Sets up range detection parameters based on system configuration.

**Parameters**:
- `params`: Pointer to range parameters structure
- `sample_rate`: Signal sampling rate (Hz)
- `pulse_width`: Transmitted pulse width (s)
- `max_range`: Maximum detection range (m)

**Returns**: `true` if initialization successful, `false` otherwise

#### time_delay_to_range
```c
double time_delay_to_range(double time_delay, bool is_two_way);
```
**Description**: Converts time delay measurement to range using speed of light.

**Parameters**:
- `time_delay`: Time delay in seconds
- `is_two_way`: Flag indicating if delay is two-way (radar) or one-way (LiDAR)

**Returns**: Range in meters

**Formula**:
- Two-way (radar): `R = c * t / 2`
- One-way (LiDAR): `R = c * t`

#### generate_range_profile
```c
bool generate_range_profile(const SignalBuffer* rx_signal, const RangeParams* params,
                           RangeMeasurement* measurement);
```
**Description**: Creates a range profile showing signal power vs. range from received signal.

**Parameters**:
- `rx_signal`: Received signal buffer
- `params`: Range detection parameters
- `measurement`: Output range measurement with profile

**Returns**: `true` if profile generation successful, `false` otherwise

## Utilities

### Memory Management

#### allocate_signal_buffer
```c
bool allocate_signal_buffer(SignalBuffer* buffer, size_t length, double sample_rate);
```
**Description**: Allocates memory for a signal buffer with specified length and sample rate.

**Parameters**:
- `buffer`: Pointer to signal buffer structure
- `length`: Number of samples to allocate
- `sample_rate`: Sampling rate in Hz

**Returns**: `true` if allocation successful, `false` otherwise

**Post-conditions**: 
- If successful, `buffer->is_allocated` will be `true`
- All samples will be initialized to zero
- Caller must call `free_signal_buffer()` to prevent memory leaks

#### free_signal_buffer
```c
void free_signal_buffer(SignalBuffer* buffer);
```
**Description**: Frees memory allocated for a signal buffer and resets structure fields.

**Parameters**:
- `buffer`: Pointer to signal buffer structure

**Post-conditions**: 
- All allocated memory is freed
- `buffer->samples` will be `NULL`
- `buffer->is_allocated` will be `false`

### Mathematical Functions

#### random_uniform
```c
double random_uniform(double min, double max);
```
**Description**: Generates uniformly distributed random numbers between min and max.

**Parameters**:
- `min`: Minimum value
- `max`: Maximum value

**Returns**: Random value between min and max

**Thread Safety**: Not thread-safe (uses global random state)

#### random_normal
```c
double random_normal(double mean, double std_dev);
```
**Description**: Generates normally distributed random numbers using Box-Muller transform.

**Parameters**:
- `mean`: Mean of distribution
- `std_dev`: Standard deviation

**Returns**: Random value from normal distribution

#### calculate_signal_power
```c
double calculate_signal_power(const SignalBuffer* signal);
```
**Description**: Calculates average signal power (mean squared magnitude).

**Parameters**:
- `signal`: Input signal buffer

**Returns**: Signal power (linear scale)

**Formula**: 
```
P = (1/N) * Σ|x[n]|²
```

### File I/O

#### save_signal_to_file
```c
bool save_signal_to_file(const SignalBuffer* signal, const char* filename);
```
**Description**: Saves signal buffer to binary file format.

**Parameters**:
- `signal`: Signal buffer to save
- `filename`: Output filename

**Returns**: `true` if save successful, `false` otherwise

**File Format**: Custom binary format with header containing signal metadata followed by complex samples.

#### export_signal_to_csv
```c
bool export_signal_to_csv(const SignalBuffer* signal, const char* filename, bool include_phase);
```
**Description**: Exports signal to CSV format for analysis in external tools.

**Parameters**:
- `signal`: Signal buffer to export
- `filename`: Output CSV filename
- `include_phase`: Flag to include phase information

**Returns**: `true` if export successful, `false` otherwise

**CSV Format**:
```
Sample,Real,Imaginary,Magnitude[,Phase]
0,1.234,-0.567,1.357[,0.432]
...
```

### Timing Functions

#### timer_start
```c
bool timer_start(HighResTimer* timer);
```
**Description**: Starts a high-resolution timer for performance measurement.

**Parameters**:
- `timer`: Timer structure

**Returns**: `true` if timer started successfully, `false` otherwise

#### timer_get_elapsed_seconds
```c
double timer_get_elapsed_seconds(const HighResTimer* timer);
```
**Description**: Gets elapsed time in seconds since timer was started.

**Parameters**:
- `timer`: Timer structure

**Returns**: Elapsed time in seconds

**Precision**: Microsecond precision on most systems

## Error Handling

### Return Value Conventions

The library follows consistent error handling conventions:

- **Boolean Functions**: Return `true` on success, `false` on failure
- **Pointer Functions**: Return valid pointer on success, `NULL` on failure  
- **Numeric Functions**: Return valid result on success, `NaN` or `HUGE_VAL` on mathematical errors
- **ID Functions**: Return non-zero ID on success, `0` on failure

### Common Error Conditions

1. **Memory Allocation Failures**: 
   - Check return values of all allocation functions
   - Call cleanup functions even if initialization fails

2. **Invalid Parameters**:
   - NULL pointer arguments where non-NULL expected
   - Invalid sizes (zero or negative where positive expected)
   - Out-of-range values (frequencies, sample rates, etc.)

3. **Uninitialized Structures**:
   - Using processors before calling init functions
   - Calling functions on cleaned-up structures

### Error Checking Example

```c
FFTProcessor processor;
SignalBuffer input, output;

// Check all allocations and initializations
if (!fft_processor_init(&processor, 1024, WINDOW_HAMMING)) {
    fprintf(stderr, "Failed to initialize FFT processor\n");
    return false;
}

if (!allocate_signal_buffer(&input, 1024, 48000.0)) {
    fprintf(stderr, "Failed to allocate input buffer\n");
    fft_processor_cleanup(&processor);
    return false;
}

if (!allocate_signal_buffer(&output, 1024, 48000.0)) {
    fprintf(stderr, "Failed to allocate output buffer\n");
    free_signal_buffer(&input);
    fft_processor_cleanup(&processor);
    return false;
}

// Perform processing
bool success = fft_forward(&processor, &input, &output);

// Always cleanup
free_signal_buffer(&input);
free_signal_buffer(&output);
fft_processor_cleanup(&processor);

return success;
```

## Performance Considerations

### Memory Usage

- **Signal Buffers**: Each complex sample uses 16 bytes (2 × 8-byte doubles)
- **FFT Processors**: Allocate approximately 4× FFT size in working memory
- **Large Simulations**: Use streaming processing for signals > 1M samples

### Computational Complexity

- **FFT Operations**: O(N log N) where N is FFT size
- **Signal Generation**: O(N) where N is signal length
- **Multi-Target Simulation**: O(M × N) where M is number of targets
- **Doppler Analysis**: O(P × N log N) where P is number of pulses

### Optimization Tips

1. **Choose Appropriate FFT Sizes**: Powers of 2 are most efficient
2. **Reuse Processors**: Initialize once, use many times
3. **Memory Alignment**: Use aligned allocation for better SIMD performance
4. **Window Functions**: Pre-compute and reuse when possible
5. **Batch Processing**: Process multiple signals together when possible

### Threading Considerations

While individual functions are not thread-safe, the library can be used effectively in multi-threaded applications:

- **Parallel Target Processing**: Each thread can simulate different targets
- **Pipeline Processing**: Use producer-consumer patterns with thread-safe queues
- **Independent Processors**: Each thread can have its own FFT processor instance

```c
// Thread-safe usage pattern
typedef struct {
    FFTProcessor processor;
    SignalBuffer input;
    SignalBuffer output;
    bool initialized;
} ThreadData;

// Initialize per-thread data
bool init_thread_data(ThreadData* data, uint32_t fft_size) {
    data->initialized = false;
    
    if (!fft_processor_init(&data->processor, fft_size, WINDOW_HAMMING))
        return false;
        
    if (!allocate_signal_buffer(&data->input, fft_size, 48000.0)) {
        fft_processor_cleanup(&data->processor);
        return false;
    }
    
    if (!allocate_signal_buffer(&data->output, fft_size, 48000.0)) {
        free_signal_buffer(&data->input);
        fft_processor_cleanup(&data->processor);
        return false;
    }
    
    data->initialized = true;
    return true;
}

// Cleanup per-thread data
void cleanup_thread_data(ThreadData* data) {
    if (data->initialized) {
        free_signal_buffer(&data->input);
        free_signal_buffer(&data->output);
        fft_processor_cleanup(&data->processor);
        data->initialized = false;
    }
}
```

---

*This document covers the major APIs available in the Radar/LiDAR Simulation library. For additional details on specific functions or advanced usage patterns, please refer to the header files and example code.*
