/**
 * @file signal_types.h
 * @brief Core signal type definitions and data structures for radar/LiDAR simulation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This header defines all fundamental data structures used throughout the
 * radar and LiDAR signal processing simulation system.
 */

#ifndef SIGNAL_TYPES_H
#define SIGNAL_TYPES_H

#include <complex.h>
#include <stdint.h>
#include <stdbool.h>

// Complex number definitions for C compatibility
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Complex signal sample type
 * 
 * Represents a single complex-valued signal sample with real and imaginary components.
 * Used throughout the signal processing chain for baseband and IF signals.
 */
typedef struct {
    double real;        /**< Real component of the complex sample */
    double imag;        /**< Imaginary component of the complex sample */
} ComplexSample;

/**
 * @brief Signal waveform types enumeration
 * 
 * Defines the various waveform types that can be generated and processed
 * by the simulation system.
 */
typedef enum {
    WAVEFORM_RECTANGULAR,   /**< Rectangular pulse waveform */
    WAVEFORM_GAUSSIAN,      /**< Gaussian-shaped pulse waveform */
    WAVEFORM_CHIRP_LINEAR,  /**< Linear frequency modulated (LFM) chirp */
    WAVEFORM_CHIRP_QUAD,    /**< Quadratic frequency modulated chirp */
    WAVEFORM_BARKER_CODE,   /**< Barker code modulated pulse */
    WAVEFORM_CUSTOM         /**< User-defined custom waveform */
} WaveformType;

/**
 * @brief Simulation mode enumeration
 * 
 * Defines the operational mode of the simulation system.
 */
typedef enum {
    MODE_RADAR,     /**< Radar simulation mode */
    MODE_LIDAR,     /**< LiDAR simulation mode */
    MODE_HYBRID     /**< Combined radar/LiDAR simulation */
} SimulationMode;

/**
 * @brief Signal buffer structure
 * 
 * Container for storing signal samples with metadata about the signal properties.
 * This is the primary data structure for passing signals between processing stages.
 */
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

/**
 * @brief Radar pulse parameters structure
 * 
 * Contains all parameters needed to define a radar pulse signal.
 */
typedef struct {
    double pulse_width;         /**< Pulse width in seconds */
    double pulse_repetition_freq; /**< Pulse repetition frequency in Hz */
    double carrier_frequency;   /**< Carrier frequency in Hz */
    double peak_power;          /**< Peak transmit power in watts */
    double duty_cycle;          /**< Duty cycle (0.0 to 1.0) */
    WaveformType modulation;    /**< Pulse modulation type */
    double chirp_bandwidth;     /**< Chirp bandwidth for LFM signals (Hz) */
    double phase_offset;        /**< Initial phase offset in radians */
} PulseParams;

/**
 * @brief Target characteristics structure
 * 
 * Defines the physical and motion characteristics of a simulated target.
 */
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

/**
 * @brief Antenna pattern structure
 * 
 * Defines the antenna gain pattern for directional simulations.
 */
typedef struct {
    double* azimuth_angles;     /**< Array of azimuth angles in radians */
    double* elevation_angles;   /**< Array of elevation angles in radians */
    double* gain_pattern;       /**< 2D gain pattern in dB */
    size_t azimuth_points;      /**< Number of azimuth sample points */
    size_t elevation_points;    /**< Number of elevation sample points */
    double max_gain_db;         /**< Maximum antenna gain in dB */
    double beamwidth_az;        /**< Azimuth beamwidth in radians */
    double beamwidth_el;        /**< Elevation beamwidth in radians */
} AntennaPattern;

/**
 * @brief System configuration structure
 * 
 * Contains global system parameters and configuration settings.
 */
typedef struct {
    SimulationMode mode;        /**< Operating mode (radar/LiDAR/hybrid) */
    double system_noise_temp;   /**< System noise temperature in Kelvin */
    double noise_figure_db;     /**< Receiver noise figure in dB */
    double sample_rate;         /**< System sampling rate in Hz */
    double simulation_time;     /**< Total simulation duration in seconds */
    uint32_t fft_size;          /**< FFT size for frequency analysis */
    uint32_t overlap_samples;   /**< Overlap samples for windowed processing */
    bool enable_doppler;        /**< Enable Doppler processing */
    bool enable_clutter;        /**< Enable clutter simulation */
    double range_resolution;    /**< Required range resolution in meters */
    double velocity_resolution; /**< Required velocity resolution in m/s */
} SystemConfig;

/**
 * @brief Detection result structure
 * 
 * Contains the results of target detection and parameter estimation.
 */
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

/**
 * @brief FFT processing parameters
 * 
 * Configuration for FFT-based signal processing operations.
 */
typedef struct {
    uint32_t fft_size;          /**< FFT size (power of 2) */
    uint32_t window_type;       /**< Window function type */
    double overlap_ratio;       /**< Overlap ratio for windowed FFT */
    bool zero_padding;          /**< Enable zero padding */
    uint32_t zero_pad_factor;   /**< Zero padding factor */
    bool dc_removal;            /**< Enable DC component removal */
} FFTParams;

/**
 * @brief Range-Doppler map structure
 * 
 * Contains the 2D range-Doppler map data for target analysis.
 */
typedef struct {
    double** magnitude_map;     /**< 2D magnitude map */
    double** phase_map;         /**< 2D phase map */
    double* range_bins;         /**< Range bin values in meters */
    double* doppler_bins;       /**< Doppler bin values in Hz */
    size_t range_bins_count;    /**< Number of range bins */
    size_t doppler_bins_count;  /**< Number of Doppler bins */
    double range_resolution;    /**< Range resolution per bin */
    double doppler_resolution;  /**< Doppler resolution per bin */
} RangeDopplerMap;

// Function pointer types for customizable processing
typedef void (*SignalProcessingCallback)(SignalBuffer* input, SignalBuffer* output, void* user_data);
typedef bool (*TargetDetectionCallback)(const SignalBuffer* signal, DetectionResult* result, void* user_data);

// Constants for array sizing and validation
#define MAX_TARGETS 256             /**< Maximum number of simultaneous targets */
#define MAX_SIGNAL_LENGTH 1048576   /**< Maximum signal buffer length (1M samples) */
#define MIN_SAMPLE_RATE 1000.0      /**< Minimum supported sample rate (Hz) */
#define MAX_SAMPLE_RATE 1e12        /**< Maximum supported sample rate (Hz) */
#define SPEED_OF_LIGHT 299792458.0  /**< Speed of light in m/s */

// Utility macros for complex number operations
#define COMPLEX_MAG(c) sqrt((c).real*(c).real + (c).imag*(c).imag)
#define COMPLEX_PHASE(c) atan2((c).imag, (c).real)
#define COMPLEX_CONJ(c) ((ComplexSample){(c).real, -(c).imag})
#define COMPLEX_ADD(a, b) ((ComplexSample){(a).real + (b).real, (a).imag + (b).imag})
#define COMPLEX_MUL(a, b) ((ComplexSample){(a).real*(b).real - (a).imag*(b).imag, (a).real*(b).imag + (a).imag*(b).real})

#ifdef __cplusplus
}
#endif

#endif // SIGNAL_TYPES_H
