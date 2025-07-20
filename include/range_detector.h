/**
 * @file range_detector.h
 * @brief Range detection and measurement functions for radar/LiDAR systems
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This header provides functions for range detection, measurement, and
 * time-of-flight analysis for radar and LiDAR systems.
 */

#ifndef RANGE_DETECTOR_H
#define RANGE_DETECTOR_H

#include "signal_types.h"
#include "radar_config.h"
#include "fft_processor.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// RANGE DETECTION STRUCTURES
// ============================================================================

/**
 * @brief Range detection parameters
 * 
 * Configuration parameters for range detection and measurement algorithms.
 */
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

/**
 * @brief Range measurement results
 * 
 * Contains the results of range detection and measurement operations.
 */
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

/**
 * @brief Pulse compression filter
 * 
 * Matched filter implementation for pulse compression processing.
 */
typedef struct {
    ComplexSample* reference_pulse; /**< Reference pulse for matching */
    ComplexSample* compressed_pulse; /**< Output compressed pulse */
    uint32_t pulse_length;          /**< Length of reference pulse */
    FFTProcessor* fft_processor;    /**< FFT processor for convolution */
    double compression_ratio;       /**< Pulse compression ratio */
    double processing_gain_db;      /**< Processing gain (dB) */
    bool is_initialized;            /**< Initialization status */
} PulseCompressionFilter;

/**
 * @brief CFAR (Constant False Alarm Rate) detector
 * 
 * Adaptive threshold detector for maintaining constant false alarm rate.
 */
typedef struct {
    double* reference_cells;        /**< Reference cell data */
    uint32_t num_reference_cells;   /**< Number of reference cells */
    uint32_t num_guard_cells;       /**< Number of guard cells */
    double threshold_factor;        /**< CFAR threshold factor */
    double false_alarm_rate;        /**< Target false alarm rate */
    uint32_t detector_type;         /**< CFAR detector type (CA, OS, etc.) */
    bool is_initialized;            /**< Initialization status */
} CFARDetector;

/**
 * @brief Range tracking filter
 * 
 * Kalman filter for range tracking and prediction.
 */
typedef struct {
    double* state_vector;           /**< State vector [range, range_rate] */
    double* state_covariance;       /**< State covariance matrix */
    double* process_noise;          /**< Process noise covariance */
    double measurement_noise;       /**< Measurement noise variance */
    double* kalman_gain;            /**< Kalman gain matrix */
    bool is_initialized;            /**< Initialization status */
} RangeTracker;

// ============================================================================
// RANGE DETECTION FUNCTIONS
// ============================================================================

/**
 * @brief Initialize range detection parameters
 * 
 * Sets up range detection parameters based on system configuration.
 * 
 * @param params Pointer to range parameters structure
 * @param sample_rate Signal sampling rate (Hz)
 * @param pulse_width Transmitted pulse width (s)
 * @param max_range Maximum detection range (m)
 * @return true if initialization successful, false otherwise
 */
bool range_params_init(RangeParams* params, double sample_rate, double pulse_width, double max_range);

/**
 * @brief Perform basic range detection using time-of-flight
 * 
 * Detects targets by measuring time delay between transmitted and received pulses.
 * 
 * @param tx_signal Transmitted signal buffer
 * @param rx_signal Received signal buffer
 * @param params Range detection parameters
 * @param measurement Output range measurement results
 * @return true if detection successful, false otherwise
 */
bool detect_range_basic(const SignalBuffer* tx_signal, const SignalBuffer* rx_signal,
                        const RangeParams* params, RangeMeasurement* measurement);

/**
 * @brief Generate range profile from received signal
 * 
 * Creates a range profile showing signal power vs. range.
 * 
 * @param rx_signal Received signal buffer
 * @param params Range detection parameters
 * @param measurement Output range measurement with profile
 * @return true if profile generation successful, false otherwise
 */
bool generate_range_profile(const SignalBuffer* rx_signal, const RangeParams* params,
                           RangeMeasurement* measurement);

/**
 * @brief Calculate range from time delay
 * 
 * Converts time delay measurement to range using speed of light.
 * 
 * @param time_delay Time delay in seconds
 * @param is_two_way Flag indicating if delay is two-way (radar) or one-way (LiDAR)
 * @return Range in meters
 */
double time_delay_to_range(double time_delay, bool is_two_way);

/**
 * @brief Calculate time delay from range
 * 
 * Converts range measurement to time delay.
 * 
 * @param range Range in meters
 * @param is_two_way Flag indicating if calculation is for two-way or one-way propagation
 * @return Time delay in seconds
 */
double range_to_time_delay(double range, bool is_two_way);

// ============================================================================
// PULSE COMPRESSION
// ============================================================================

/**
 * @brief Initialize pulse compression filter
 * 
 * Sets up matched filter for pulse compression processing.
 * 
 * @param filter Pointer to pulse compression filter structure
 * @param reference_pulse Reference pulse for matched filtering
 * @param fft_size FFT size for frequency domain processing
 * @return true if initialization successful, false otherwise
 */
bool pulse_compression_init(PulseCompressionFilter* filter, const SignalBuffer* reference_pulse,
                           uint32_t fft_size);

/**
 * @brief Cleanup pulse compression filter
 * 
 * @param filter Pointer to pulse compression filter structure
 */
void pulse_compression_cleanup(PulseCompressionFilter* filter);

/**
 * @brief Apply pulse compression to received signal
 * 
 * Performs matched filtering to compress transmitted chirp and improve range resolution.
 * 
 * @param filter Pointer to initialized pulse compression filter
 * @param rx_signal Received signal buffer
 * @param compressed_signal Output compressed signal
 * @return true if compression successful, false otherwise
 */
bool apply_pulse_compression(PulseCompressionFilter* filter, const SignalBuffer* rx_signal,
                            SignalBuffer* compressed_signal);

/**
 * @brief Calculate pulse compression ratio
 * 
 * Computes the pulse compression ratio based on pulse width and bandwidth.
 * 
 * @param pulse_width Pulse width in seconds
 * @param bandwidth Signal bandwidth in Hz
 * @return Pulse compression ratio
 */
double calculate_compression_ratio(double pulse_width, double bandwidth);

/**
 * @brief Calculate pulse compression gain
 * 
 * Computes the processing gain achieved through pulse compression.
 * 
 * @param compression_ratio Pulse compression ratio
 * @return Processing gain in dB
 */
double calculate_compression_gain(double compression_ratio);

// ============================================================================
// CFAR DETECTION
// ============================================================================

/**
 * @brief Initialize CFAR detector
 * 
 * Sets up CFAR detector with specified parameters.
 * 
 * @param detector Pointer to CFAR detector structure
 * @param num_reference_cells Number of reference cells
 * @param num_guard_cells Number of guard cells
 * @param false_alarm_rate Target false alarm rate
 * @param detector_type CFAR detector type (0=CA-CFAR, 1=OS-CFAR, etc.)
 * @return true if initialization successful, false otherwise
 */
bool cfar_detector_init(CFARDetector* detector, uint32_t num_reference_cells,
                        uint32_t num_guard_cells, double false_alarm_rate, uint32_t detector_type);

/**
 * @brief Cleanup CFAR detector
 * 
 * @param detector Pointer to CFAR detector structure
 */
void cfar_detector_cleanup(CFARDetector* detector);

/**
 * @brief Apply CFAR detection to range profile
 * 
 * Performs adaptive threshold detection using CFAR algorithm.
 * 
 * @param detector Pointer to initialized CFAR detector
 * @param range_profile Input range profile data
 * @param profile_length Length of range profile
 * @param detections Output detection flags (1=detection, 0=no detection)
 * @param thresholds Output adaptive threshold values
 * @return Number of detections found
 */
uint32_t apply_cfar_detection(CFARDetector* detector, const double* range_profile,
                             uint32_t profile_length, uint8_t* detections, double* thresholds);

/**
 * @brief Calculate CFAR threshold factor
 * 
 * Computes threshold factor for desired false alarm rate.
 * 
 * @param false_alarm_rate Desired false alarm rate
 * @param num_reference_cells Number of reference cells
 * @param detector_type CFAR detector type
 * @return Threshold factor
 */
double calculate_cfar_threshold(double false_alarm_rate, uint32_t num_reference_cells, 
                               uint32_t detector_type);

// ============================================================================
// RANGE TRACKING
// ============================================================================

/**
 * @brief Initialize range tracking filter
 * 
 * Sets up Kalman filter for range tracking.
 * 
 * @param tracker Pointer to range tracker structure
 * @param initial_range Initial range estimate (m)
 * @param initial_velocity Initial range rate estimate (m/s)
 * @param process_noise_std Process noise standard deviation
 * @param measurement_noise_std Measurement noise standard deviation
 * @return true if initialization successful, false otherwise
 */
bool range_tracker_init(RangeTracker* tracker, double initial_range, double initial_velocity,
                        double process_noise_std, double measurement_noise_std);

/**
 * @brief Cleanup range tracker
 * 
 * @param tracker Pointer to range tracker structure
 */
void range_tracker_cleanup(RangeTracker* tracker);

/**
 * @brief Update range tracker with new measurement
 * 
 * Performs Kalman filter update with new range measurement.
 * 
 * @param tracker Pointer to range tracker structure
 * @param measurement New range measurement (m)
 * @param time_step Time step since last update (s)
 * @param filtered_range Output filtered range estimate (m)
 * @param filtered_velocity Output filtered velocity estimate (m/s)
 * @return true if update successful, false otherwise
 */
bool range_tracker_update(RangeTracker* tracker, double measurement, double time_step,
                         double* filtered_range, double* filtered_velocity);

/**
 * @brief Predict next range measurement
 * 
 * Uses tracker state to predict next range measurement.
 * 
 * @param tracker Pointer to range tracker structure
 * @param time_step Prediction time step (s)
 * @param predicted_range Output predicted range (m)
 * @param prediction_uncertainty Output prediction uncertainty (m)
 * @return true if prediction successful, false otherwise
 */
bool range_tracker_predict(RangeTracker* tracker, double time_step,
                          double* predicted_range, double* prediction_uncertainty);

// ============================================================================
// ADVANCED RANGE PROCESSING
// ============================================================================

/**
 * @brief Perform high-resolution range estimation
 * 
 * Uses super-resolution techniques for improved range accuracy.
 * 
 * @param range_profile Input range profile
 * @param profile_length Length of range profile
 * @param params Range detection parameters
 * @param high_res_ranges Output high-resolution range estimates
 * @param max_detections Maximum number of detections to return
 * @return Number of high-resolution detections found
 */
uint32_t high_resolution_range_estimation(const double* range_profile, uint32_t profile_length,
                                         const RangeParams* params, double* high_res_ranges,
                                         uint32_t max_detections);

/**
 * @brief Estimate range measurement accuracy
 * 
 * Calculates theoretical range measurement accuracy based on SNR and bandwidth.
 * 
 * @param snr_db Signal-to-noise ratio in dB
 * @param bandwidth Signal bandwidth in Hz
 * @param pulse_width Pulse width in seconds
 * @return Range measurement accuracy (m)
 */
double estimate_range_accuracy(double snr_db, double bandwidth, double pulse_width);

/**
 * @brief Detect multiple targets in same range gate
 * 
 * Uses spectral estimation to resolve multiple targets within a single range gate.
 * 
 * @param range_gate_data Complex data from single range gate
 * @param gate_length Length of range gate data
 * @param target_ranges Output array of detected target ranges
 * @param target_magnitudes Output array of target magnitudes
 * @param max_targets Maximum number of targets to detect
 * @return Number of targets detected
 */
uint32_t detect_multiple_targets_in_gate(const ComplexSample* range_gate_data, uint32_t gate_length,
                                        double* target_ranges, double* target_magnitudes,
                                        uint32_t max_targets);

// ============================================================================
// RANGE AMBIGUITY RESOLUTION
// ============================================================================

/**
 * @brief Resolve range ambiguity using multiple PRF
 * 
 * Uses measurements at different PRFs to resolve range ambiguity.
 * 
 * @param ambiguous_ranges Array of ambiguous range measurements
 * @param prfs Array of corresponding PRF values
 * @param num_measurements Number of measurements
 * @param max_unambiguous_range Maximum unambiguous range
 * @param resolved_range Output resolved range
 * @return true if ambiguity resolved successfully, false otherwise
 */
bool resolve_range_ambiguity(const double* ambiguous_ranges, const double* prfs,
                            uint32_t num_measurements, double max_unambiguous_range,
                            double* resolved_range);

/**
 * @brief Calculate maximum unambiguous range
 * 
 * Computes maximum unambiguous range based on PRF.
 * 
 * @param prf Pulse repetition frequency (Hz)
 * @param is_two_way Flag indicating two-way propagation
 * @return Maximum unambiguous range (m)
 */
double calculate_max_unambiguous_range(double prf, bool is_two_way);

// ============================================================================
// CALIBRATION AND CORRECTION
// ============================================================================

/**
 * @brief Apply range calibration correction
 * 
 * Corrects range measurements for systematic errors.
 * 
 * @param raw_range Raw range measurement (m)
 * @param calibration_offset Range calibration offset (m)
 * @param scale_factor Range scale factor correction
 * @return Calibrated range measurement (m)
 */
double apply_range_calibration(double raw_range, double calibration_offset, double scale_factor);

/**
 * @brief Compensate for atmospheric delay
 * 
 * Corrects range measurement for atmospheric propagation delay.
 * 
 * @param measured_range Measured range (m)
 * @param frequency Operating frequency (Hz)
 * @param temperature Temperature (K)
 * @param humidity Relative humidity (0.0 to 1.0)
 * @param pressure Atmospheric pressure (Pa)
 * @return Corrected range (m)
 */
double compensate_atmospheric_delay(double measured_range, double frequency,
                                   double temperature, double humidity, double pressure);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Validate range parameters
 * 
 * @param params Range parameters to validate
 * @return true if parameters are valid, false otherwise
 */
bool validate_range_parameters(const RangeParams* params);

/**
 * @brief Convert range bin to range value
 * 
 * @param bin Range bin index
 * @param params Range parameters
 * @return Range value (m)
 */
double range_bin_to_value(uint32_t bin, const RangeParams* params);

/**
 * @brief Convert range value to bin index
 * 
 * @param range Range value (m)
 * @param params Range parameters
 * @return Range bin index
 */
uint32_t range_value_to_bin(double range, const RangeParams* params);

/**
 * @brief Calculate range resolution
 * 
 * Computes theoretical range resolution based on signal bandwidth.
 * 
 * @param bandwidth Signal bandwidth (Hz)
 * @param is_two_way Flag indicating two-way propagation
 * @return Range resolution (m)
 */
double calculate_range_resolution(double bandwidth, bool is_two_way);

// ============================================================================
// CLEANUP FUNCTIONS
// ============================================================================

/**
 * @brief Cleanup range measurement results
 * 
 * @param measurement Range measurement structure to cleanup
 */
void range_measurement_cleanup(RangeMeasurement* measurement);

#ifdef __cplusplus
}
#endif

#endif // RANGE_DETECTOR_H
