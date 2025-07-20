/**
 * @file doppler_analyzer.h
 * @brief Doppler shift analysis and velocity estimation functions
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This header provides functions for Doppler shift analysis, velocity estimation,
 * and moving target indication (MTI) processing for radar/LiDAR systems.
 */

#ifndef DOPPLER_ANALYZER_H
#define DOPPLER_ANALYZER_H

#include "signal_types.h"
#include "radar_config.h"
#include "fft_processor.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// DOPPLER ANALYSIS STRUCTURES
// ============================================================================

/**
 * @brief Doppler processing parameters
 * 
 * Configuration parameters for Doppler analysis and velocity estimation.
 */
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

/**
 * @brief Doppler analysis results
 * 
 * Contains the results of Doppler processing including velocity estimates
 * and confidence metrics.
 */
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

/**
 * @brief Moving Target Indication (MTI) filter
 * 
 * Implements MTI filtering for clutter suppression in radar systems.
 */
typedef struct {
    double* filter_coeffs;          /**< MTI filter coefficients */
    ComplexSample* delay_line;      /**< Filter delay line */
    uint32_t filter_order;          /**< Filter order */
    uint32_t delay_length;          /**< Delay line length */
    double improvement_factor_db;   /**< MTI improvement factor (dB) */
    bool is_initialized;            /**< Initialization status */
} MTIFilter;

/**
 * @brief Range-Doppler processor
 * 
 * Processes multiple range gates simultaneously for Doppler analysis.
 */
typedef struct {
    FFTProcessor* range_fft;        /**< Range FFT processor */
    FFTProcessor* doppler_fft;      /**< Doppler FFT processor */
    uint32_t num_range_gates;       /**< Number of range gates */
    uint32_t num_pulses;            /**< Number of pulses to process */
    ComplexSample** pulse_data;     /**< 2D array of pulse data */
    bool is_initialized;            /**< Initialization status */
} RangeDopplerProcessor;

// ============================================================================
// DOPPLER ANALYSIS FUNCTIONS
// ============================================================================

/**
 * @brief Initialize Doppler analysis parameters
 * 
 * Sets up Doppler processing parameters based on radar system configuration.
 * 
 * @param params Pointer to Doppler parameters structure
 * @param carrier_freq Radar carrier frequency (Hz)
 * @param prf Pulse repetition frequency (Hz)
 * @param num_pulses Number of pulses for coherent processing
 * @return true if initialization successful, false otherwise
 */
bool doppler_params_init(DopplerParams* params, double carrier_freq, double prf, uint32_t num_pulses);

/**
 * @brief Perform Doppler analysis on pulse train
 * 
 * Analyzes a coherent pulse train to extract Doppler information
 * and estimate target velocities.
 * 
 * @param pulse_train Array of received pulse signals
 * @param num_pulses Number of pulses in the train
 * @param params Doppler processing parameters
 * @param analysis Output Doppler analysis results
 * @return true if analysis successful, false otherwise
 */
bool analyze_doppler(const SignalBuffer* pulse_train, uint32_t num_pulses, 
                     const DopplerParams* params, DopplerAnalysis* analysis);

/**
 * @brief Calculate Doppler frequency from velocity
 * 
 * Converts radial velocity to Doppler frequency shift using the radar equation.
 * 
 * @param velocity Radial velocity (m/s, positive = approaching)
 * @param carrier_frequency Radar carrier frequency (Hz)
 * @return Doppler frequency shift (Hz)
 */
double velocity_to_doppler(double velocity, double carrier_frequency);

/**
 * @brief Calculate velocity from Doppler frequency
 * 
 * Converts Doppler frequency shift to radial velocity.
 * 
 * @param doppler_freq Doppler frequency shift (Hz)
 * @param carrier_frequency Radar carrier frequency (Hz)
 * @return Radial velocity (m/s, positive = approaching)
 */
double doppler_to_velocity(double doppler_freq, double carrier_frequency);

/**
 * @brief Estimate maximum unambiguous velocity
 * 
 * Calculates the maximum unambiguous velocity based on PRF and carrier frequency.
 * 
 * @param prf Pulse repetition frequency (Hz)
 * @param carrier_frequency Radar carrier frequency (Hz)
 * @return Maximum unambiguous velocity (m/s)
 */
double calculate_max_unambiguous_velocity(double prf, double carrier_frequency);

// ============================================================================
// MOVING TARGET INDICATION (MTI)
// ============================================================================

/**
 * @brief Initialize MTI filter
 * 
 * Creates and initializes an MTI filter for clutter suppression.
 * 
 * @param mti Pointer to MTI filter structure
 * @param filter_order Order of the MTI filter
 * @param prf Pulse repetition frequency (Hz)
 * @return true if initialization successful, false otherwise
 */
bool mti_filter_init(MTIFilter* mti, uint32_t filter_order, double prf);

/**
 * @brief Cleanup MTI filter resources
 * 
 * @param mti Pointer to MTI filter structure
 */
void mti_filter_cleanup(MTIFilter* mti);

/**
 * @brief Apply MTI filtering to pulse train
 * 
 * Applies MTI filtering to suppress stationary and slow-moving clutter.
 * 
 * @param mti Pointer to initialized MTI filter
 * @param input_pulses Input pulse train
 * @param output_pulses Output filtered pulse train
 * @param num_pulses Number of pulses to process
 * @return true if filtering successful, false otherwise
 */
bool apply_mti_filter(MTIFilter* mti, const SignalBuffer* input_pulses, 
                      SignalBuffer* output_pulses, uint32_t num_pulses);

/**
 * @brief Calculate MTI improvement factor
 * 
 * Computes the clutter suppression improvement factor for the MTI filter.
 * 
 * @param mti Pointer to MTI filter
 * @param clutter_velocity Clutter velocity (m/s)
 * @return Improvement factor (dB)
 */
double calculate_mti_improvement(const MTIFilter* mti, double clutter_velocity);

// ============================================================================
// RANGE-DOPPLER PROCESSING
// ============================================================================

/**
 * @brief Initialize Range-Doppler processor
 * 
 * Sets up processing for simultaneous range and Doppler analysis.
 * 
 * @param processor Pointer to Range-Doppler processor
 * @param num_range_gates Number of range gates
 * @param num_pulses Number of pulses per CPI
 * @param range_fft_size FFT size for range processing
 * @param doppler_fft_size FFT size for Doppler processing
 * @return true if initialization successful, false otherwise
 */
bool range_doppler_init(RangeDopplerProcessor* processor, uint32_t num_range_gates,
                        uint32_t num_pulses, uint32_t range_fft_size, uint32_t doppler_fft_size);

/**
 * @brief Cleanup Range-Doppler processor
 * 
 * @param processor Pointer to Range-Doppler processor
 */
void range_doppler_cleanup(RangeDopplerProcessor* processor);

/**
 * @brief Generate Range-Doppler map
 * 
 * Creates a 2D Range-Doppler map from multiple pulse returns.
 * 
 * @param processor Pointer to initialized Range-Doppler processor
 * @param pulse_data 2D array of pulse data [pulse][range_sample]
 * @param rd_map Output Range-Doppler map structure
 * @return true if processing successful, false otherwise
 */
bool generate_range_doppler_map(RangeDopplerProcessor* processor, 
                               const ComplexSample** pulse_data, RangeDopplerMap* rd_map);

/**
 * @brief Detect targets in Range-Doppler map
 * 
 * Applies CFAR detection to identify targets in the Range-Doppler map.
 * 
 * @param rd_map Input Range-Doppler map
 * @param threshold_db Detection threshold above noise (dB)
 * @param detections Output array of detection results
 * @param max_detections Maximum number of detections to return
 * @return Number of detections found
 */
uint32_t detect_targets_rd_map(const RangeDopplerMap* rd_map, double threshold_db,
                               DetectionResult* detections, uint32_t max_detections);

// ============================================================================
// ADVANCED DOPPLER PROCESSING
// ============================================================================

/**
 * @brief Perform pulse-Doppler processing
 * 
 * Advanced processing combining pulse compression and Doppler analysis.
 * 
 * @param received_pulses Array of received pulse signals
 * @param reference_pulse Reference pulse for matched filtering
 * @param num_pulses Number of pulses to process
 * @param params Doppler processing parameters
 * @param analysis Output analysis results
 * @return true if processing successful, false otherwise
 */
bool pulse_doppler_processing(const SignalBuffer* received_pulses, const SignalBuffer* reference_pulse,
                              uint32_t num_pulses, const DopplerParams* params, 
                              DopplerAnalysis* analysis);

/**
 * @brief Resolve velocity ambiguity
 * 
 * Attempts to resolve velocity ambiguity using multiple PRF measurements
 * or other techniques.
 * 
 * @param measurements Array of velocity measurements at different PRFs
 * @param prfs Array of corresponding PRF values
 * @param num_measurements Number of measurements
 * @param carrier_freq Carrier frequency
 * @param resolved_velocity Output resolved velocity
 * @return true if ambiguity resolved successfully, false otherwise
 */
bool resolve_velocity_ambiguity(const double* measurements, const double* prfs,
                                uint32_t num_measurements, double carrier_freq,
                                double* resolved_velocity);

/**
 * @brief Estimate target acceleration
 * 
 * Estimates target acceleration from velocity measurements over time.
 * 
 * @param velocity_history Array of velocity measurements
 * @param time_stamps Array of corresponding time stamps
 * @param num_measurements Number of measurements
 * @param acceleration Output acceleration estimate (m/s²)
 * @param confidence Output confidence level
 * @return true if estimation successful, false otherwise
 */
bool estimate_target_acceleration(const double* velocity_history, const double* time_stamps,
                                 uint32_t num_measurements, double* acceleration, double* confidence);

// ============================================================================
// SPECTRAL ANALYSIS FOR DOPPLER
// ============================================================================

/**
 * @brief Compute Doppler power spectral density
 * 
 * Calculates PSD of Doppler spectrum for detailed velocity analysis.
 * 
 * @param pulse_train Input pulse train
 * @param num_pulses Number of pulses
 * @param params Doppler parameters
 * @param psd_output Output power spectral density
 * @param velocity_bins Output velocity bin values
 * @param spectrum_length Output length of spectrum arrays
 * @return true if computation successful, false otherwise
 */
bool compute_doppler_psd(const SignalBuffer* pulse_train, uint32_t num_pulses,
                         const DopplerParams* params, double* psd_output,
                         double* velocity_bins, uint32_t* spectrum_length);

/**
 * @brief Find multiple velocity peaks in Doppler spectrum
 * 
 * Identifies multiple targets or components in the Doppler spectrum.
 * 
 * @param doppler_spectrum Input Doppler magnitude spectrum
 * @param velocity_bins Velocity bin values
 * @param spectrum_length Length of spectrum arrays
 * @param threshold_db Detection threshold (dB)
 * @param peak_velocities Output array of peak velocities
 * @param peak_magnitudes Output array of peak magnitudes
 * @param max_peaks Maximum number of peaks to find
 * @return Number of peaks found
 */
uint32_t find_doppler_peaks(const double* doppler_spectrum, const double* velocity_bins,
                            uint32_t spectrum_length, double threshold_db,
                            double* peak_velocities, double* peak_magnitudes, uint32_t max_peaks);

/**
 * @brief Estimate Doppler centroid and spread
 * 
 * Calculates the centroid and spread of the Doppler spectrum for
 * distributed targets or clutter analysis.
 * 
 * @param doppler_spectrum Input Doppler spectrum
 * @param velocity_bins Velocity bin values
 * @param spectrum_length Length of spectrum
 * @param centroid Output centroid velocity (m/s)
 * @param spread Output velocity spread (m/s)
 * @return true if calculation successful, false otherwise
 */
bool estimate_doppler_centroid_spread(const double* doppler_spectrum, const double* velocity_bins,
                                     uint32_t spectrum_length, double* centroid, double* spread);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Validate Doppler parameters
 * 
 * @param params Doppler parameters to validate
 * @return true if parameters are valid, false otherwise
 */
bool validate_doppler_parameters(const DopplerParams* params);

/**
 * @brief Convert velocity bin to velocity value
 * 
 * @param bin Velocity bin index
 * @param params Doppler parameters
 * @return Velocity value (m/s)
 */
double velocity_bin_to_value(uint32_t bin, const DopplerParams* params);

/**
 * @brief Convert velocity value to bin index
 * 
 * @param velocity Velocity value (m/s)
 * @param params Doppler parameters
 * @return Velocity bin index
 */
uint32_t velocity_value_to_bin(double velocity, const DopplerParams* params);

/**
 * @brief Calculate Doppler processing gain
 * 
 * Computes the processing gain achieved through coherent integration.
 * 
 * @param num_pulses Number of coherently integrated pulses
 * @return Processing gain (dB)
 */
double calculate_doppler_processing_gain(uint32_t num_pulses);

// ============================================================================
// CLEANUP FUNCTIONS
// ============================================================================

/**
 * @brief Cleanup Doppler analysis results
 * 
 * @param analysis Doppler analysis structure to cleanup
 */
void doppler_analysis_cleanup(DopplerAnalysis* analysis);

/**
 * @brief Cleanup Range-Doppler map
 * 
 * @param rd_map Range-Doppler map structure to cleanup
 */
void range_doppler_map_cleanup(RangeDopplerMap* rd_map);

#ifdef __cplusplus
}
#endif

#endif // DOPPLER_ANALYZER_H
