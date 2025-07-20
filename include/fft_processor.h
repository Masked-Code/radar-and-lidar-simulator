/**
 * @file fft_processor.h
 * @brief FFT processing functions and frequency domain analysis
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This header provides functions for Fast Fourier Transform operations,
 * spectral analysis, and frequency domain signal processing for radar/LiDAR
 * applications.
 */

#ifndef FFT_PROCESSOR_H
#define FFT_PROCESSOR_H

#include "signal_types.h"
#include "radar_config.h"
#include <fftw3.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// FFT PROCESSOR STRUCTURES
// ============================================================================

/**
 * @brief FFT plan and workspace structure
 * 
 * Contains FFTW plans and workspace memory for efficient FFT computation.
 * This structure maintains state between FFT calls for optimal performance.
 */
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

/**
 * @brief Spectral analysis results structure
 * 
 * Contains the results of spectral analysis operations including
 * power spectral density and peak detection results.
 */
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

// ============================================================================
// FFT PROCESSOR INITIALIZATION AND CLEANUP
// ============================================================================

/**
 * @brief Initialize FFT processor with specified parameters
 * 
 * Creates FFTW plans and allocates memory for FFT operations. Must be called
 * before using any other FFT processing functions.
 * 
 * @param processor Pointer to FFT processor structure
 * @param fft_size Size of FFT (must be power of 2)
 * @param window_type Type of window function to apply
 * @return true if initialization successful, false otherwise
 */
bool fft_processor_init(FFTProcessor* processor, uint32_t fft_size, WindowType window_type);

/**
 * @brief Cleanup and deallocate FFT processor resources
 * 
 * Destroys FFTW plans and frees allocated memory. Should be called when
 * FFT processor is no longer needed.
 * 
 * @param processor Pointer to FFT processor structure
 */
void fft_processor_cleanup(FFTProcessor* processor);

/**
 * @brief Reconfigure existing FFT processor with new parameters
 * 
 * @param processor Pointer to FFT processor structure
 * @param new_size New FFT size (must be power of 2)
 * @param new_window New window function type
 * @return true if reconfiguration successful, false otherwise
 */
bool fft_processor_reconfigure(FFTProcessor* processor, uint32_t new_size, WindowType new_window);

// ============================================================================
// CORE FFT OPERATIONS
// ============================================================================

/**
 * @brief Compute forward FFT of signal buffer
 * 
 * Applies window function and computes FFT of input signal. Result is stored
 * in the output signal buffer with frequency domain representation.
 * 
 * @param processor Pointer to initialized FFT processor
 * @param input Input signal buffer (time domain)
 * @param output Output signal buffer (frequency domain)
 * @return true if FFT computation successful, false otherwise
 */
bool fft_forward(FFTProcessor* processor, const SignalBuffer* input, SignalBuffer* output);

/**
 * @brief Compute inverse FFT of frequency domain signal
 * 
 * Computes inverse FFT and applies appropriate scaling to recover time domain
 * signal from frequency domain representation.
 * 
 * @param processor Pointer to initialized FFT processor
 * @param input Input signal buffer (frequency domain)
 * @param output Output signal buffer (time domain)
 * @return true if IFFT computation successful, false otherwise
 */
bool fft_inverse(FFTProcessor* processor, const SignalBuffer* input, SignalBuffer* output);

/**
 * @brief Compute real-valued FFT for efficiency when input is real
 * 
 * @param processor Pointer to initialized FFT processor
 * @param input Real-valued input signal
 * @param input_length Length of input signal
 * @param output Complex-valued frequency domain output
 * @return true if computation successful, false otherwise
 */
bool fft_real_forward(FFTProcessor* processor, const double* input, uint32_t input_length, 
                      ComplexSample* output);

// ============================================================================
// SPECTRAL ANALYSIS FUNCTIONS
// ============================================================================

/**
 * @brief Perform comprehensive spectral analysis of signal
 * 
 * Computes magnitude spectrum, phase spectrum, power spectral density,
 * and identifies spectral peaks for detailed signal analysis.
 * 
 * @param processor Pointer to initialized FFT processor
 * @param signal Input signal buffer
 * @param analysis Output spectral analysis results
 * @return true if analysis successful, false otherwise
 */
bool spectral_analysis(FFTProcessor* processor, const SignalBuffer* signal, SpectralAnalysis* analysis);

/**
 * @brief Compute power spectral density using Welch's method
 * 
 * Uses overlapping windowed segments for robust PSD estimation with
 * reduced variance compared to single FFT.
 * 
 * @param processor Pointer to initialized FFT processor
 * @param signal Input signal buffer
 * @param overlap_ratio Overlap ratio between segments (0.0 to 1.0)
 * @param psd_output Power spectral density array
 * @param freq_output Frequency bin array
 * @param output_length Length of output arrays
 * @return true if PSD computation successful, false otherwise
 */
bool psd_welch_method(FFTProcessor* processor, const SignalBuffer* signal, double overlap_ratio,
                      double* psd_output, double* freq_output, uint32_t* output_length);

/**
 * @brief Find spectral peaks above threshold
 * 
 * Identifies local maxima in the spectrum that exceed the specified threshold
 * relative to the noise floor.
 * 
 * @param spectrum Magnitude spectrum array
 * @param spectrum_length Length of spectrum array
 * @param threshold_db Threshold above noise floor (dB)
 * @param peak_frequencies Output array for peak frequencies
 * @param peak_magnitudes Output array for peak magnitudes
 * @param max_peaks Maximum number of peaks to find
 * @return Number of peaks found
 */
uint32_t find_spectral_peaks(const double* spectrum, uint32_t spectrum_length, double threshold_db,
                             double* peak_frequencies, double* peak_magnitudes, uint32_t max_peaks);

// ============================================================================
// WINDOW FUNCTION GENERATION
// ============================================================================

/**
 * @brief Generate window function coefficients
 * 
 * Creates window function coefficients for the specified window type and length.
 * Window functions reduce spectral leakage in FFT analysis.
 * 
 * @param window_type Type of window function
 * @param window_length Length of window function
 * @param coefficients Output array for window coefficients
 * @param beta Kaiser window beta parameter (ignored for other windows)
 * @return true if window generation successful, false otherwise
 */
bool generate_window(WindowType window_type, uint32_t window_length, double* coefficients, double beta);

/**
 * @brief Apply window function to signal buffer
 * 
 * Multiplies signal samples by window function coefficients to reduce
 * spectral leakage during FFT analysis.
 * 
 * @param signal Signal buffer to be windowed
 * @param window_coeffs Window function coefficients
 * @param window_length Length of window function
 * @return true if windowing successful, false otherwise
 */
bool apply_window(SignalBuffer* signal, const double* window_coeffs, uint32_t window_length);

/**
 * @brief Calculate window function correction factors
 * 
 * Computes correction factors needed to compensate for window function
 * amplitude and power loss.
 * 
 * @param window_coeffs Window function coefficients
 * @param window_length Length of window function
 * @param amplitude_correction Output amplitude correction factor
 * @param power_correction Output power correction factor
 * @return true if calculation successful, false otherwise
 */
bool calculate_window_correction(const double* window_coeffs, uint32_t window_length,
                                double* amplitude_correction, double* power_correction);

// ============================================================================
// ADVANCED FFT OPERATIONS
// ============================================================================

/**
 * @brief Compute spectrogram using Short-Time Fourier Transform (STFT)
 * 
 * Generates time-frequency representation of signal by computing FFT
 * over sliding windows.
 * 
 * @param processor Pointer to initialized FFT processor
 * @param signal Input signal buffer
 * @param window_size Size of analysis window
 * @param overlap_samples Number of overlapping samples
 * @param spectrogram Output 2D spectrogram matrix
 * @param time_bins Output time bin values
 * @param freq_bins Output frequency bin values
 * @param time_bins_count Number of time bins
 * @param freq_bins_count Number of frequency bins
 * @return true if spectrogram computation successful, false otherwise
 */
bool compute_spectrogram(FFTProcessor* processor, const SignalBuffer* signal, 
                         uint32_t window_size, uint32_t overlap_samples,
                         double** spectrogram, double* time_bins, double* freq_bins,
                         uint32_t* time_bins_count, uint32_t* freq_bins_count);

/**
 * @brief Cross-correlation using FFT (frequency domain)
 * 
 * Computes cross-correlation between two signals using FFT for efficiency.
 * Useful for matched filtering and time delay estimation.
 * 
 * @param processor Pointer to initialized FFT processor
 * @param signal1 First input signal
 * @param signal2 Second input signal
 * @param correlation Output cross-correlation result
 * @return true if cross-correlation successful, false otherwise
 */
bool fft_cross_correlation(FFTProcessor* processor, const SignalBuffer* signal1, 
                           const SignalBuffer* signal2, SignalBuffer* correlation);

/**
 * @brief Convolution using FFT (frequency domain)
 * 
 * Performs convolution of two signals using FFT for computational efficiency.
 * 
 * @param processor Pointer to initialized FFT processor
 * @param signal Input signal to be convolved
 * @param kernel Convolution kernel
 * @param result Output convolved signal
 * @return true if convolution successful, false otherwise
 */
bool fft_convolution(FFTProcessor* processor, const SignalBuffer* signal, 
                     const SignalBuffer* kernel, SignalBuffer* result);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Convert frequency to bin index
 * 
 * @param frequency Frequency in Hz
 * @param sample_rate Sample rate in Hz
 * @param fft_size FFT size
 * @return Corresponding bin index
 */
uint32_t frequency_to_bin(double frequency, double sample_rate, uint32_t fft_size);

/**
 * @brief Convert bin index to frequency
 * 
 * @param bin Bin index
 * @param sample_rate Sample rate in Hz
 * @param fft_size FFT size
 * @return Corresponding frequency in Hz
 */
double bin_to_frequency(uint32_t bin, double sample_rate, uint32_t fft_size);

/**
 * @brief Estimate noise floor from spectrum
 * 
 * @param spectrum Magnitude spectrum array
 * @param spectrum_length Length of spectrum
 * @param percentile Percentile for noise floor estimation (e.g., 0.1 for 10th percentile)
 * @return Estimated noise floor value
 */
double estimate_noise_floor(const double* spectrum, uint32_t spectrum_length, double percentile);

/**
 * @brief Check if FFT size is valid (power of 2)
 * 
 * @param size FFT size to validate
 * @return true if valid FFT size, false otherwise
 */
bool is_valid_fft_size(uint32_t size);

/**
 * @brief Get next larger valid FFT size
 * 
 * @param min_size Minimum required size
 * @return Next power of 2 greater than or equal to min_size
 */
uint32_t get_next_fft_size(uint32_t min_size);

// ============================================================================
// CLEANUP FUNCTIONS
// ============================================================================

/**
 * @brief Free spectral analysis results memory
 * 
 * @param analysis Spectral analysis structure to cleanup
 */
void spectral_analysis_cleanup(SpectralAnalysis* analysis);

/**
 * @brief Free spectrogram memory
 * 
 * @param spectrogram 2D spectrogram matrix
 * @param time_bins_count Number of time bins
 */
void spectrogram_cleanup(double** spectrogram, uint32_t time_bins_count);

#ifdef __cplusplus
}
#endif

#endif // FFT_PROCESSOR_H
