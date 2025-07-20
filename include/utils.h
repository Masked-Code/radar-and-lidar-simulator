/**
 * @file utils.h
 * @brief Utility functions and helper routines for radar/LiDAR simulation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This header provides utility functions, mathematical helpers, file I/O,
 * memory management, and other common routines used throughout the simulation.
 */

#ifndef UTILS_H
#define UTILS_H

#include "signal_types.h"
#include "radar_config.h"
#include <stdio.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// MEMORY MANAGEMENT UTILITIES
// ============================================================================

/**
 * @brief Allocate signal buffer with specified length
 * 
 * @param buffer Pointer to signal buffer structure
 * @param length Number of samples to allocate
 * @param sample_rate Sampling rate in Hz
 * @return true if allocation successful, false otherwise
 */
bool allocate_signal_buffer(SignalBuffer* buffer, size_t length, double sample_rate);

/**
 * @brief Free signal buffer memory
 * 
 * @param buffer Pointer to signal buffer structure
 */
void free_signal_buffer(SignalBuffer* buffer);

/**
 * @brief Copy signal buffer data
 * 
 * @param src Source signal buffer
 * @param dst Destination signal buffer
 * @return true if copy successful, false otherwise
 */
bool copy_signal_buffer(const SignalBuffer* src, SignalBuffer* dst);

/**
 * @brief Resize signal buffer
 * 
 * @param buffer Pointer to signal buffer
 * @param new_length New buffer length
 * @return true if resize successful, false otherwise
 */
bool resize_signal_buffer(SignalBuffer* buffer, size_t new_length);

/**
 * @brief Allocate 2D double array
 * 
 * @param rows Number of rows
 * @param cols Number of columns
 * @return Pointer to allocated 2D array, NULL if failed
 */
double** allocate_2d_double_array(size_t rows, size_t cols);

/**
 * @brief Free 2D double array
 * 
 * @param array Pointer to 2D array
 * @param rows Number of rows
 */
void free_2d_double_array(double** array, size_t rows);

/**
 * @brief Allocate 3D double array
 * 
 * @param depth Number of depth layers
 * @param rows Number of rows
 * @param cols Number of columns
 * @return Pointer to allocated 3D array, NULL if failed
 */
double*** allocate_3d_double_array(size_t depth, size_t rows, size_t cols);

/**
 * @brief Free 3D double array
 * 
 * @param array Pointer to 3D array
 * @param depth Number of depth layers
 * @param rows Number of rows
 */
void free_3d_double_array(double*** array, size_t depth, size_t rows);

// ============================================================================
// MATHEMATICAL UTILITIES
// ============================================================================

/**
 * @brief Generate uniformly distributed random numbers
 * 
 * @param min Minimum value
 * @param max Maximum value
 * @return Random value between min and max
 */
double random_uniform(double min, double max);

/**
 * @brief Generate normally distributed random numbers using Box-Muller transform
 * 
 * @param mean Mean of distribution
 * @param std_dev Standard deviation
 * @return Random value from normal distribution
 */
double random_normal(double mean, double std_dev);

/**
 * @brief Generate complex white Gaussian noise
 * 
 * @param noise_power Power of noise (linear scale)
 * @return Complex noise sample
 */
ComplexSample generate_complex_noise(double noise_power);

/**
 * @brief Calculate mean of double array
 * 
 * @param data Input data array
 * @param length Array length
 * @return Mean value
 */
double calculate_mean(const double* data, size_t length);

/**
 * @brief Calculate standard deviation of double array
 * 
 * @param data Input data array
 * @param length Array length
 * @return Standard deviation
 */
double calculate_std_dev(const double* data, size_t length);

/**
 * @brief Calculate median of double array
 * 
 * @param data Input data array
 * @param length Array length
 * @return Median value
 */
double calculate_median(const double* data, size_t length);

/**
 * @brief Find maximum value and index in array
 * 
 * @param data Input data array
 * @param length Array length
 * @param max_index Output index of maximum value
 * @return Maximum value
 */
double find_array_max(const double* data, size_t length, size_t* max_index);

/**
 * @brief Find minimum value and index in array
 * 
 * @param data Input data array
 * @param length Array length
 * @param min_index Output index of minimum value
 * @return Minimum value
 */
double find_array_min(const double* data, size_t length, size_t* min_index);

/**
 * @brief Linear interpolation between two points
 * 
 * @param x Input value
 * @param x1 First x coordinate
 * @param y1 First y coordinate
 * @param x2 Second x coordinate
 * @param y2 Second y coordinate
 * @return Interpolated y value
 */
double linear_interpolate(double x, double x1, double y1, double x2, double y2);

/**
 * @brief Bilinear interpolation for 2D data
 * 
 * @param x X coordinate for interpolation
 * @param y Y coordinate for interpolation
 * @param x1 First x grid point
 * @param x2 Second x grid point
 * @param y1 First y grid point
 * @param y2 Second y grid point
 * @param q11 Data value at (x1, y1)
 * @param q12 Data value at (x1, y2)
 * @param q21 Data value at (x2, y1)
 * @param q22 Data value at (x2, y2)
 * @return Interpolated value
 */
double bilinear_interpolate(double x, double y, double x1, double x2, double y1, double y2,
                           double q11, double q12, double q21, double q22);

/**
 * @brief Calculate root mean square of array
 * 
 * @param data Input data array
 * @param length Array length
 * @return RMS value
 */
double calculate_rms(const double* data, size_t length);

/**
 * @brief Calculate signal power (average of squared magnitudes)
 * 
 * @param signal Input signal buffer
 * @return Signal power (linear scale)
 */
double calculate_signal_power(const SignalBuffer* signal);

// ============================================================================
// COMPLEX NUMBER UTILITIES
// ============================================================================

/**
 * @brief Calculate magnitude of complex sample
 * 
 * @param sample Complex sample
 * @return Magnitude
 */
double complex_magnitude(const ComplexSample sample);

/**
 * @brief Calculate phase of complex sample
 * 
 * @param sample Complex sample
 * @return Phase in radians
 */
double complex_phase(const ComplexSample sample);

/**
 * @brief Complex conjugate
 * 
 * @param sample Complex sample
 * @return Complex conjugate
 */
ComplexSample complex_conjugate(const ComplexSample sample);

/**
 * @brief Add two complex samples
 * 
 * @param a First complex sample
 * @param b Second complex sample
 * @return Sum of complex samples
 */
ComplexSample complex_add(const ComplexSample a, const ComplexSample b);

/**
 * @brief Subtract two complex samples
 * 
 * @param a First complex sample
 * @param b Second complex sample
 * @return Difference (a - b)
 */
ComplexSample complex_subtract(const ComplexSample a, const ComplexSample b);

/**
 * @brief Multiply two complex samples
 * 
 * @param a First complex sample
 * @param b Second complex sample
 * @return Product of complex samples
 */
ComplexSample complex_multiply(const ComplexSample a, const ComplexSample b);

/**
 * @brief Divide two complex samples
 * 
 * @param a Numerator complex sample
 * @param b Denominator complex sample
 * @return Quotient (a / b)
 */
ComplexSample complex_divide(const ComplexSample a, const ComplexSample b);

/**
 * @brief Scale complex sample by real factor
 * 
 * @param sample Complex sample
 * @param scale Real scaling factor
 * @return Scaled complex sample
 */
ComplexSample complex_scale(const ComplexSample sample, double scale);

/**
 * @brief Convert complex sample to polar form
 * 
 * @param sample Complex sample
 * @param magnitude Output magnitude
 * @param phase Output phase in radians
 */
void complex_to_polar(const ComplexSample sample, double* magnitude, double* phase);

/**
 * @brief Convert polar form to complex sample
 * 
 * @param magnitude Magnitude
 * @param phase Phase in radians
 * @return Complex sample
 */
ComplexSample polar_to_complex(double magnitude, double phase);

// ============================================================================
// FILE I/O UTILITIES
// ============================================================================

/**
 * @brief Save signal buffer to binary file
 * 
 * @param signal Signal buffer to save
 * @param filename Output filename
 * @return true if save successful, false otherwise
 */
bool save_signal_to_file(const SignalBuffer* signal, const char* filename);

/**
 * @brief Load signal buffer from binary file
 * 
 * @param signal Signal buffer to load into
 * @param filename Input filename
 * @return true if load successful, false otherwise
 */
bool load_signal_from_file(SignalBuffer* signal, const char* filename);

/**
 * @brief Export signal to CSV format
 * 
 * @param signal Signal buffer to export
 * @param filename Output CSV filename
 * @param include_phase Flag to include phase information
 * @return true if export successful, false otherwise
 */
bool export_signal_to_csv(const SignalBuffer* signal, const char* filename, bool include_phase);

/**
 * @brief Save detection results to CSV file
 * 
 * @param detections Array of detection results
 * @param num_detections Number of detections
 * @param filename Output filename
 * @return true if save successful, false otherwise
 */
bool save_detections_to_csv(const DetectionResult* detections, uint32_t num_detections, 
                            const char* filename);

/**
 * @brief Save range-Doppler map to file
 * 
 * @param rd_map Range-Doppler map structure
 * @param filename Output filename
 * @return true if save successful, false otherwise
 */
bool save_range_doppler_map(const RangeDopplerMap* rd_map, const char* filename);

/**
 * @brief Create configuration file from system parameters
 * 
 * @param config System configuration structure
 * @param filename Output configuration filename
 * @return true if creation successful, false otherwise
 */
bool create_config_file(const SystemConfig* config, const char* filename);

/**
 * @brief Load configuration from file
 * 
 * @param config System configuration structure to populate
 * @param filename Input configuration filename
 * @return true if load successful, false otherwise
 */
bool load_config_file(SystemConfig* config, const char* filename);

// ============================================================================
// STRING UTILITIES
// ============================================================================

/**
 * @brief Safe string copy with length limit
 * 
 * @param dest Destination buffer
 * @param src Source string
 * @param dest_size Size of destination buffer
 * @return true if copy successful, false if truncated
 */
bool safe_string_copy(char* dest, const char* src, size_t dest_size);

/**
 * @brief Format timestamp string
 * 
 * @param buffer Output buffer
 * @param buffer_size Size of output buffer
 * @param timestamp Timestamp to format
 * @return Formatted timestamp string
 */
char* format_timestamp(char* buffer, size_t buffer_size, time_t timestamp);

/**
 * @brief Parse parameter from string
 * 
 * @param str Input string
 * @param param_name Parameter name to find
 * @param value Output value
 * @return true if parameter found and parsed, false otherwise
 */
bool parse_parameter_double(const char* str, const char* param_name, double* value);

/**
 * @brief Parse integer parameter from string
 * 
 * @param str Input string
 * @param param_name Parameter name to find
 * @param value Output value
 * @return true if parameter found and parsed, false otherwise
 */
bool parse_parameter_int(const char* str, const char* param_name, int* value);

// ============================================================================
// TIMING AND PROFILING UTILITIES
// ============================================================================

/**
 * @brief High-resolution timer structure
 */
typedef struct {
    struct timespec start_time;     /**< Start time */
    struct timespec end_time;       /**< End time */
    bool is_running;                /**< Timer state */
} HighResTimer;

/**
 * @brief Start high-resolution timer
 * 
 * @param timer Timer structure
 * @return true if timer started successfully, false otherwise
 */
bool timer_start(HighResTimer* timer);

/**
 * @brief Stop high-resolution timer
 * 
 * @param timer Timer structure
 * @return true if timer stopped successfully, false otherwise
 */
bool timer_stop(HighResTimer* timer);

/**
 * @brief Get elapsed time in seconds
 * 
 * @param timer Timer structure
 * @return Elapsed time in seconds
 */
double timer_get_elapsed_seconds(const HighResTimer* timer);

/**
 * @brief Get elapsed time in milliseconds
 * 
 * @param timer Timer structure
 * @return Elapsed time in milliseconds
 */
double timer_get_elapsed_ms(const HighResTimer* timer);

/**
 * @brief Get elapsed time in microseconds
 * 
 * @param timer Timer structure
 * @return Elapsed time in microseconds
 */
double timer_get_elapsed_us(const HighResTimer* timer);

// ============================================================================
// LOGGING UTILITIES
// ============================================================================

/**
 * @brief Log levels enumeration
 */
typedef enum {
    LOG_LEVEL_DEBUG = 0,    /**< Debug messages */
    LOG_LEVEL_INFO,         /**< Informational messages */
    LOG_LEVEL_WARNING,      /**< Warning messages */
    LOG_LEVEL_ERROR,        /**< Error messages */
    LOG_LEVEL_FATAL         /**< Fatal error messages */
} LogLevel;

/**
 * @brief Initialize logging system
 * 
 * @param log_filename Log file name (NULL for stdout)
 * @param min_level Minimum log level to record
 * @return true if initialization successful, false otherwise
 */
bool log_init(const char* log_filename, LogLevel min_level);

/**
 * @brief Cleanup logging system
 */
void log_cleanup(void);

/**
 * @brief Write log message
 * 
 * @param level Log level
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void log_message(LogLevel level, const char* format, ...);

/**
 * @brief Write debug message
 * 
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void log_debug(const char* format, ...);

/**
 * @brief Write info message
 * 
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void log_info(const char* format, ...);

/**
 * @brief Write warning message
 * 
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void log_warning(const char* format, ...);

/**
 * @brief Write error message
 * 
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void log_error(const char* format, ...);

// ============================================================================
// SIGNAL GENERATION UTILITIES
// ============================================================================

/**
 * @brief Generate sinusoidal signal
 * 
 * @param signal Output signal buffer
 * @param frequency Signal frequency (Hz)
 * @param amplitude Signal amplitude
 * @param phase Initial phase (radians)
 * @return true if generation successful, false otherwise
 */
bool generate_sine_wave(SignalBuffer* signal, double frequency, double amplitude, double phase);

/**
 * @brief Generate complex exponential signal
 * 
 * @param signal Output signal buffer
 * @param frequency Signal frequency (Hz)
 * @param amplitude Signal amplitude
 * @param phase Initial phase (radians)
 * @return true if generation successful, false otherwise
 */
bool generate_complex_exponential(SignalBuffer* signal, double frequency, double amplitude, double phase);


/**
 * @brief Add white Gaussian noise to signal
 * 
 * @param signal Signal buffer to add noise to
 * @param noise_power Noise power (linear scale)
 * @return true if noise addition successful, false otherwise
 */
bool add_awgn_to_signal(SignalBuffer* signal, double noise_power);

// ============================================================================
// VALIDATION UTILITIES
// ============================================================================

/**
 * @brief Validate signal buffer integrity
 * 
 * @param signal Signal buffer to validate
 * @return true if valid, false otherwise
 */
bool validate_signal_buffer(const SignalBuffer* signal);

/**
 * @brief Check for NaN or infinite values in array
 * 
 * @param data Array to check
 * @param length Array length
 * @return true if array contains only valid values, false otherwise
 */
bool check_array_validity(const double* data, size_t length);

/**
 * @brief Sanitize signal buffer (replace invalid values)
 * 
 * @param signal Signal buffer to sanitize
 * @param replacement_value Value to replace invalid samples with
 * @return Number of samples that were replaced
 */
size_t sanitize_signal_buffer(SignalBuffer* signal, ComplexSample replacement_value);

// ============================================================================
// PERFORMANCE UTILITIES
// ============================================================================

/**
 * @brief Get system memory usage in MB
 * 
 * @return Current memory usage in MB, -1 if unable to determine
 */
double get_memory_usage_mb(void);

/**
 * @brief Get number of available CPU cores
 * 
 * @return Number of CPU cores, 1 if unable to determine
 */
int get_cpu_core_count(void);

/**
 * @brief Sleep for specified number of milliseconds
 * 
 * @param milliseconds Number of milliseconds to sleep
 */
void sleep_ms(int milliseconds);

// ============================================================================
// CLEANUP UTILITIES
// ============================================================================

/**
 * @brief Register cleanup function to be called at exit
 * 
 * @param cleanup_func Cleanup function pointer
 * @return true if registration successful, false otherwise
 */
bool register_cleanup_function(void (*cleanup_func)(void));

/**
 * @brief Call all registered cleanup functions
 */
void call_cleanup_functions(void);

#ifdef __cplusplus
}
#endif

#endif // UTILS_H
