/**
 * @file utils.c
 * @brief Utility functions implementation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include "utils.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

// Global logging state
static FILE* log_file = NULL;
static LogLevel min_log_level = LOG_LEVEL_INFO;

// Memory management functions
bool allocate_signal_buffer(SignalBuffer* buffer, size_t length, double sample_rate) {
    if (!buffer || length == 0 || sample_rate <= 0) {
        return false;
    }

    buffer->samples = calloc(length, sizeof(ComplexSample));
    if (!buffer->samples) {
        return false;
    }

    buffer->length = length;
    buffer->sample_rate = sample_rate;
    buffer->duration = (double)length / sample_rate;
    buffer->is_allocated = true;
    buffer->center_frequency = 0.0;
    buffer->bandwidth = 0.0;
    buffer->timestamp = 0.0;
    buffer->waveform = WAVEFORM_CUSTOM;

    return true;
}

void free_signal_buffer(SignalBuffer* buffer) {
    if (!buffer) return;
    
    free(buffer->samples);
    memset(buffer, 0, sizeof(SignalBuffer));
}

bool copy_signal_buffer(const SignalBuffer* src, SignalBuffer* dst) {
    if (!src || !dst || !src->is_allocated) {
        return false;
    }

    if (!allocate_signal_buffer(dst, src->length, src->sample_rate)) {
        return false;
    }

    memcpy(dst->samples, src->samples, src->length * sizeof(ComplexSample));
    dst->center_frequency = src->center_frequency;
    dst->bandwidth = src->bandwidth;
    dst->waveform = src->waveform;
    dst->timestamp = src->timestamp;

    return true;
}

bool resize_signal_buffer(SignalBuffer* buffer, size_t new_length) {
    if (!buffer || new_length == 0) {
        return false;
    }

    ComplexSample* new_samples = realloc(buffer->samples, new_length * sizeof(ComplexSample));
    if (!new_samples) {
        return false;
    }

    buffer->samples = new_samples;
    
    // Zero out new samples if buffer grew
    if (new_length > buffer->length) {
        memset(&buffer->samples[buffer->length], 0, 
               (new_length - buffer->length) * sizeof(ComplexSample));
    }
    
    buffer->length = new_length;
    buffer->duration = (double)new_length / buffer->sample_rate;

    return true;
}

// Mathematical utilities
double random_uniform(double min, double max) {
    return min + (max - min) * ((double)rand() / RAND_MAX);
}

double random_normal(double mean, double std_dev) {
    static bool has_spare = false;
    static double spare;

    if (has_spare) {
        has_spare = false;
        return spare * std_dev + mean;
    }

    has_spare = true;
    double u = random_uniform(0.0, 1.0);
    double v = random_uniform(0.0, 1.0);
    
    double mag = std_dev * sqrt(-2.0 * log(u));
    spare = mag * cos(2.0 * M_PI * v);
    
    return mag * sin(2.0 * M_PI * v) + mean;
}

ComplexSample generate_complex_noise(double noise_power) {
    double std_dev = sqrt(noise_power / 2.0);
    ComplexSample noise;
    noise.real = random_normal(0.0, std_dev);
    noise.imag = random_normal(0.0, std_dev);
    return noise;
}

double calculate_mean(const double* data, size_t length) {
    if (!data || length == 0) return 0.0;
    
    double sum = 0.0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum / length;
}

double calculate_std_dev(const double* data, size_t length) {
    if (!data || length <= 1) return 0.0;
    
    double mean = calculate_mean(data, length);
    double variance = 0.0;
    
    for (size_t i = 0; i < length; i++) {
        double diff = data[i] - mean;
        variance += diff * diff;
    }
    
    return sqrt(variance / (length - 1));
}

double find_array_max(const double* data, size_t length, size_t* max_index) {
    if (!data || length == 0) {
        if (max_index) *max_index = 0;
        return 0.0;
    }
    
    double max_val = data[0];
    size_t max_idx = 0;
    
    for (size_t i = 1; i < length; i++) {
        if (data[i] > max_val) {
            max_val = data[i];
            max_idx = i;
        }
    }
    
    if (max_index) *max_index = max_idx;
    return max_val;
}

double calculate_signal_power(const SignalBuffer* signal) {
    if (!signal || !signal->samples || signal->length == 0) {
        return 0.0;
    }

    double power = 0.0;
    for (size_t i = 0; i < signal->length; i++) {
        double mag_sq = signal->samples[i].real * signal->samples[i].real +
                       signal->samples[i].imag * signal->samples[i].imag;
        power += mag_sq;
    }
    
    return power / signal->length;
}

// Complex number utilities
double complex_magnitude(const ComplexSample sample) {
    return sqrt(sample.real * sample.real + sample.imag * sample.imag);
}

double complex_phase(const ComplexSample sample) {
    return atan2(sample.imag, sample.real);
}

ComplexSample complex_add(const ComplexSample a, const ComplexSample b) {
    ComplexSample result;
    result.real = a.real + b.real;
    result.imag = a.imag + b.imag;
    return result;
}

ComplexSample complex_multiply(const ComplexSample a, const ComplexSample b) {
    ComplexSample result;
    result.real = a.real * b.real - a.imag * b.imag;
    result.imag = a.real * b.imag + a.imag * b.real;
    return result;
}

ComplexSample complex_scale(const ComplexSample sample, double scale) {
    ComplexSample result;
    result.real = sample.real * scale;
    result.imag = sample.imag * scale;
    return result;
}

// Timing functions
bool timer_start(HighResTimer* timer) {
    if (!timer) return false;
    
    timer->is_running = true;
#ifdef _WIN32
    // Windows implementation would go here
    timer->start_time.tv_sec = (long)time(NULL);
    timer->start_time.tv_nsec = 0;
#else
    clock_gettime(CLOCK_MONOTONIC, &timer->start_time);
#endif
    return true;
}

bool timer_stop(HighResTimer* timer) {
    if (!timer || !timer->is_running) return false;
    
#ifdef _WIN32
    timer->end_time.tv_sec = (long)time(NULL);
    timer->end_time.tv_nsec = 0;
#else
    clock_gettime(CLOCK_MONOTONIC, &timer->end_time);
#endif
    timer->is_running = false;
    return true;
}

double timer_get_elapsed_seconds(const HighResTimer* timer) {
    if (!timer) return 0.0;
    
    struct timespec end_time = timer->is_running ? timer->start_time : timer->end_time;
    if (timer->is_running) {
#ifdef _WIN32
        end_time.tv_sec = (long)time(NULL);
        end_time.tv_nsec = 0;
#else
        clock_gettime(CLOCK_MONOTONIC, &end_time);
#endif
    }
    
    double elapsed = (end_time.tv_sec - timer->start_time.tv_sec) +
                    (end_time.tv_nsec - timer->start_time.tv_nsec) / 1e9;
    return elapsed;
}

// Logging functions
bool log_init(const char* log_filename, LogLevel min_level) {
    min_log_level = min_level;
    
    if (log_filename) {
        log_file = fopen(log_filename, "w");
        if (!log_file) {
            return false;
        }
    } else {
        log_file = stdout;
    }
    
    return true;
}

void log_cleanup(void) {
    if (log_file && log_file != stdout && log_file != stderr) {
        fclose(log_file);
    }
    log_file = NULL;
}

void log_message(LogLevel level, const char* format, ...) {
    if (level < min_log_level || !format) return;
    
    const char* level_strings[] = {"DEBUG", "INFO", "WARNING", "ERROR", "FATAL"};
    
    FILE* output = log_file ? log_file : stdout;
    
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    
    fprintf(output, "[%04d-%02d-%02d %02d:%02d:%02d] %s: ",
            tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
            tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
            level_strings[level]);
    
    va_list args;
    va_start(args, format);
    vfprintf(output, format, args);
    va_end(args);
    
    fprintf(output, "\n");
    fflush(output);
}

void log_info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    
    FILE* output = log_file ? log_file : stdout;
    fprintf(output, "[INFO] ");
    vfprintf(output, format, args);
    fprintf(output, "\n");
    fflush(output);
    
    va_end(args);
}

void log_error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    
    FILE* output = log_file ? log_file : stderr;
    fprintf(output, "[ERROR] ");
    vfprintf(output, format, args);
    fprintf(output, "\n");
    fflush(output);
    
    va_end(args);
}

void log_warning(const char* format, ...) {
    va_list args;
    va_start(args, format);
    
    FILE* output = log_file ? log_file : stdout;
    fprintf(output, "[WARNING] ");
    vfprintf(output, format, args);
    fprintf(output, "\n");
    fflush(output);
    
    va_end(args);
}

// Signal generation utilities
bool generate_sine_wave(SignalBuffer* signal, double frequency, double amplitude, double phase) {
    if (!signal || !signal->is_allocated || frequency < 0) {
        return false;
    }
    
    for (size_t i = 0; i < signal->length; i++) {
        double t = (double)i / signal->sample_rate;
        double phase_t = 2.0 * M_PI * frequency * t + phase;
        signal->samples[i].real = amplitude * cos(phase_t);
        signal->samples[i].imag = 0.0; // Real sinusoid
    }
    
    return true;
}

bool generate_complex_exponential(SignalBuffer* signal, double frequency, double amplitude, double phase) {
    if (!signal || !signal->is_allocated) {
        return false;
    }
    
    for (size_t i = 0; i < signal->length; i++) {
        double t = (double)i / signal->sample_rate;
        double phase_t = 2.0 * M_PI * frequency * t + phase;
        signal->samples[i].real = amplitude * cos(phase_t);
        signal->samples[i].imag = amplitude * sin(phase_t);
    }
    
    return true;
}

bool add_awgn_to_signal(SignalBuffer* signal, double noise_power) {
    if (!signal || !signal->is_allocated || noise_power < 0) {
        return false;
    }
    
    for (size_t i = 0; i < signal->length; i++) {
        ComplexSample noise = generate_complex_noise(noise_power);
        signal->samples[i].real += noise.real;
        signal->samples[i].imag += noise.imag;
    }
    
    return true;
}

// File I/O functions
bool save_detections_to_csv(const DetectionResult* detections, uint32_t num_detections, 
                            const char* filename) {
    if (!detections || !filename) {
        return false;
    }
    
    FILE* file = fopen(filename, "w");
    if (!file) {
        return false;
    }
    
    // Write header
    fprintf(file, "target_id,range_m,velocity_ms,snr_db,confidence,valid\n");
    
    // Write data
    for (uint32_t i = 0; i < num_detections; i++) {
        const DetectionResult* det = &detections[i];
        fprintf(file, "%u,%.3f,%.3f,%.2f,%.3f,%s\n",
                det->target_id, det->detected_range, det->detected_velocity,
                det->snr_db, det->confidence, 
                det->is_valid ? "true" : "false");
    }
    
    fclose(file);
    return true;
}

bool validate_signal_buffer(const SignalBuffer* signal) {
    if (!signal) return false;
    
    return signal->is_allocated && 
           signal->samples != NULL && 
           signal->length > 0 && 
           signal->sample_rate > MIN_SAMPLE_RATE &&
           signal->sample_rate < MAX_SAMPLE_RATE;
}
