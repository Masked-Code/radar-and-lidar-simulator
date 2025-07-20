/**
 * @file test_fft.c
 * @brief FFT processor unit tests
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <string.h>
#include <stdbool.h>

#include "fft_processor.h"
#include "signal_types.h"
#include "radar_config.h"
#include "utils.h"

#define TEST_TOLERANCE 1e-10
#define FLOAT_TOLERANCE 1e-6

// Test framework macros
#define TEST_ASSERT(condition) \
    do { \
        if (!(condition)) { \
            printf("FAIL: %s:%d - Assertion failed: %s\n", __FILE__, __LINE__, #condition); \
            return false; \
        } \
    } while(0)

#define TEST_ASSERT_NEAR(expected, actual, tolerance) \
    do { \
        double diff = fabs((double)(expected) - (double)(actual)); \
        if (diff > (tolerance)) { \
            printf("FAIL: %s:%d - Expected %g ± %g, got %g (diff: %g)\n", \
                   __FILE__, __LINE__, (double)(expected), (tolerance), (double)(actual), diff); \
            return false; \
        } \
    } while(0)

#define TEST_ASSERT_NOT_NULL(ptr) \
    do { \
        if ((ptr) == NULL) { \
            printf("FAIL: %s:%d - Pointer should not be NULL: %s\n", __FILE__, __LINE__, #ptr); \
            return false; \
        } \
    } while(0)

// ============================================================================
// FFT INITIALIZATION TESTS
// ============================================================================

bool test_fft_initialization(void) {
    FFTProcessor processor;
    
    // Test successful initialization
    TEST_ASSERT(fft_processor_init(&processor, 1024, WINDOW_HAMMING));
    TEST_ASSERT(processor.is_initialized);
    TEST_ASSERT(processor.fft_size == 1024);
    TEST_ASSERT(processor.window_type == WINDOW_HAMMING);
    TEST_ASSERT_NOT_NULL(processor.input_buffer);
    TEST_ASSERT_NOT_NULL(processor.output_buffer);
    TEST_ASSERT_NOT_NULL(processor.window_function);
    
    fft_processor_cleanup(&processor);
    
    // Test initialization with different sizes
    const uint32_t test_sizes[] = {64, 128, 256, 512, 1024, 2048, 4096};
    const size_t num_sizes = sizeof(test_sizes) / sizeof(test_sizes[0]);
    
    for (size_t i = 0; i < num_sizes; i++) {
        TEST_ASSERT(fft_processor_init(&processor, test_sizes[i], WINDOW_RECTANGULAR));
        TEST_ASSERT(processor.fft_size == test_sizes[i]);
        fft_processor_cleanup(&processor);
    }
    
    // Test invalid FFT sizes
    TEST_ASSERT(!fft_processor_init(&processor, 0, WINDOW_HAMMING));
    TEST_ASSERT(!fft_processor_init(&processor, 100, WINDOW_HAMMING)); // Not power of 2
    TEST_ASSERT(!fft_processor_init(&processor, MAX_FFT_SIZE + 1, WINDOW_HAMMING));
    
    return true;
}

// ============================================================================
// FFT FORWARD TRANSFORM TESTS
// ============================================================================

bool test_fft_forward_transform(void) {
    FFTProcessor processor;
    SignalBuffer input_signal, output_signal;
    const uint32_t fft_size = 256;
    
    // Initialize FFT processor
    TEST_ASSERT(fft_processor_init(&processor, fft_size, WINDOW_RECTANGULAR));
    
    // Initialize signal buffers
    TEST_ASSERT(allocate_signal_buffer(&input_signal, fft_size, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(allocate_signal_buffer(&output_signal, fft_size, DEFAULT_SAMPLE_RATE));
    
    // Test with known signal: DC component
    for (size_t i = 0; i < fft_size; i++) {
        input_signal.samples[i].real = 1.0;  // DC signal
        input_signal.samples[i].imag = 0.0;
    }
    
    TEST_ASSERT(fft_forward(&processor, &input_signal, &output_signal));
    
    // Check DC component (should be at bin 0)
    TEST_ASSERT_NEAR(output_signal.samples[0].real, (double)fft_size, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(output_signal.samples[0].imag, 0.0, FLOAT_TOLERANCE);
    
    // Check other bins should be close to zero
    for (size_t i = 1; i < fft_size; i++) {
        double magnitude = sqrt(output_signal.samples[i].real * output_signal.samples[i].real + 
                               output_signal.samples[i].imag * output_signal.samples[i].imag);
        TEST_ASSERT(magnitude < FLOAT_TOLERANCE);
    }
    
    // Test with sinusoidal signal
    const double test_frequency = DEFAULT_SAMPLE_RATE / 16.0; // 16 samples per cycle
    for (size_t i = 0; i < fft_size; i++) {
        double t = (double)i / DEFAULT_SAMPLE_RATE;
        input_signal.samples[i].real = cos(2.0 * M_PI * test_frequency * t);
        input_signal.samples[i].imag = 0.0;
    }
    
    TEST_ASSERT(fft_forward(&processor, &input_signal, &output_signal));
    
    // Find peak frequency bin
    size_t peak_bin = 0;
    double peak_magnitude = 0.0;
    
    for (size_t i = 0; i < fft_size / 2; i++) {
        double magnitude = sqrt(output_signal.samples[i].real * output_signal.samples[i].real + 
                               output_signal.samples[i].imag * output_signal.samples[i].imag);
        if (magnitude > peak_magnitude) {
            peak_magnitude = magnitude;
            peak_bin = i;
        }
    }
    
    // Check that peak is at expected frequency bin
    double expected_bin = test_frequency * fft_size / DEFAULT_SAMPLE_RATE;
    TEST_ASSERT_NEAR((double)peak_bin, expected_bin, 1.0);
    
    // Cleanup
    fft_processor_cleanup(&processor);
    free_signal_buffer(&input_signal);
    free_signal_buffer(&output_signal);
    
    return true;
}

// ============================================================================
// FFT INVERSE TRANSFORM TESTS
// ============================================================================

bool test_fft_inverse_transform(void) {
    FFTProcessor processor;
    SignalBuffer original_signal, freq_domain, reconstructed_signal;
    const uint32_t fft_size = 128;
    
    // Initialize FFT processor
    TEST_ASSERT(fft_processor_init(&processor, fft_size, WINDOW_RECTANGULAR));
    
    // Initialize signal buffers
    TEST_ASSERT(allocate_signal_buffer(&original_signal, fft_size, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(allocate_signal_buffer(&freq_domain, fft_size, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(allocate_signal_buffer(&reconstructed_signal, fft_size, DEFAULT_SAMPLE_RATE));
    
    // Create test signal
    for (size_t i = 0; i < fft_size; i++) {
        double t = (double)i / DEFAULT_SAMPLE_RATE;
        original_signal.samples[i].real = sin(2.0 * M_PI * 1000.0 * t) + 
                                         0.5 * cos(2.0 * M_PI * 2000.0 * t);
        original_signal.samples[i].imag = 0.0;
    }
    
    // Forward transform
    TEST_ASSERT(fft_forward(&processor, &original_signal, &freq_domain));
    
    // Inverse transform
    TEST_ASSERT(fft_inverse(&processor, &freq_domain, &reconstructed_signal));
    
    // Check reconstruction accuracy
    for (size_t i = 0; i < fft_size; i++) {
        TEST_ASSERT_NEAR(original_signal.samples[i].real, reconstructed_signal.samples[i].real, FLOAT_TOLERANCE);
        TEST_ASSERT_NEAR(original_signal.samples[i].imag, reconstructed_signal.samples[i].imag, FLOAT_TOLERANCE);
    }
    
    // Cleanup
    fft_processor_cleanup(&processor);
    free_signal_buffer(&original_signal);
    free_signal_buffer(&freq_domain);
    free_signal_buffer(&reconstructed_signal);
    
    return true;
}

// ============================================================================
// FFT REAL FORWARD TESTS
// ============================================================================

bool test_fft_real_forward(void) {
    FFTProcessor processor;
    const uint32_t signal_length = 256;
    double* real_input = malloc(signal_length * sizeof(double));
    ComplexSample* complex_output = malloc(signal_length * sizeof(ComplexSample));
    
    TEST_ASSERT_NOT_NULL(real_input);
    TEST_ASSERT_NOT_NULL(complex_output);
    
    // Initialize FFT processor
    TEST_ASSERT(fft_processor_init(&processor, signal_length, WINDOW_RECTANGULAR));
    
    // Create real test signal
    const double test_freq = DEFAULT_SAMPLE_RATE / 32.0;
    for (size_t i = 0; i < signal_length; i++) {
        double t = (double)i / DEFAULT_SAMPLE_RATE;
        real_input[i] = cos(2.0 * M_PI * test_freq * t);
    }
    
    // Perform real FFT
    TEST_ASSERT(fft_real_forward(&processor, real_input, signal_length, complex_output));
    
    // Find peak frequency
    size_t peak_bin = 0;
    double peak_magnitude = 0.0;
    
    for (size_t i = 0; i < signal_length / 2; i++) {
        double magnitude = sqrt(complex_output[i].real * complex_output[i].real + 
                               complex_output[i].imag * complex_output[i].imag);
        if (magnitude > peak_magnitude) {
            peak_magnitude = magnitude;
            peak_bin = i;
        }
    }
    
    // Check that peak is at expected frequency
    double expected_bin = test_freq * signal_length / DEFAULT_SAMPLE_RATE;
    TEST_ASSERT_NEAR((double)peak_bin, expected_bin, 1.0);
    
    // Cleanup
    fft_processor_cleanup(&processor);
    free(real_input);
    free(complex_output);
    
    return true;
}

// ============================================================================
// WINDOW FUNCTION TESTS
// ============================================================================

bool test_fft_window_functions(void) {
    const uint32_t window_length = 64;
    double* coefficients = malloc(window_length * sizeof(double));
    TEST_ASSERT_NOT_NULL(coefficients);
    
    // Test different window types
    const WindowType window_types[] = {
        WINDOW_RECTANGULAR,
        WINDOW_HAMMING,
        WINDOW_HANNING,
        WINDOW_BLACKMAN,
        WINDOW_GAUSSIAN,
        WINDOW_BARTLETT
    };
    const size_t num_types = sizeof(window_types) / sizeof(window_types[0]);
    
    for (size_t i = 0; i < num_types; i++) {
        TEST_ASSERT(generate_window(window_types[i], window_length, coefficients, 0.0));
        
        // Check window properties
        TEST_ASSERT(coefficients[0] >= 0.0 && coefficients[0] <= 1.0);
        TEST_ASSERT(coefficients[window_length - 1] >= 0.0 && coefficients[window_length - 1] <= 1.0);
        
        // For symmetric windows, check symmetry
        if (window_types[i] != WINDOW_RECTANGULAR) {
            for (size_t j = 0; j < window_length / 2; j++) {
                TEST_ASSERT_NEAR(coefficients[j], coefficients[window_length - 1 - j], FLOAT_TOLERANCE);
            }
        }
    }
    
    // Test window correction factors
    TEST_ASSERT(generate_window(WINDOW_HAMMING, window_length, coefficients, 0.0));
    
    double amplitude_correction, power_correction;
    TEST_ASSERT(calculate_window_correction(coefficients, window_length, 
                                          &amplitude_correction, &power_correction));
    
    TEST_ASSERT(amplitude_correction > 0.0);
    TEST_ASSERT(power_correction > 0.0);
    
    free(coefficients);
    return true;
}

// ============================================================================
// SPECTRAL ANALYSIS TESTS
// ============================================================================

bool test_fft_spectral_analysis(void) {
    FFTProcessor processor;
    SignalBuffer test_signal;
    SpectralAnalysis analysis;
    const uint32_t signal_length = 512;
    
    // Initialize
    TEST_ASSERT(fft_processor_init(&processor, signal_length, WINDOW_HAMMING));
    TEST_ASSERT(allocate_signal_buffer(&test_signal, signal_length, DEFAULT_SAMPLE_RATE));
    
    // Create multi-tone test signal
    const double freq1 = 1000.0;
    const double freq2 = 2500.0;
    const double amplitude1 = 1.0;
    const double amplitude2 = 0.5;
    
    for (size_t i = 0; i < signal_length; i++) {
        double t = (double)i / DEFAULT_SAMPLE_RATE;
        test_signal.samples[i].real = amplitude1 * sin(2.0 * M_PI * freq1 * t) + 
                                     amplitude2 * sin(2.0 * M_PI * freq2 * t);
        test_signal.samples[i].imag = 0.0;
    }
    
    // Perform spectral analysis
    TEST_ASSERT(spectral_analysis(&processor, &test_signal, &analysis));
    
    // Verify analysis results
    TEST_ASSERT(analysis.spectrum_length == signal_length);
    TEST_ASSERT_NOT_NULL(analysis.frequency_bins);
    TEST_ASSERT_NOT_NULL(analysis.magnitude_spectrum);
    TEST_ASSERT_NOT_NULL(analysis.phase_spectrum);
    TEST_ASSERT_NOT_NULL(analysis.power_spectrum);
    TEST_ASSERT(analysis.frequency_resolution > 0.0);
    TEST_ASSERT(analysis.peak_frequency > 0.0);
    TEST_ASSERT(analysis.peak_magnitude > 0.0);
    TEST_ASSERT(analysis.total_power > 0.0);
    
    // Find peaks
    double peak_frequencies[10];
    double peak_magnitudes[10];
    uint32_t num_peaks = find_spectral_peaks(analysis.magnitude_spectrum, analysis.spectrum_length,
                                           10.0, peak_frequencies, peak_magnitudes, 10);
    
    TEST_ASSERT(num_peaks >= 2); // Should find at least the two tones
    
    // Cleanup
    spectral_analysis_cleanup(&analysis);
    fft_processor_cleanup(&processor);
    free_signal_buffer(&test_signal);
    
    return true;
}

// ============================================================================
// CROSS-CORRELATION TESTS
// ============================================================================

bool test_fft_cross_correlation(void) {
    FFTProcessor processor;
    SignalBuffer signal1, signal2, correlation;
    const uint32_t signal_length = 128;
    
    // Initialize
    TEST_ASSERT(fft_processor_init(&processor, signal_length * 2, WINDOW_RECTANGULAR));
    TEST_ASSERT(allocate_signal_buffer(&signal1, signal_length, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(allocate_signal_buffer(&signal2, signal_length, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(allocate_signal_buffer(&correlation, signal_length * 2, DEFAULT_SAMPLE_RATE));
    
    // Create test signals
    for (size_t i = 0; i < signal_length; i++) {
        signal1.samples[i].real = (i < signal_length / 4) ? 1.0 : 0.0;
        signal1.samples[i].imag = 0.0;
        signal2.samples[i].real = (i >= 10 && i < signal_length / 4 + 10) ? 1.0 : 0.0;
        signal2.samples[i].imag = 0.0;
    }
    
    // Perform cross-correlation
    TEST_ASSERT(fft_cross_correlation(&processor, &signal1, &signal2, &correlation));
    
    // Find peak correlation
    size_t peak_index = 0;
    double peak_value = 0.0;
    
    for (size_t i = 0; i < correlation.length; i++) {
        double magnitude = sqrt(correlation.samples[i].real * correlation.samples[i].real + 
                               correlation.samples[i].imag * correlation.samples[i].imag);
        if (magnitude > peak_value) {
            peak_value = magnitude;
            peak_index = i;
        }
    }
    
    // Peak should occur at delay corresponding to the 10-sample shift
    TEST_ASSERT(peak_index > 0);
    TEST_ASSERT(peak_value > 0.0);
    
    // Cleanup
    fft_processor_cleanup(&processor);
    free_signal_buffer(&signal1);
    free_signal_buffer(&signal2);
    free_signal_buffer(&correlation);
    
    return true;
}

// ============================================================================
// CONVOLUTION TESTS
// ============================================================================

bool test_fft_convolution(void) {
    FFTProcessor processor;
    SignalBuffer signal, kernel, result;
    const uint32_t signal_length = 100;
    const uint32_t kernel_length = 10;
    const uint32_t result_length = signal_length + kernel_length - 1;
    
    // Initialize
    TEST_ASSERT(fft_processor_init(&processor, 256, WINDOW_RECTANGULAR)); // Next power of 2
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(allocate_signal_buffer(&kernel, kernel_length, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(allocate_signal_buffer(&result, result_length, DEFAULT_SAMPLE_RATE));
    
    // Create test signal (impulse at center)
    for (size_t i = 0; i < signal_length; i++) {
        signal.samples[i].real = (i == signal_length / 2) ? 1.0 : 0.0;
        signal.samples[i].imag = 0.0;
    }
    
    // Create simple averaging kernel
    for (size_t i = 0; i < kernel_length; i++) {
        kernel.samples[i].real = 1.0 / kernel_length;
        kernel.samples[i].imag = 0.0;
    }
    
    // Perform convolution
    TEST_ASSERT(fft_convolution(&processor, &signal, &kernel, &result));
    
    // Check result - should be a scaled and shifted version of the kernel
    double sum_magnitude = 0.0;
    for (size_t i = 0; i < result.length; i++) {
        double magnitude = sqrt(result.samples[i].real * result.samples[i].real + 
                               result.samples[i].imag * result.samples[i].imag);
        sum_magnitude += magnitude;
    }
    
    TEST_ASSERT_NEAR(sum_magnitude, 1.0, FLOAT_TOLERANCE);
    
    // Cleanup
    fft_processor_cleanup(&processor);
    free_signal_buffer(&signal);
    free_signal_buffer(&kernel);
    free_signal_buffer(&result);
    
    return true;
}
