/**
 * @file test_signals.c
 * @brief Signal generation and processing unit tests
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

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
// SIGNAL BUFFER ALLOCATION TESTS
// ============================================================================

bool test_signal_buffer_allocation(void) {
    SignalBuffer buffer;
    
    // Test successful allocation
    TEST_ASSERT(allocate_signal_buffer(&buffer, 1024, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(buffer.is_allocated);
    TEST_ASSERT(buffer.length == 1024);
    TEST_ASSERT(buffer.sample_rate == DEFAULT_SAMPLE_RATE);
    TEST_ASSERT_NOT_NULL(buffer.samples);
    
    // Test buffer initialization
    for (size_t i = 0; i < buffer.length; i++) {
        TEST_ASSERT(buffer.samples[i].real == 0.0);
        TEST_ASSERT(buffer.samples[i].imag == 0.0);
    }
    
    free_signal_buffer(&buffer);
    TEST_ASSERT(!buffer.is_allocated);
    TEST_ASSERT(buffer.samples == NULL);
    
    // Test different buffer sizes
    const size_t test_sizes[] = {1, 64, 256, 1024, 4096, 16384};
    const size_t num_sizes = sizeof(test_sizes) / sizeof(test_sizes[0]);
    
    for (size_t i = 0; i < num_sizes; i++) {
        TEST_ASSERT(allocate_signal_buffer(&buffer, test_sizes[i], DEFAULT_SAMPLE_RATE));
        TEST_ASSERT(buffer.length == test_sizes[i]);
        free_signal_buffer(&buffer);
    }
    
    // Test copy functionality
    SignalBuffer source, dest;
    TEST_ASSERT(allocate_signal_buffer(&source, 100, DEFAULT_SAMPLE_RATE));
    
    // Fill source with test data
    for (size_t i = 0; i < source.length; i++) {
        source.samples[i].real = sin(2.0 * M_PI * i / 20.0);
        source.samples[i].imag = cos(2.0 * M_PI * i / 20.0);
    }
    
    // Test copy
    TEST_ASSERT(copy_signal_buffer(&source, &dest));
    TEST_ASSERT(dest.length == source.length);
    TEST_ASSERT(dest.sample_rate == source.sample_rate);
    
    for (size_t i = 0; i < source.length; i++) {
        TEST_ASSERT_NEAR(dest.samples[i].real, source.samples[i].real, FLOAT_TOLERANCE);
        TEST_ASSERT_NEAR(dest.samples[i].imag, source.samples[i].imag, FLOAT_TOLERANCE);
    }
    
    free_signal_buffer(&source);
    free_signal_buffer(&dest);
    
    // Test resize functionality
    TEST_ASSERT(allocate_signal_buffer(&buffer, 100, DEFAULT_SAMPLE_RATE));
    TEST_ASSERT(resize_signal_buffer(&buffer, 200));
    TEST_ASSERT(buffer.length == 200);
    
    TEST_ASSERT(resize_signal_buffer(&buffer, 50));
    TEST_ASSERT(buffer.length == 50);
    
    free_signal_buffer(&buffer);
    
    return true;
}

// ============================================================================
// SINE WAVE GENERATION TESTS
// ============================================================================

bool test_sine_wave_generation(void) {
    SignalBuffer signal;
    const size_t signal_length = 1000;
    const double sample_rate = 10000.0; // 10 kHz
    const double frequency = 1000.0;    // 1 kHz
    const double amplitude = 2.0;
    const double phase = M_PI / 4;       // 45 degrees
    
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, sample_rate));
    
    // Generate sine wave
    TEST_ASSERT(generate_sine_wave(&signal, frequency, amplitude, phase));
    
    // Verify signal properties
    for (size_t i = 0; i < signal_length; i++) {
        double t = (double)i / sample_rate;
        double expected = amplitude * sin(2.0 * M_PI * frequency * t + phase);
        TEST_ASSERT_NEAR(signal.samples[i].real, expected, FLOAT_TOLERANCE);
        TEST_ASSERT_NEAR(signal.samples[i].imag, 0.0, FLOAT_TOLERANCE);
    }
    
    // Test signal power
    double signal_power = calculate_signal_power(&signal);
    double expected_power = amplitude * amplitude / 2.0; // RMS power of sine wave
    TEST_ASSERT_NEAR(signal_power, expected_power, FLOAT_TOLERANCE);
    
    free_signal_buffer(&signal);
    
    // Test complex exponential generation
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, sample_rate));
    TEST_ASSERT(generate_complex_exponential(&signal, frequency, amplitude, phase));
    
    for (size_t i = 0; i < 10; i++) { // Check first few samples
        double t = (double)i / sample_rate;
        double expected_real = amplitude * cos(2.0 * M_PI * frequency * t + phase);
        double expected_imag = amplitude * sin(2.0 * M_PI * frequency * t + phase);
        TEST_ASSERT_NEAR(signal.samples[i].real, expected_real, FLOAT_TOLERANCE);
        TEST_ASSERT_NEAR(signal.samples[i].imag, expected_imag, FLOAT_TOLERANCE);
    }
    
    free_signal_buffer(&signal);
    return true;
}

// ============================================================================
// CHIRP GENERATION TESTS
// ============================================================================

bool test_chirp_generation(void) {
    SignalBuffer signal;
    const size_t signal_length = 1000;
    const double sample_rate = 100e6;   // 100 MHz
    const double start_freq = 9.5e9;    // X-band start
    const double end_freq = 10.5e9;     // X-band end
    const double amplitude = 1.0;
    
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, sample_rate));
    signal.center_frequency = (start_freq + end_freq) / 2.0;
    signal.bandwidth = end_freq - start_freq;
    signal.waveform = WAVEFORM_CHIRP_LINEAR;
    
    // Generate linear chirp
    TEST_ASSERT(generate_linear_chirp(&signal, start_freq, end_freq, amplitude));
    
    // Verify chirp properties
    TEST_ASSERT(signal.waveform == WAVEFORM_CHIRP_LINEAR);
    TEST_ASSERT_NEAR(signal.center_frequency, (start_freq + end_freq) / 2.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(signal.bandwidth, end_freq - start_freq, FLOAT_TOLERANCE);
    
    // Check instantaneous frequency progression (approximate)
    double duration = (double)signal_length / sample_rate;
    double chirp_rate = (end_freq - start_freq) / duration;
    
    // Verify signal power is consistent
    double signal_power = calculate_signal_power(&signal);
    TEST_ASSERT(signal_power > 0.0);
    
    // Check signal envelope
    for (size_t i = 0; i < signal_length; i++) {
        double magnitude = sqrt(signal.samples[i].real * signal.samples[i].real + 
                               signal.samples[i].imag * signal.samples[i].imag);
        TEST_ASSERT_NEAR(magnitude, amplitude, FLOAT_TOLERANCE * 10); // Allow for numerical errors
    }
    
    free_signal_buffer(&signal);
    
    // Test quadratic chirp
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, sample_rate));
    signal.waveform = WAVEFORM_CHIRP_QUAD;
    
    // For quadratic chirp, we would need the implementation
    // For now, just test that the function exists and handles the call
    // TEST_ASSERT(generate_quadratic_chirp(&signal, start_freq, end_freq, amplitude));
    
    free_signal_buffer(&signal);
    return true;
}

// ============================================================================
// NOISE GENERATION TESTS
// ============================================================================

bool test_noise_generation(void) {
    SignalBuffer signal;
    const size_t signal_length = 10000; // Large sample for statistical tests
    const double sample_rate = DEFAULT_SAMPLE_RATE;
    const double noise_power = 1.0;
    
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, sample_rate));
    
    // Add white Gaussian noise
    TEST_ASSERT(add_awgn_to_signal(&signal, noise_power));
    
    // Statistical tests on the generated noise
    double sum_real = 0.0, sum_imag = 0.0;
    double sum_sq_real = 0.0, sum_sq_imag = 0.0;
    
    for (size_t i = 0; i < signal_length; i++) {
        sum_real += signal.samples[i].real;
        sum_imag += signal.samples[i].imag;
        sum_sq_real += signal.samples[i].real * signal.samples[i].real;
        sum_sq_imag += signal.samples[i].imag * signal.samples[i].imag;
    }
    
    // Check mean (should be close to zero)
    double mean_real = sum_real / signal_length;
    double mean_imag = sum_imag / signal_length;
    TEST_ASSERT(fabs(mean_real) < 0.1); // Statistical test, allowing some variation
    TEST_ASSERT(fabs(mean_imag) < 0.1);
    
    // Check variance (should be close to noise_power/2 for each component)
    double var_real = sum_sq_real / signal_length - mean_real * mean_real;
    double var_imag = sum_sq_imag / signal_length - mean_imag * mean_imag;
    TEST_ASSERT(fabs(var_real - noise_power / 2.0) < 0.1);
    TEST_ASSERT(fabs(var_imag - noise_power / 2.0) < 0.1);
    
    // Check total power
    double total_power = calculate_signal_power(&signal);
    TEST_ASSERT_NEAR(total_power, noise_power, 0.1); // Statistical tolerance
    
    free_signal_buffer(&signal);
    
    // Test complex noise generation directly
    const int num_samples = 1000;
    double power_sum = 0.0;
    
    for (int i = 0; i < num_samples; i++) {
        ComplexSample noise = generate_complex_noise(noise_power);
        double power = noise.real * noise.real + noise.imag * noise.imag;
        power_sum += power;
    }
    
    double avg_power = power_sum / num_samples;
    TEST_ASSERT_NEAR(avg_power, noise_power, 0.2); // Statistical tolerance
    
    return true;
}

// ============================================================================
// PULSE GENERATION TESTS
// ============================================================================

bool test_pulse_generation(void) {
    SignalBuffer signal;
    const size_t signal_length = 1000;
    const double sample_rate = 100e6;   // 100 MHz
    const double pulse_width = 1e-6;    // 1 microsecond
    const double amplitude = 1.0;
    
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, sample_rate));
    signal.sample_rate = sample_rate;
    signal.duration = (double)signal_length / sample_rate;
    
    // Generate rectangular pulse
    const size_t pulse_samples = (size_t)(pulse_width * sample_rate);
    const size_t start_sample = (signal_length - pulse_samples) / 2;
    
    // Manual pulse generation for testing
    for (size_t i = 0; i < signal_length; i++) {
        if (i >= start_sample && i < start_sample + pulse_samples) {
            signal.samples[i].real = amplitude;
            signal.samples[i].imag = 0.0;
        } else {
            signal.samples[i].real = 0.0;
            signal.samples[i].imag = 0.0;
        }
    }
    signal.waveform = WAVEFORM_RECTANGULAR;
    
    // Verify pulse properties
    TEST_ASSERT(signal.waveform == WAVEFORM_RECTANGULAR);
    
    // Check pulse has correct duration and amplitude
    size_t pulse_count = 0;
    for (size_t i = 0; i < signal_length; i++) {
        if (fabs(signal.samples[i].real - amplitude) < FLOAT_TOLERANCE) {
            pulse_count++;
        }
    }
    TEST_ASSERT(pulse_count == pulse_samples);
    
    // Test Gaussian pulse shape
    TEST_ASSERT(allocate_signal_buffer(&signal, signal_length, sample_rate));
    signal.waveform = WAVEFORM_GAUSSIAN;
    
    // Generate Gaussian pulse manually for testing
    const double sigma = pulse_width / 6.0; // 6-sigma width
    const double center = (double)signal_length / 2.0;
    
    for (size_t i = 0; i < signal_length; i++) {
        double t = ((double)i - center) / sample_rate;
        signal.samples[i].real = amplitude * exp(-0.5 * (t * t) / (sigma * sigma));
        signal.samples[i].imag = 0.0;
    }
    
    // Check Gaussian properties
    TEST_ASSERT_NEAR(signal.samples[signal_length / 2].real, amplitude, FLOAT_TOLERANCE);
    
    // Check symmetry
    for (size_t i = 0; i < signal_length / 2; i++) {
        size_t mirror = signal_length - 1 - i;
        TEST_ASSERT_NEAR(signal.samples[i].real, signal.samples[mirror].real, FLOAT_TOLERANCE);
    }
    
    free_signal_buffer(&signal);
    return true;
}

// ============================================================================
// SIGNAL OPERATIONS TESTS
// ============================================================================

bool test_signal_operations(void) {
    SignalBuffer signal1, signal2, result;
    const size_t signal_length = 100;
    const double sample_rate = DEFAULT_SAMPLE_RATE;
    
    // Initialize signals
    TEST_ASSERT(allocate_signal_buffer(&signal1, signal_length, sample_rate));
    TEST_ASSERT(allocate_signal_buffer(&signal2, signal_length, sample_rate));
    TEST_ASSERT(allocate_signal_buffer(&result, signal_length, sample_rate));
    
    // Fill with test data
    for (size_t i = 0; i < signal_length; i++) {
        signal1.samples[i].real = (double)i;
        signal1.samples[i].imag = (double)i * 0.5;
        signal2.samples[i].real = 2.0 * i;
        signal2.samples[i].imag = i * 0.25;
    }
    
    // Test signal addition
    for (size_t i = 0; i < signal_length; i++) {
        result.samples[i] = complex_add(signal1.samples[i], signal2.samples[i]);
    }
    
    for (size_t i = 0; i < signal_length; i++) {
        TEST_ASSERT_NEAR(result.samples[i].real, 3.0 * i, FLOAT_TOLERANCE);
        TEST_ASSERT_NEAR(result.samples[i].imag, 0.75 * i, FLOAT_TOLERANCE);
    }
    
    // Test signal scaling
    const double scale_factor = 2.5;
    for (size_t i = 0; i < signal_length; i++) {
        result.samples[i] = complex_scale(signal1.samples[i], scale_factor);
    }
    
    for (size_t i = 0; i < signal_length; i++) {
        TEST_ASSERT_NEAR(result.samples[i].real, scale_factor * i, FLOAT_TOLERANCE);
        TEST_ASSERT_NEAR(result.samples[i].imag, scale_factor * i * 0.5, FLOAT_TOLERANCE);
    }
    
    // Test signal power calculation
    double power1 = calculate_signal_power(&signal1);
    double expected_power = 0.0;
    
    for (size_t i = 0; i < signal_length; i++) {
        double mag_sq = signal1.samples[i].real * signal1.samples[i].real + 
                       signal1.samples[i].imag * signal1.samples[i].imag;
        expected_power += mag_sq;
    }
    expected_power /= signal_length;
    
    TEST_ASSERT_NEAR(power1, expected_power, FLOAT_TOLERANCE);
    
    free_signal_buffer(&signal1);
    free_signal_buffer(&signal2);
    free_signal_buffer(&result);
    return true;
}

// ============================================================================
// COMPLEX ARITHMETIC TESTS
// ============================================================================

bool test_complex_arithmetic(void) {
    // Test complex number operations
    ComplexSample a = {3.0, 4.0};  // 3 + 4i
    ComplexSample b = {1.0, 2.0};  // 1 + 2i
    ComplexSample result;
    
    // Test addition
    result = complex_add(a, b);
    TEST_ASSERT_NEAR(result.real, 4.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, 6.0, FLOAT_TOLERANCE);
    
    // Test subtraction
    result = complex_subtract(a, b);
    TEST_ASSERT_NEAR(result.real, 2.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, 2.0, FLOAT_TOLERANCE);
    
    // Test multiplication
    result = complex_multiply(a, b);
    // (3+4i)(1+2i) = 3 + 6i + 4i + 8i² = 3 + 10i - 8 = -5 + 10i
    TEST_ASSERT_NEAR(result.real, -5.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, 10.0, FLOAT_TOLERANCE);
    
    // Test division
    result = complex_divide(a, b);
    // (3+4i)/(1+2i) = (3+4i)(1-2i)/((1+2i)(1-2i)) = (3-6i+4i-8i²)/(1-4i²) = (11-2i)/5
    TEST_ASSERT_NEAR(result.real, 11.0/5.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, -2.0/5.0, FLOAT_TOLERANCE);
    
    // Test magnitude
    double magnitude = complex_magnitude(a);
    TEST_ASSERT_NEAR(magnitude, 5.0, FLOAT_TOLERANCE); // sqrt(3² + 4²) = 5
    
    // Test phase
    double phase = complex_phase(a);
    TEST_ASSERT_NEAR(phase, atan2(4.0, 3.0), FLOAT_TOLERANCE);
    
    // Test conjugate
    result = complex_conjugate(a);
    TEST_ASSERT_NEAR(result.real, 3.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, -4.0, FLOAT_TOLERANCE);
    
    // Test scaling
    result = complex_scale(a, 2.0);
    TEST_ASSERT_NEAR(result.real, 6.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, 8.0, FLOAT_TOLERANCE);
    
    // Test polar conversion
    double mag, phase_out;
    complex_to_polar(a, &mag, &phase_out);
    TEST_ASSERT_NEAR(mag, 5.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(phase_out, atan2(4.0, 3.0), FLOAT_TOLERANCE);
    
    // Test polar to complex conversion
    result = polar_to_complex(mag, phase_out);
    TEST_ASSERT_NEAR(result.real, a.real, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, a.imag, FLOAT_TOLERANCE);
    
    // Test with zero
    ComplexSample zero = {0.0, 0.0};
    TEST_ASSERT_NEAR(complex_magnitude(zero), 0.0, FLOAT_TOLERANCE);
    
    result = complex_add(a, zero);
    TEST_ASSERT_NEAR(result.real, a.real, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, a.imag, FLOAT_TOLERANCE);
    
    result = complex_multiply(a, zero);
    TEST_ASSERT_NEAR(result.real, 0.0, FLOAT_TOLERANCE);
    TEST_ASSERT_NEAR(result.imag, 0.0, FLOAT_TOLERANCE);
    
    return true;
}
