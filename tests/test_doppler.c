/**
 * @file test_doppler.c
 * @brief Doppler analysis unit tests
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "doppler_analyzer.h"
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
// DOPPLER PARAMETERS INITIALIZATION TESTS
// ============================================================================

bool test_doppler_params_initialization(void) {
    DopplerParams params;
    
    const double carrier_freq = DEFAULT_RADAR_FREQUENCY;
    const double prf = 1000.0;
    const uint32_t num_pulses = 16;
    
    // Test successful initialization
    TEST_ASSERT(doppler_params_init(&params, carrier_freq, prf, num_pulses));
    TEST_ASSERT(params.carrier_frequency == carrier_freq);
    TEST_ASSERT(params.prf == prf);
    TEST_ASSERT(params.coherent_pulses == num_pulses);
    TEST_ASSERT(params.doppler_fft_size >= num_pulses);
    TEST_ASSERT(params.max_unambiguous_velocity > 0.0);
    TEST_ASSERT(params.velocity_resolution > 0.0);
    
    // Check max unambiguous velocity calculation
    double expected_max_vel = calculate_max_unambiguous_velocity(prf, carrier_freq);
    TEST_ASSERT_NEAR(params.max_unambiguous_velocity, expected_max_vel, FLOAT_TOLERANCE);
    
    // Check velocity resolution
    double expected_vel_res = 2.0 * params.max_unambiguous_velocity / params.doppler_fft_size;
    TEST_ASSERT_NEAR(params.velocity_resolution, expected_vel_res, FLOAT_TOLERANCE);
    
    // Test parameter validation
    TEST_ASSERT(validate_doppler_parameters(&params));
    
    // Test with different parameters
    const double test_frequencies[] = {1e9, 5e9, 10e9, 35e9};
    const double test_prfs[] = {100.0, 1000.0, 5000.0, 10000.0};
    const uint32_t test_pulses[] = {8, 16, 32, 64};
    
    for (size_t i = 0; i < 4; i++) {
        TEST_ASSERT(doppler_params_init(&params, test_frequencies[i], 
                                       test_prfs[i], test_pulses[i]));
        TEST_ASSERT(validate_doppler_parameters(&params));
    }
    
    // Test invalid parameters
    TEST_ASSERT(!doppler_params_init(&params, 0.0, prf, num_pulses));      // Invalid frequency
    TEST_ASSERT(!doppler_params_init(&params, carrier_freq, 0.0, num_pulses)); // Invalid PRF
    TEST_ASSERT(!doppler_params_init(&params, carrier_freq, prf, 0));      // Invalid pulse count
    
    return true;
}

// ============================================================================
// VELOCITY-DOPPLER CONVERSION TESTS
// ============================================================================

bool test_velocity_doppler_conversion(void) {
    const double carrier_freq = 10e9; // X-band
    const double test_velocities[] = {0.0, 10.0, 50.0, 100.0, 300.0}; // m/s
    const size_t num_velocities = sizeof(test_velocities) / sizeof(test_velocities[0]);
    
    // Test velocity to Doppler conversion
    for (size_t i = 0; i < num_velocities; i++) {
        double velocity = test_velocities[i];
        double doppler_freq = velocity_to_doppler(velocity, carrier_freq);
        
        // Check expected relationship: fd = 2*v*fc/c
        double expected_doppler = 2.0 * velocity * carrier_freq / SPEED_OF_LIGHT;
        TEST_ASSERT_NEAR(doppler_freq, expected_doppler, FLOAT_TOLERANCE);
        
        // Test reverse conversion
        double converted_velocity = doppler_to_velocity(doppler_freq, carrier_freq);
        TEST_ASSERT_NEAR(converted_velocity, velocity, FLOAT_TOLERANCE);
    }
    
    // Test negative velocities (receding targets)
    for (size_t i = 0; i < num_velocities; i++) {
        double velocity = -test_velocities[i];
        double doppler_freq = velocity_to_doppler(velocity, carrier_freq);
        double converted_velocity = doppler_to_velocity(doppler_freq, carrier_freq);
        TEST_ASSERT_NEAR(converted_velocity, velocity, FLOAT_TOLERANCE);
    }
    
    // Test with different carrier frequencies
    const double frequencies[] = {1e9, 5e9, 10e9, 24e9, 35e9};
    const size_t num_freqs = sizeof(frequencies) / sizeof(frequencies[0]);
    
    for (size_t i = 0; i < num_freqs; i++) {
        double velocity = 50.0; // m/s
        double doppler_freq = velocity_to_doppler(velocity, frequencies[i]);
        double converted_velocity = doppler_to_velocity(doppler_freq, frequencies[i]);
        TEST_ASSERT_NEAR(converted_velocity, velocity, FLOAT_TOLERANCE);
    }
    
    return true;
}

// ============================================================================
// DOPPLER SPECTRUM ANALYSIS TESTS
// ============================================================================

bool test_doppler_spectrum_analysis(void) {
    const uint32_t num_pulses = 32;
    const double carrier_freq = 10e9;
    const double prf = 1000.0;
    const double target_velocity = 50.0; // m/s
    const double sample_rate = 100e6;
    const size_t samples_per_pulse = 1000;
    
    // Create pulse train
    SignalBuffer* pulse_train = calloc(num_pulses, sizeof(SignalBuffer));
    TEST_ASSERT_NOT_NULL(pulse_train);
    
    // Generate coherent pulse train with Doppler shift
    double doppler_freq = velocity_to_doppler(target_velocity, carrier_freq);
    double phase_increment = 2.0 * M_PI * doppler_freq / prf;
    
    for (uint32_t pulse = 0; pulse < num_pulses; pulse++) {
        TEST_ASSERT(allocate_signal_buffer(&pulse_train[pulse], samples_per_pulse, sample_rate));
        
        // Add Doppler phase shift between pulses
        double pulse_phase = phase_increment * pulse;
        
        for (size_t sample = 0; sample < samples_per_pulse; sample++) {
            double t = (double)sample / sample_rate;
            pulse_train[pulse].samples[sample].real = cos(2.0 * M_PI * 1e6 * t + pulse_phase);
            pulse_train[pulse].samples[sample].imag = sin(2.0 * M_PI * 1e6 * t + pulse_phase);
        }
    }
    
    // Initialize Doppler parameters
    DopplerParams params;
    TEST_ASSERT(doppler_params_init(&params, carrier_freq, prf, num_pulses));
    
    // Perform Doppler analysis
    DopplerAnalysis analysis;
    TEST_ASSERT(analyze_doppler(pulse_train, num_pulses, &params, &analysis));
    
    // Verify analysis results
    TEST_ASSERT(analysis.spectrum_length > 0);
    TEST_ASSERT_NOT_NULL(analysis.velocity_bins);
    TEST_ASSERT_NOT_NULL(analysis.doppler_spectrum);
    TEST_ASSERT_NOT_NULL(analysis.doppler_phase);
    TEST_ASSERT(analysis.peak_magnitude > 0.0);
    TEST_ASSERT(!analysis.velocity_ambiguous);
    
    // Check that detected velocity is close to expected
    TEST_ASSERT_NEAR(analysis.peak_velocity, target_velocity, 5.0); // 5 m/s tolerance
    
    // Test velocity bin conversion functions
    uint32_t velocity_bin = velocity_value_to_bin(target_velocity, &params);
    double converted_velocity = velocity_bin_to_value(velocity_bin, &params);
    TEST_ASSERT_NEAR(converted_velocity, target_velocity, params.velocity_resolution);
    
    // Test Doppler processing gain calculation
    double processing_gain = calculate_doppler_processing_gain(num_pulses);
    double expected_gain = 10.0 * log10((double)num_pulses);
    TEST_ASSERT_NEAR(processing_gain, expected_gain, FLOAT_TOLERANCE);
    
    // Cleanup
    for (uint32_t i = 0; i < num_pulses; i++) {
        free_signal_buffer(&pulse_train[i]);
    }
    free(pulse_train);
    doppler_analysis_cleanup(&analysis);
    
    return true;
}

// ============================================================================
// MTI FILTERING TESTS
// ============================================================================

bool test_mti_filtering(void) {
    const uint32_t filter_order = 3;
    const double prf = 1000.0;
    const uint32_t num_pulses = 10;
    const size_t samples_per_pulse = 100;
    
    MTIFilter mti;
    TEST_ASSERT(mti_filter_init(&mti, filter_order, prf));
    TEST_ASSERT(mti.is_initialized);
    TEST_ASSERT(mti.filter_order == filter_order);
    TEST_ASSERT_NOT_NULL(mti.filter_coeffs);
    TEST_ASSERT_NOT_NULL(mti.delay_line);
    TEST_ASSERT(mti.improvement_factor_db > 0.0);
    
    // Create test pulse train with stationary clutter and moving target
    SignalBuffer* input_pulses = calloc(num_pulses, sizeof(SignalBuffer));
    SignalBuffer* output_pulses = calloc(num_pulses, sizeof(SignalBuffer));
    TEST_ASSERT_NOT_NULL(input_pulses);
    TEST_ASSERT_NOT_NULL(output_pulses);
    
    const double clutter_amplitude = 10.0;  // Strong clutter
    const double target_amplitude = 1.0;    // Weak target
    const double target_velocity = 100.0;   // Moving target
    
    for (uint32_t pulse = 0; pulse < num_pulses; pulse++) {
        TEST_ASSERT(allocate_signal_buffer(&input_pulses[pulse], samples_per_pulse, DEFAULT_SAMPLE_RATE));
        TEST_ASSERT(allocate_signal_buffer(&output_pulses[pulse], samples_per_pulse, DEFAULT_SAMPLE_RATE));
        
        // Add phase shift for moving target
        double target_phase = 2.0 * M_PI * target_velocity * pulse / (SPEED_OF_LIGHT / (2.0 * DEFAULT_RADAR_FREQUENCY));
        
        for (size_t sample = 0; sample < samples_per_pulse; sample++) {
            // Stationary clutter (no phase change between pulses)
            double clutter_real = clutter_amplitude;
            double clutter_imag = 0.0;
            
            // Moving target (phase changes between pulses)
            double target_real = target_amplitude * cos(target_phase);
            double target_imag = target_amplitude * sin(target_phase);
            
            input_pulses[pulse].samples[sample].real = clutter_real + target_real;
            input_pulses[pulse].samples[sample].imag = clutter_imag + target_imag;
        }
    }
    
    // Apply MTI filtering
    TEST_ASSERT(apply_mti_filter(&mti, input_pulses, output_pulses, num_pulses));
    
    // Verify clutter suppression
    // After MTI filtering, stationary clutter should be significantly reduced
    double input_power = 0.0, output_power = 0.0;
    
    for (uint32_t pulse = 0; pulse < num_pulses; pulse++) {
        for (size_t sample = 0; sample < samples_per_pulse; sample++) {
            input_power += input_pulses[pulse].samples[sample].real * input_pulses[pulse].samples[sample].real +
                          input_pulses[pulse].samples[sample].imag * input_pulses[pulse].samples[sample].imag;
            output_power += output_pulses[pulse].samples[sample].real * output_pulses[pulse].samples[sample].real +
                           output_pulses[pulse].samples[sample].imag * output_pulses[pulse].samples[sample].imag;
        }
    }
    
    // Output power should be significantly less than input power due to clutter suppression
    TEST_ASSERT(output_power < input_power);
    
    // Test MTI improvement factor calculation
    double improvement = calculate_mti_improvement(&mti, 0.0); // Stationary clutter
    TEST_ASSERT(improvement > 20.0); // Should provide good clutter suppression
    
    double moving_improvement = calculate_mti_improvement(&mti, target_velocity);
    TEST_ASSERT(moving_improvement < improvement); // Less suppression for moving targets
    
    // Cleanup
    for (uint32_t i = 0; i < num_pulses; i++) {
        free_signal_buffer(&input_pulses[i]);
        free_signal_buffer(&output_pulses[i]);
    }
    free(input_pulses);
    free(output_pulses);
    mti_filter_cleanup(&mti);
    
    return true;
}

// ============================================================================
// RANGE-DOPPLER PROCESSING TESTS
// ============================================================================

bool test_range_doppler_processing(void) {
    const uint32_t num_range_gates = 64;
    const uint32_t num_pulses = 32;
    const uint32_t range_fft_size = 128;
    const uint32_t doppler_fft_size = 64;
    
    RangeDopplerProcessor processor;
    TEST_ASSERT(range_doppler_init(&processor, num_range_gates, num_pulses,
                                  range_fft_size, doppler_fft_size));
    TEST_ASSERT(processor.is_initialized);
    TEST_ASSERT(processor.num_range_gates == num_range_gates);
    TEST_ASSERT(processor.num_pulses == num_pulses);
    TEST_ASSERT_NOT_NULL(processor.range_fft);
    TEST_ASSERT_NOT_NULL(processor.doppler_fft);
    
    // Create synthetic data with target at specific range and velocity
    const size_t target_range_gate = 20;
    const size_t target_doppler_bin = 10;
    const double target_amplitude = 1.0;
    
    ComplexSample** pulse_data = malloc(num_pulses * sizeof(ComplexSample*));
    TEST_ASSERT_NOT_NULL(pulse_data);
    
    for (uint32_t pulse = 0; pulse < num_pulses; pulse++) {
        pulse_data[pulse] = calloc(num_range_gates, sizeof(ComplexSample));
        TEST_ASSERT_NOT_NULL(pulse_data[pulse]);
        
        // Add noise
        for (uint32_t gate = 0; gate < num_range_gates; gate++) {
            pulse_data[pulse][gate] = generate_complex_noise(0.01);
        }
        
        // Add target signal with Doppler shift
        double doppler_phase = 2.0 * M_PI * target_doppler_bin * pulse / num_pulses;
        pulse_data[pulse][target_range_gate].real += target_amplitude * cos(doppler_phase);
        pulse_data[pulse][target_range_gate].imag += target_amplitude * sin(doppler_phase);
    }
    
    // Generate Range-Doppler map
    RangeDopplerMap rd_map;
    TEST_ASSERT(generate_range_doppler_map(&processor, (const ComplexSample**)pulse_data, &rd_map));
    
    // Verify Range-Doppler map structure
    TEST_ASSERT(rd_map.range_bins_count == num_range_gates);
    TEST_ASSERT(rd_map.doppler_bins_count == num_pulses);
    TEST_ASSERT_NOT_NULL(rd_map.magnitude_map);
    TEST_ASSERT_NOT_NULL(rd_map.phase_map);
    TEST_ASSERT_NOT_NULL(rd_map.range_bins);
    TEST_ASSERT_NOT_NULL(rd_map.doppler_bins);
    TEST_ASSERT(rd_map.range_resolution > 0.0);
    TEST_ASSERT(rd_map.doppler_resolution > 0.0);
    
    // Find peak in Range-Doppler map
    double peak_magnitude = 0.0;
    size_t peak_range_bin = 0, peak_doppler_bin = 0;
    
    for (size_t r = 0; r < rd_map.range_bins_count; r++) {
        for (size_t d = 0; d < rd_map.doppler_bins_count; d++) {
            if (rd_map.magnitude_map[r][d] > peak_magnitude) {
                peak_magnitude = rd_map.magnitude_map[r][d];
                peak_range_bin = r;
                peak_doppler_bin = d;
            }
        }
    }
    
    // Verify target detection
    TEST_ASSERT(peak_range_bin == target_range_gate || 
                peak_range_bin == target_range_gate + 1 || 
                peak_range_bin == target_range_gate - 1); // Allow for small errors
    
    // Test CFAR detection on Range-Doppler map
    DetectionResult detections[10];
    uint32_t num_detections = detect_targets_rd_map(&rd_map, 10.0, detections, 10);
    TEST_ASSERT(num_detections > 0);
    TEST_ASSERT(detections[0].is_valid);
    TEST_ASSERT(detections[0].detected_range >= 0.0);
    TEST_ASSERT(detections[0].snr_db > 0.0);
    
    // Cleanup
    for (uint32_t i = 0; i < num_pulses; i++) {
        free(pulse_data[i]);
    }
    free(pulse_data);
    range_doppler_map_cleanup(&rd_map);
    range_doppler_cleanup(&processor);
    
    return true;
}

// ============================================================================
// VELOCITY AMBIGUITY RESOLUTION TESTS
// ============================================================================

bool test_velocity_ambiguity_resolution(void) {
    const double carrier_freq = 10e9; // X-band
    const double true_velocity = 150.0; // m/s
    
    // Create measurements at different PRFs
    const double prfs[] = {1000.0, 1200.0, 1500.0};
    const size_t num_measurements = sizeof(prfs) / sizeof(prfs[0]);
    double measurements[3];
    
    // Calculate ambiguous measurements
    for (size_t i = 0; i < num_measurements; i++) {
        double max_unambig_vel = calculate_max_unambiguous_velocity(prfs[i], carrier_freq);
        measurements[i] = fmod(true_velocity, 2.0 * max_unambig_vel);
        if (measurements[i] > max_unambig_vel) {
            measurements[i] = 2.0 * max_unambig_vel - measurements[i];
        }
    }
    
    // Resolve velocity ambiguity
    double resolved_velocity;
    TEST_ASSERT(resolve_velocity_ambiguity(measurements, prfs, num_measurements,
                                          carrier_freq, &resolved_velocity));
    
    // Check that resolved velocity is close to true velocity
    TEST_ASSERT_NEAR(resolved_velocity, true_velocity, 5.0); // 5 m/s tolerance
    
    // Test with different true velocities
    const double test_velocities[] = {50.0, 100.0, 200.0, 350.0};
    const size_t num_test_velocities = sizeof(test_velocities) / sizeof(test_velocities[0]);
    
    for (size_t v = 0; v < num_test_velocities; v++) {
        double test_vel = test_velocities[v];
        
        // Generate ambiguous measurements
        for (size_t i = 0; i < num_measurements; i++) {
            double max_unambig_vel = calculate_max_unambiguous_velocity(prfs[i], carrier_freq);
            measurements[i] = fmod(test_vel, 2.0 * max_unambig_vel);
            if (measurements[i] > max_unambig_vel) {
                measurements[i] = 2.0 * max_unambig_vel - measurements[i];
            }
        }
        
        // Resolve ambiguity
        if (resolve_velocity_ambiguity(measurements, prfs, num_measurements,
                                     carrier_freq, &resolved_velocity)) {
            TEST_ASSERT(fabs(resolved_velocity - test_vel) < 10.0); // 10 m/s tolerance
        }
    }
    
    return true;
}

// ============================================================================
// PULSE-DOPPLER PROCESSING TESTS
// ============================================================================

bool test_pulse_doppler_processing(void) {
    const uint32_t num_pulses = 16;
    const double carrier_freq = 10e9;
    const double prf = 1000.0;
    const size_t samples_per_pulse = 256;
    const double sample_rate = 100e6;
    
    // Create received pulse train
    SignalBuffer* received_pulses = calloc(num_pulses, sizeof(SignalBuffer));
    TEST_ASSERT_NOT_NULL(received_pulses);
    
    // Create reference pulse (chirp)
    SignalBuffer reference_pulse;
    TEST_ASSERT(allocate_signal_buffer(&reference_pulse, samples_per_pulse, sample_rate));
    reference_pulse.center_frequency = carrier_freq;
    reference_pulse.bandwidth = 10e6; // 10 MHz bandwidth
    reference_pulse.waveform = WAVEFORM_CHIRP_LINEAR;
    TEST_ASSERT(generate_linear_chirp(&reference_pulse, 
                                     carrier_freq - 5e6, 
                                     carrier_freq + 5e6, 1.0));
    
    // Generate received pulses with target echoes
    const double target_velocity = 75.0; // m/s
    const double target_range = 5000.0;  // 5 km
    const size_t target_delay_samples = (size_t)(2.0 * target_range / SPEED_OF_LIGHT * sample_rate);
    
    for (uint32_t pulse = 0; pulse < num_pulses; pulse++) {
        TEST_ASSERT(allocate_signal_buffer(&received_pulses[pulse], samples_per_pulse, sample_rate));
        
        // Add noise
        TEST_ASSERT(add_awgn_to_signal(&received_pulses[pulse], 0.01));
        
        // Add target echo with Doppler shift
        double doppler_phase = 2.0 * M_PI * velocity_to_doppler(target_velocity, carrier_freq) * pulse / prf;
        
        if (target_delay_samples < samples_per_pulse) {
            for (size_t s = target_delay_samples; s < samples_per_pulse && s < target_delay_samples + 50; s++) {
                double echo_amplitude = 0.1; // Weak echo
                received_pulses[pulse].samples[s].real += echo_amplitude * cos(doppler_phase);
                received_pulses[pulse].samples[s].imag += echo_amplitude * sin(doppler_phase);
            }
        }
    }
    
    // Initialize Doppler parameters
    DopplerParams params;
    TEST_ASSERT(doppler_params_init(&params, carrier_freq, prf, num_pulses));
    params.enable_mti = true;
    params.mti_order = 2;
    params.clutter_threshold_db = 30.0;
    
    // Perform pulse-Doppler processing
    DopplerAnalysis analysis;
    TEST_ASSERT(pulse_doppler_processing(received_pulses, &reference_pulse,
                                        num_pulses, &params, &analysis));
    
    // Verify analysis results
    TEST_ASSERT(analysis.spectrum_length > 0);
    TEST_ASSERT_NOT_NULL(analysis.velocity_bins);
    TEST_ASSERT_NOT_NULL(analysis.doppler_spectrum);
    TEST_ASSERT(analysis.peak_magnitude > 0.0);
    TEST_ASSERT(analysis.snr_db > 0.0);
    
    // Check that peak velocity is reasonable
    TEST_ASSERT(fabs(analysis.peak_velocity) <= calculate_max_unambiguous_velocity(prf, carrier_freq));
    
    // Cleanup
    for (uint32_t i = 0; i < num_pulses; i++) {
        free_signal_buffer(&received_pulses[i]);
    }
    free(received_pulses);
    free_signal_buffer(&reference_pulse);
    doppler_analysis_cleanup(&analysis);
    
    return true;
}
