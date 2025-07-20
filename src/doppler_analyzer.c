/**
 * @file doppler_analyzer.c
 * @brief Doppler analysis and velocity estimation implementation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include "doppler_analyzer.h"
#include "utils.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Forward declaration for helper function
static void simple_fft_doppler(ComplexSample* data, int n);

bool doppler_params_init(DopplerParams* params, double carrier_freq, double prf, uint32_t num_pulses) {
    if (!params || carrier_freq <= 0.0 || prf <= 0.0 || num_pulses == 0) {
        return false;
    }

    memset(params, 0, sizeof(DopplerParams));

    params->carrier_frequency = carrier_freq;
    params->prf = prf;
    params->coherent_pulses = num_pulses;
    params->doppler_fft_size = get_next_fft_size(num_pulses);
    
    // Calculate maximum unambiguous velocity
    params->max_unambiguous_velocity = calculate_max_unambiguous_velocity(prf, carrier_freq);
    
    // Calculate velocity resolution
    params->velocity_resolution = params->max_unambiguous_velocity / (params->doppler_fft_size / 2);
    
    params->enable_mti = false;
    params->mti_order = 1;
    params->clutter_threshold_db = CLUTTER_REJECTION_THRESHOLD;

    return true;
}

bool analyze_doppler(const SignalBuffer* pulse_train, uint32_t num_pulses, 
                     const DopplerParams* params, DopplerAnalysis* analysis) {
    if (!pulse_train || !params || !analysis || num_pulses == 0) {
        return false;
    }

    memset(analysis, 0, sizeof(DopplerAnalysis));

    // Allocate analysis arrays
    analysis->spectrum_length = params->doppler_fft_size;
    analysis->velocity_bins = calloc(analysis->spectrum_length, sizeof(double));
    analysis->doppler_spectrum = calloc(analysis->spectrum_length, sizeof(double));
    analysis->doppler_phase = calloc(analysis->spectrum_length, sizeof(double));

    if (!analysis->velocity_bins || !analysis->doppler_spectrum || !analysis->doppler_phase) {
        doppler_analysis_cleanup(analysis);
        return false;
    }

    // Generate velocity bins
    for (uint32_t i = 0; i < analysis->spectrum_length; i++) {
        analysis->velocity_bins[i] = velocity_bin_to_value(i, params);
    }

    // Simple Doppler processing - take first sample from each pulse
    ComplexSample* doppler_data = calloc(params->doppler_fft_size, sizeof(ComplexSample));
    if (!doppler_data) {
        doppler_analysis_cleanup(analysis);
        return false;
    }

    // Extract coherent pulse data (simplified - takes first sample from each pulse)
    uint32_t samples_to_process = (num_pulses < params->doppler_fft_size) ? num_pulses : params->doppler_fft_size;
    
    for (uint32_t i = 0; i < samples_to_process; i++) {
        if (pulse_train[i].is_allocated && pulse_train[i].length > 0) {
            doppler_data[i] = pulse_train[i].samples[0];  // Use first sample from each pulse
        } else {
            doppler_data[i] = (ComplexSample){0.0, 0.0};
        }
    }

    // Zero pad remaining samples
    for (uint32_t i = samples_to_process; i < params->doppler_fft_size; i++) {
        doppler_data[i] = (ComplexSample){0.0, 0.0};
    }

    // Apply simple FFT for Doppler spectrum
    simple_fft_doppler(doppler_data, params->doppler_fft_size);

    // Calculate magnitude spectrum
    for (uint32_t i = 0; i < analysis->spectrum_length; i++) {
        double mag = complex_magnitude(doppler_data[i]);
        analysis->doppler_spectrum[i] = mag * mag;  // Power spectrum
        analysis->doppler_phase[i] = complex_phase(doppler_data[i]);
    }

    // Find peak velocity
    size_t peak_idx;
    analysis->peak_magnitude = find_array_max(analysis->doppler_spectrum, analysis->spectrum_length, &peak_idx);
    analysis->peak_bin = (uint32_t)peak_idx;
    analysis->peak_velocity = analysis->velocity_bins[peak_idx];

    // Calculate confidence based on peak-to-average ratio
    double average_power = calculate_mean(analysis->doppler_spectrum, analysis->spectrum_length);
    if (average_power > 0.0) {
        analysis->confidence = analysis->peak_magnitude / average_power;
        analysis->confidence = (analysis->confidence > 1.0) ? 1.0 : analysis->confidence;
    }

    // Estimate velocity spread (simplified)
    analysis->velocity_spread = params->velocity_resolution * 2.0;  // Rough estimate

    free(doppler_data);
    return true;
}

double velocity_to_doppler(double velocity, double carrier_frequency) {
    return 2.0 * velocity * carrier_frequency / SPEED_OF_LIGHT;
}

double doppler_to_velocity(double doppler_freq, double carrier_frequency) {
    if (carrier_frequency <= 0.0) return 0.0;
    return doppler_freq * SPEED_OF_LIGHT / (2.0 * carrier_frequency);
}

double calculate_max_unambiguous_velocity(double prf, double carrier_frequency) {
    if (prf <= 0.0 || carrier_frequency <= 0.0) return 0.0;
    
    double max_doppler_freq = prf / 2.0;  // Nyquist frequency
    return doppler_to_velocity(max_doppler_freq, carrier_frequency);
}

bool mti_filter_init(MTIFilter* mti, uint32_t filter_order, double prf) {
    if (!mti || filter_order == 0 || prf <= 0.0) {
        return false;
    }

    memset(mti, 0, sizeof(MTIFilter));

    mti->filter_order = filter_order;
    mti->delay_length = filter_order + 1;
    
    // Allocate filter coefficients and delay line
    mti->filter_coeffs = calloc(mti->delay_length, sizeof(double));
    mti->delay_line = calloc(mti->delay_length, sizeof(ComplexSample));

    if (!mti->filter_coeffs || !mti->delay_line) {
        mti_filter_cleanup(mti);
        return false;
    }

    // Set up simple MTI filter coefficients (difference filter)
    if (filter_order == 1) {
        mti->filter_coeffs[0] = 1.0;
        mti->filter_coeffs[1] = -1.0;
    } else if (filter_order == 2) {
        mti->filter_coeffs[0] = 1.0;
        mti->filter_coeffs[1] = -2.0;
        mti->filter_coeffs[2] = 1.0;
    } else {
        // Default to first-order difference
        mti->filter_coeffs[0] = 1.0;
        mti->filter_coeffs[1] = -1.0;
    }

    mti->improvement_factor_db = 20.0 * filter_order;  // Rough estimate
    mti->is_initialized = true;

    return true;
}

void mti_filter_cleanup(MTIFilter* mti) {
    if (!mti) return;

    free(mti->filter_coeffs);
    free(mti->delay_line);
    memset(mti, 0, sizeof(MTIFilter));
}

bool apply_mti_filter(MTIFilter* mti, const SignalBuffer* input_pulses, 
                      SignalBuffer* output_pulses, uint32_t num_pulses) {
    if (!mti || !mti->is_initialized || !input_pulses || !output_pulses || num_pulses == 0) {
        return false;
    }

    // For simplicity, assume all pulses have the same length
    size_t pulse_length = input_pulses[0].length;
    double sample_rate = input_pulses[0].sample_rate;

    // Process each pulse
    for (uint32_t p = 0; p < num_pulses; p++) {
        if (!allocate_signal_buffer(&output_pulses[p], pulse_length, sample_rate)) {
            return false;
        }

        // Process each sample in the pulse
        for (size_t s = 0; s < pulse_length; s++) {
            // Shift delay line
            for (uint32_t d = mti->delay_length - 1; d > 0; d--) {
                mti->delay_line[d] = mti->delay_line[d - 1];
            }
            
            // Add new sample
            if (input_pulses[p].is_allocated && s < input_pulses[p].length) {
                mti->delay_line[0] = input_pulses[p].samples[s];
            } else {
                mti->delay_line[0] = (ComplexSample){0.0, 0.0};
            }

            // Apply filter
            ComplexSample output = {0.0, 0.0};
            for (uint32_t d = 0; d < mti->delay_length; d++) {
                ComplexSample scaled = complex_scale(mti->delay_line[d], mti->filter_coeffs[d]);
                output = complex_add(output, scaled);
            }
            
            output_pulses[p].samples[s] = output;
        }
    }

    return true;
}

double calculate_mti_improvement(const MTIFilter* mti, double clutter_velocity) {
    if (!mti || !mti->is_initialized) return 0.0;
    
    // Simplified improvement calculation
    if (fabs(clutter_velocity) < 1.0) {  // Near-zero velocity clutter
        return mti->improvement_factor_db;
    }
    
    return mti->improvement_factor_db * 0.5;  // Reduced improvement for moving clutter
}

bool validate_doppler_parameters(const DopplerParams* params) {
    if (!params) return false;
    
    return params->carrier_frequency > 0.0 &&
           params->prf > 0.0 &&
           params->coherent_pulses > 0 &&
           params->doppler_fft_size > 0 &&
           params->max_unambiguous_velocity > 0.0;
}

double velocity_bin_to_value(uint32_t bin, const DopplerParams* params) {
    if (!params || params->doppler_fft_size == 0) return 0.0;
    
    double bin_spacing = 2.0 * params->max_unambiguous_velocity / params->doppler_fft_size;
    double velocity = (double)bin * bin_spacing - params->max_unambiguous_velocity;
    
    return velocity;
}

uint32_t velocity_value_to_bin(double velocity, const DopplerParams* params) {
    if (!params || params->doppler_fft_size == 0) return 0;
    
    double bin_spacing = 2.0 * params->max_unambiguous_velocity / params->doppler_fft_size;
    double normalized_vel = (velocity + params->max_unambiguous_velocity) / bin_spacing;
    
    return (uint32_t)(normalized_vel + 0.5);  // Round to nearest bin
}

double calculate_doppler_processing_gain(uint32_t num_pulses) {
    if (num_pulses == 0) return 0.0;
    return 10.0 * log10((double)num_pulses);
}

void doppler_analysis_cleanup(DopplerAnalysis* analysis) {
    if (!analysis) return;

    free(analysis->velocity_bins);
    free(analysis->doppler_spectrum);
    free(analysis->doppler_phase);
    
    memset(analysis, 0, sizeof(DopplerAnalysis));
}

void range_doppler_map_cleanup(RangeDopplerMap* rd_map) {
    if (!rd_map) return;

    if (rd_map->magnitude_map) {
        for (size_t i = 0; i < rd_map->range_bins_count; i++) {
            free(rd_map->magnitude_map[i]);
        }
        free(rd_map->magnitude_map);
    }
    
    if (rd_map->phase_map) {
        for (size_t i = 0; i < rd_map->range_bins_count; i++) {
            free(rd_map->phase_map[i]);
        }
        free(rd_map->phase_map);
    }
    
    free(rd_map->range_bins);
    free(rd_map->doppler_bins);
    
    memset(rd_map, 0, sizeof(RangeDopplerMap));
}

// Helper function for simple Doppler FFT
static void simple_fft_doppler(ComplexSample* data, int n) {
    // Bit-reversal permutation
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        
        if (i < j) {
            ComplexSample temp = data[i];
            data[i] = data[j];
            data[j] = temp;
        }
    }

    // Cooley-Tukey FFT
    for (int len = 2; len <= n; len <<= 1) {
        double angle = -2.0 * M_PI / len;
        ComplexSample wlen = {cos(angle), sin(angle)};
        
        for (int i = 0; i < n; i += len) {
            ComplexSample w = {1.0, 0.0};
            
            for (int j = 0; j < len / 2; j++) {
                ComplexSample u = data[i + j];
                ComplexSample v = complex_multiply(data[i + j + len / 2], w);
                
                data[i + j] = complex_add(u, v);
                data[i + j + len / 2] = complex_add(u, complex_scale(v, -1.0));
                
                w = complex_multiply(w, wlen);
            }
        }
    }
}

