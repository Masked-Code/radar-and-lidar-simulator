/**
 * @file range_detector.c
 * @brief Range detection and measurement implementation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include "range_detector.h"
#include "utils.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

bool range_params_init(RangeParams* params, double sample_rate, double pulse_width, double max_range) {
    if (!params || sample_rate <= 0.0 || pulse_width <= 0.0 || max_range <= 0.0) {
        return false;
    }

    memset(params, 0, sizeof(RangeParams));

    params->sample_rate = sample_rate;
    params->pulse_width = pulse_width;
    params->max_range = max_range;
    
    // Calculate range resolution based on pulse width
    params->range_resolution = SPEED_OF_LIGHT * pulse_width / 2.0;
    
    // Calculate number of range gates
    params->num_range_gates = (uint32_t)(2.0 * max_range * sample_rate / SPEED_OF_LIGHT);
    params->gate_spacing = SPEED_OF_LIGHT / (2.0 * sample_rate);
    
    params->detection_threshold_db = DEFAULT_DETECTION_THRESHOLD;
    params->enable_pulse_compression = false;
    params->enable_cfar = false;

    return true;
}

bool generate_range_profile(const SignalBuffer* rx_signal, const RangeParams* params,
                           RangeMeasurement* measurement) {
    if (!rx_signal || !params || !measurement) {
        return false;
    }

    // Allocate measurement arrays
    measurement->profile_length = params->num_range_gates;
    measurement->range_bins = calloc(measurement->profile_length, sizeof(double));
    measurement->range_profile = calloc(measurement->profile_length, sizeof(double));
    measurement->range_phase = calloc(measurement->profile_length, sizeof(double));

    if (!measurement->range_bins || !measurement->range_profile || !measurement->range_phase) {
        range_measurement_cleanup(measurement);
        return false;
    }

    // Generate range bins
    for (uint32_t i = 0; i < measurement->profile_length; i++) {
        measurement->range_bins[i] = i * params->gate_spacing;
    }

    // Calculate range profile (magnitude squared)
    uint32_t profile_length = (rx_signal->length < measurement->profile_length) ? 
                             rx_signal->length : measurement->profile_length;

    for (uint32_t i = 0; i < profile_length; i++) {
        double real = rx_signal->samples[i].real;
        double imag = rx_signal->samples[i].imag;
        measurement->range_profile[i] = real * real + imag * imag;
        measurement->range_phase[i] = atan2(imag, real);
    }

    // Find peak
    size_t peak_idx;
    measurement->peak_magnitude = find_array_max(measurement->range_profile, profile_length, &peak_idx);
    measurement->peak_bin = (uint32_t)peak_idx;
    measurement->detected_range = measurement->range_bins[peak_idx];

    return true;
}

double time_delay_to_range(double time_delay, bool is_two_way) {
    double factor = is_two_way ? 2.0 : 1.0;
    return SPEED_OF_LIGHT * time_delay / factor;
}

double range_to_time_delay(double range, bool is_two_way) {
    double factor = is_two_way ? 2.0 : 1.0;
    return range * factor / SPEED_OF_LIGHT;
}

bool pulse_compression_init(PulseCompressionFilter* filter, const SignalBuffer* reference_pulse,
                           uint32_t fft_size) {
    if (!filter || !reference_pulse || !reference_pulse->is_allocated) {
        return false;
    }

    memset(filter, 0, sizeof(PulseCompressionFilter));

    // Allocate memory for reference and compressed pulses
    filter->pulse_length = reference_pulse->length;
    filter->reference_pulse = calloc(filter->pulse_length, sizeof(ComplexSample));
    filter->compressed_pulse = calloc(fft_size, sizeof(ComplexSample));

    if (!filter->reference_pulse || !filter->compressed_pulse) {
        pulse_compression_cleanup(filter);
        return false;
    }

    // Copy and conjugate reference pulse for matched filtering
    for (uint32_t i = 0; i < filter->pulse_length; i++) {
        filter->reference_pulse[i].real = reference_pulse->samples[i].real;
        filter->reference_pulse[i].imag = -reference_pulse->samples[i].imag; // Conjugate
    }

    // Calculate compression ratio
    filter->compression_ratio = calculate_compression_ratio(
        reference_pulse->duration, reference_pulse->bandwidth);
    filter->processing_gain_db = calculate_compression_gain(filter->compression_ratio);

    filter->is_initialized = true;
    return true;
}

void pulse_compression_cleanup(PulseCompressionFilter* filter) {
    if (!filter) return;

    free(filter->reference_pulse);
    free(filter->compressed_pulse);
    memset(filter, 0, sizeof(PulseCompressionFilter));
}

bool apply_pulse_compression(PulseCompressionFilter* filter, const SignalBuffer* rx_signal,
                            SignalBuffer* compressed_signal) {
    if (!filter || !filter->is_initialized || !rx_signal || !compressed_signal) {
        return false;
    }

    // Allocate output signal
    if (!allocate_signal_buffer(compressed_signal, rx_signal->length, rx_signal->sample_rate)) {
        return false;
    }

    // Perform correlation (simplified implementation)
    for (size_t i = 0; i < rx_signal->length; i++) {
        ComplexSample sum = {0.0, 0.0};
        
        for (uint32_t j = 0; j < filter->pulse_length && (i + j) < rx_signal->length; j++) {
            ComplexSample prod = complex_multiply(rx_signal->samples[i + j], filter->reference_pulse[j]);
            sum = complex_add(sum, prod);
        }
        
        compressed_signal->samples[i] = sum;
    }

    return true;
}

double calculate_compression_ratio(double pulse_width, double bandwidth) {
    return pulse_width * bandwidth;
}

double calculate_compression_gain(double compression_ratio) {
    return 10.0 * log10(compression_ratio);
}

bool cfar_detector_init(CFARDetector* detector, uint32_t num_reference_cells,
                        uint32_t num_guard_cells, double false_alarm_rate, uint32_t detector_type) {
    if (!detector || num_reference_cells == 0) {
        return false;
    }

    memset(detector, 0, sizeof(CFARDetector));

    detector->reference_cells = calloc(num_reference_cells, sizeof(double));
    if (!detector->reference_cells) {
        return false;
    }

    detector->num_reference_cells = num_reference_cells;
    detector->num_guard_cells = num_guard_cells;
    detector->false_alarm_rate = false_alarm_rate;
    detector->detector_type = detector_type;
    
    // Calculate threshold factor
    detector->threshold_factor = calculate_cfar_threshold(false_alarm_rate, num_reference_cells, detector_type);
    
    detector->is_initialized = true;
    return true;
}

void cfar_detector_cleanup(CFARDetector* detector) {
    if (!detector) return;

    free(detector->reference_cells);
    memset(detector, 0, sizeof(CFARDetector));
}

uint32_t apply_cfar_detection(CFARDetector* detector, const double* range_profile,
                             uint32_t profile_length, uint8_t* detections, double* thresholds) {
    if (!detector || !detector->is_initialized || !range_profile || !detections || !thresholds) {
        return 0;
    }

    uint32_t detection_count = 0;
    uint32_t half_ref_cells = detector->num_reference_cells / 2;
    uint32_t half_window = half_ref_cells + detector->num_guard_cells;

    for (uint32_t i = 0; i < profile_length; i++) {
        detections[i] = 0;
        thresholds[i] = 0.0;

        // Skip cells too close to edges
        if (i < half_window || i >= (profile_length - half_window)) {
            continue;
        }

        // Calculate average of reference cells
        double ref_sum = 0.0;
        uint32_t ref_count = 0;

        // Leading reference cells
        for (uint32_t j = i - half_window - half_ref_cells; j < i - detector->num_guard_cells; j++) {
            if (j < profile_length) {
                ref_sum += range_profile[j];
                ref_count++;
            }
        }

        // Trailing reference cells  
        for (uint32_t j = i + detector->num_guard_cells + 1; j <= i + half_window; j++) {
            if (j < profile_length) {
                ref_sum += range_profile[j];
                ref_count++;
            }
        }

        if (ref_count > 0) {
            double average = ref_sum / ref_count;
            thresholds[i] = detector->threshold_factor * average;

            if (range_profile[i] > thresholds[i]) {
                detections[i] = 1;
                detection_count++;
            }
        }
    }

    return detection_count;
}

double calculate_cfar_threshold(double false_alarm_rate, uint32_t num_reference_cells, uint32_t detector_type) {
    if (false_alarm_rate <= 0.0 || false_alarm_rate >= 1.0 || num_reference_cells == 0) {
        return 1.0;
    }

    // CA-CFAR threshold calculation
    if (detector_type == 0) {
        return num_reference_cells * (pow(false_alarm_rate, -1.0/num_reference_cells) - 1.0);
    }

    // Default threshold
    return -log(false_alarm_rate);
}

double range_bin_to_value(uint32_t bin, const RangeParams* params) {
    if (!params) return 0.0;
    return bin * params->gate_spacing;
}

uint32_t range_value_to_bin(double range, const RangeParams* params) {
    if (!params || params->gate_spacing <= 0.0) return 0;
    return (uint32_t)(range / params->gate_spacing + 0.5);
}

double calculate_range_resolution(double bandwidth, bool is_two_way) {
    if (bandwidth <= 0.0) return 0.0;
    
    double factor = is_two_way ? 2.0 : 1.0;
    return SPEED_OF_LIGHT / (factor * bandwidth);
}

bool validate_range_parameters(const RangeParams* params) {
    if (!params) return false;
    
    return params->sample_rate > 0.0 &&
           params->pulse_width > 0.0 &&
           params->max_range > 0.0 &&
           params->num_range_gates > 0 &&
           params->gate_spacing > 0.0;
}

void range_measurement_cleanup(RangeMeasurement* measurement) {
    if (!measurement) return;

    free(measurement->range_bins);
    free(measurement->range_profile);
    free(measurement->range_phase);
    free(measurement->detection_ranges);
    free(measurement->detection_magnitudes);
    
    memset(measurement, 0, sizeof(RangeMeasurement));
}
