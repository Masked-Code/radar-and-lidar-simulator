#include "signal_types.h"
#include "radar_config.h"
#include "target_simulator.h"
#include "utils.h"
#include <math.h>
#include <stdlib.h>

/**
 * @brief Generate a linear chirp signal
 *
 * This function generates a linear frequency modulated (LFM) chirp signal
 * and stores it in the provided SignalBuffer.
 *
 * @param signal Output signal buffer
 * @param start_freq Start frequency of the chirp (Hz)
 * @param end_freq End frequency of the chirp (Hz)
 * @param amplitude Amplitude of the chirp
 * @return true if the chirp is successfully generated, false otherwise
 */
bool generate_linear_chirp(SignalBuffer* signal, double start_freq, double end_freq, double amplitude) {
    if (!signal || !signal->is_allocated) {
        return false;
    }
    double t, delta_f = (end_freq - start_freq) / signal->length;
    for (size_t i = 0; i < signal->length; ++i) {
        t = (double)i / signal->sample_rate;
        // double freq = start_freq + delta_f * i;  // Unused variable
        double phase = 2.0 * M_PI * (start_freq * t + 0.5 * delta_f * t * t);
        signal->samples[i].real = amplitude * cos(phase);
        signal->samples[i].imag = amplitude * sin(phase);
    }
    return true;
}

/**
 * @brief Generate a multi-target signal
 *
 * This function generates a composite signal that includes reflections
 * from multiple targets in the simulation environment.
 *
 * @param env Pointer to the simulation environment
 * @param tx_signal Transmitted signal buffer
 * @param rx_signal Output received signal buffer
 * @param pulse_params Radar pulse parameters
 * @return true if the multi-target signal is successfully generated, false otherwise
 */
bool generate_multi_target_signal(const SimulationEnvironment* env, const SignalBuffer* tx_signal,
                                  SignalBuffer* rx_signal, const PulseParams* pulse_params) {
    if (!env || !tx_signal || !rx_signal || !pulse_params) {
        return false;
    }
    
    // Allocate the received signal buffer
    if (!allocate_signal_buffer(rx_signal, tx_signal->length, tx_signal->sample_rate)) {
        return false;
    }
    
    // Initialize received signal to zero
    for (size_t i = 0; i < rx_signal->length; ++i) {
        rx_signal->samples[i] = (ComplexSample){0.0, 0.0};
    }
    
    // Accumulate signal contributions from each target
    for (uint32_t target_idx = 0; target_idx < env->target_count; ++target_idx) {
        const TargetParams* target = &env->targets[target_idx];
        if (!target->is_active) {
            continue;
        }

        // Generate the target echo and add to the received signal
        double range = target->range;
        double delay = 2.0 * range / SPEED_OF_LIGHT;
        size_t delay_samples = (size_t)(delay * tx_signal->sample_rate);
        double attenuation = DB_TO_LINEAR(-calculate_path_loss(range, pulse_params->carrier_frequency));

        if (delay_samples < tx_signal->length) {
            for (size_t i = 0; i < tx_signal->length - delay_samples; ++i) {
                size_t rx_idx = i + delay_samples;
                if (rx_idx < rx_signal->length) {
                    rx_signal->samples[rx_idx].real += attenuation * tx_signal->samples[i].real;
                    rx_signal->samples[rx_idx].imag += attenuation * tx_signal->samples[i].imag;
                }
            }
        }
    }
    return true;
}

