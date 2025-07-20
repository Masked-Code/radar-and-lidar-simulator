/**
 * @file signal_generator.h
 * @brief Signal generation functions for radar/LiDAR simulation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include "signal_types.h"
#include "target_simulator.h"

#ifdef __cplusplus
extern "C" {
#endif

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
bool generate_linear_chirp(SignalBuffer* signal, double start_freq, double end_freq, double amplitude);

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
                                  SignalBuffer* rx_signal, const PulseParams* pulse_params);

#ifdef __cplusplus
}
#endif

#endif // SIGNAL_GENERATOR_H
