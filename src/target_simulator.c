/**
 * @file target_simulator.c
 * @brief Target modeling and simulation implementation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include "target_simulator.h"
#include "utils.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

bool simulation_environment_init(SimulationEnvironment* env, uint32_t max_targets, double time_step) {
    if (!env || max_targets == 0 || time_step <= 0) {
        return false;
    }

    memset(env, 0, sizeof(SimulationEnvironment));
    
    env->targets = calloc(max_targets, sizeof(TargetParams));
    env->target_motions = calloc(max_targets, sizeof(TargetMotion));
    
    if (!env->targets || !env->target_motions) {
        free(env->targets);
        free(env->target_motions);
        return false;
    }
    
    env->max_targets = max_targets;
    env->target_count = 0;
    env->time_step = time_step;
    env->simulation_time = 0.0;
    env->enable_clutter = false;
    env->clutter_density = 0.0;
    env->atmospheric_loss_db = ATMOSPHERIC_LOSS_X_BAND;
    env->rain_rate_mm_hr = 0.0;
    env->temperature_kelvin = STANDARD_TEMPERATURE;
    env->humidity_percent = 50.0;
    
    return true;
}

void simulation_environment_cleanup(SimulationEnvironment* env) {
    if (!env) return;
    
    free(env->targets);
    free(env->target_motions);
    memset(env, 0, sizeof(SimulationEnvironment));
}

uint32_t add_target(SimulationEnvironment* env, const TargetParams* target_params, 
                   const TargetMotion* motion_params) {
    if (!env || !target_params || env->target_count >= env->max_targets) {
        return 0;
    }
    
    uint32_t target_idx = env->target_count;
    env->targets[target_idx] = *target_params;
    
    if (motion_params) {
        env->target_motions[target_idx] = *motion_params;
    } else {
        memset(&env->target_motions[target_idx], 0, sizeof(TargetMotion));
    }
    
    env->target_count++;
    return target_params->id;
}

bool remove_target(SimulationEnvironment* env, uint32_t target_id) {
    if (!env) return false;
    
    for (uint32_t i = 0; i < env->target_count; i++) {
        if (env->targets[i].id == target_id) {
            // Shift remaining targets down
            for (uint32_t j = i; j < env->target_count - 1; j++) {
                env->targets[j] = env->targets[j + 1];
                env->target_motions[j] = env->target_motions[j + 1];
            }
            env->target_count--;
            return true;
        }
    }
    return false;
}

bool update_target(SimulationEnvironment* env, uint32_t target_id, const TargetParams* new_params) {
    if (!env || !new_params) return false;
    
    for (uint32_t i = 0; i < env->target_count; i++) {
        if (env->targets[i].id == target_id) {
            env->targets[i] = *new_params;
            return true;
        }
    }
    return false;
}

bool get_target(const SimulationEnvironment* env, uint32_t target_id, TargetParams* target_params) {
    if (!env || !target_params) return false;
    
    for (uint32_t i = 0; i < env->target_count; i++) {
        if (env->targets[i].id == target_id) {
            *target_params = env->targets[i];
            return true;
        }
    }
    return false;
}

bool generate_target_echo(const SimulationEnvironment* env, uint32_t target_id,
                         const SignalBuffer* tx_signal, SignalBuffer* rx_signal,
                         const PulseParams* pulse_params) {
    if (!env || !tx_signal || !rx_signal || !pulse_params) {
        return false;
    }

    TargetParams target;
    if (!get_target(env, target_id, &target)) {
        return false;
    }

    if (!allocate_signal_buffer(rx_signal, tx_signal->length, tx_signal->sample_rate)) {
        return false;
    }

    // Calculate propagation delay
    double range = target.range;
    double delay = 2.0 * range / SPEED_OF_LIGHT;
    size_t delay_samples = (size_t)(delay * tx_signal->sample_rate);

    // Calculate attenuation
    double path_loss = calculate_path_loss(range, pulse_params->carrier_frequency);
    double attenuation = sqrt(target.rcs) * DB_TO_LINEAR(-path_loss);

    // Initialize received signal
    for (size_t i = 0; i < rx_signal->length; i++) {
        rx_signal->samples[i] = (ComplexSample){0.0, 0.0};
    }

    // Add delayed and attenuated echo
    for (size_t i = 0; i < tx_signal->length && (i + delay_samples) < rx_signal->length; i++) {
        size_t rx_idx = i + delay_samples;
        rx_signal->samples[rx_idx].real = attenuation * tx_signal->samples[i].real;
        rx_signal->samples[rx_idx].imag = attenuation * tx_signal->samples[i].imag;
    }

    return true;
}

double calculate_path_loss(double range, double frequency) {
    if (range <= 0.0 || frequency <= 0.0) {
        return 0.0;
    }
    
    double lambda = SPEED_OF_LIGHT / frequency;
    double path_loss_db = 20.0 * log10(4.0 * M_PI * range / lambda);
    return path_loss_db;
}

double calculate_atmospheric_attenuation(double range, double frequency, double temperature,
                                        double humidity, double pressure) {
    // Simplified atmospheric attenuation model
    double freq_ghz = frequency / 1e9;
    double range_km = range / 1000.0;
    
    // Basic frequency-dependent attenuation
    double attenuation_db_per_km = 0.01 * freq_ghz * freq_ghz / 100.0;
    
    // Humidity effect (simplified)
    double humidity_factor = 1.0 + 0.01 * humidity;
    
    return attenuation_db_per_km * range_km * humidity_factor;
}

double calculate_rain_attenuation(double range, double frequency, double rain_rate) {
    if (rain_rate <= 0.0 || range <= 0.0) {
        return 0.0;
    }
    
    double freq_ghz = frequency / 1e9;
    double range_km = range / 1000.0;
    
    // ITU-R P.838 model (simplified)
    double a = 0.0001 * pow(freq_ghz, 1.6);
    double b = 1.1;
    
    double specific_attenuation = a * pow(rain_rate, b);
    return specific_attenuation * range_km;
}

double calculate_total_propagation_loss(double range, double frequency, 
                                       const SimulationEnvironment* env) {
    double free_space_loss = calculate_path_loss(range, frequency);
    double atmospheric_loss = calculate_atmospheric_attenuation(range, frequency,
        env->temperature_kelvin, env->humidity_percent / 100.0, STANDARD_PRESSURE);
    double rain_loss = calculate_rain_attenuation(range, frequency, env->rain_rate_mm_hr);
    
    return free_space_loss + atmospheric_loss + rain_loss;
}

bool update_target_motion(TargetMotion* motion, double time_step) {
    if (!motion || time_step <= 0.0) {
        return false;
    }

    // Update position with velocity and acceleration
    for (int i = 0; i < 3; i++) {
        motion->position[i] += motion->velocity[i] * time_step + 
                              0.5 * motion->acceleration[i] * time_step * time_step;
        motion->velocity[i] += motion->acceleration[i] * time_step;
    }

    motion->trajectory_time += time_step;
    return true;
}

double calculate_radial_velocity(const double target_position[3], const double target_velocity[3],
                                const double radar_position[3]) {
    if (!target_position || !target_velocity || !radar_position) {
        return 0.0;
    }

    // Calculate range vector from radar to target
    double range_vector[3];
    double range = 0.0;
    
    for (int i = 0; i < 3; i++) {
        range_vector[i] = target_position[i] - radar_position[i];
        range += range_vector[i] * range_vector[i];
    }
    
    range = sqrt(range);
    if (range == 0.0) return 0.0;

    // Normalize range vector
    for (int i = 0; i < 3; i++) {
        range_vector[i] /= range;
    }

    // Calculate radial component of velocity
    double radial_velocity = 0.0;
    for (int i = 0; i < 3; i++) {
        radial_velocity += target_velocity[i] * range_vector[i];
    }

    return -radial_velocity; // Negative for approaching targets
}

double calculate_target_range(const double target_position[3], const double radar_position[3]) {
    if (!target_position || !radar_position) {
        return 0.0;
    }

    double range_squared = 0.0;
    for (int i = 0; i < 3; i++) {
        double diff = target_position[i] - radar_position[i];
        range_squared += diff * diff;
    }
    
    return sqrt(range_squared);
}

bool calculate_target_angles(const double target_position[3], const double radar_position[3],
                            double* azimuth, double* elevation) {
    if (!target_position || !radar_position || !azimuth || !elevation) {
        return false;
    }

    double dx = target_position[0] - radar_position[0];
    double dy = target_position[1] - radar_position[1];
    double dz = target_position[2] - radar_position[2];
    
    double horizontal_range = sqrt(dx * dx + dy * dy);
    
    *azimuth = atan2(dy, dx);
    *elevation = atan2(dz, horizontal_range);
    
    return true;
}

bool validate_target_parameters(const TargetParams* params) {
    if (!params) return false;
    
    return params->range >= 0.0 && 
           params->range <= MAX_SIMULATION_RANGE &&
           fabs(params->velocity) <= MAX_TARGET_VELOCITY &&
           params->rcs >= 0.0 &&
           params->reflectivity >= 0.0 && 
           params->reflectivity <= 1.0;
}

void print_simulation_status(const SimulationEnvironment* env) {
    if (!env) return;
    
    printf("Simulation Environment Status:\n");
    printf("  Active targets: %u/%u\n", env->target_count, env->max_targets);
    printf("  Simulation time: %.3f s\n", env->simulation_time);
    printf("  Time step: %.6f s\n", env->time_step);
    printf("  Temperature: %.1f K\n", env->temperature_kelvin);
    printf("  Humidity: %.1f%%\n", env->humidity_percent);
    printf("  Clutter enabled: %s\n", env->enable_clutter ? "Yes" : "No");
}

bool get_simulation_statistics(const SimulationEnvironment* env, uint32_t* total_targets,
                              uint32_t* active_targets, double* avg_range, double* avg_velocity) {
    if (!env || !total_targets || !active_targets || !avg_range || !avg_velocity) {
        return false;
    }

    *total_targets = env->target_count;
    *active_targets = 0;
    *avg_range = 0.0;
    *avg_velocity = 0.0;

    for (uint32_t i = 0; i < env->target_count; i++) {
        if (env->targets[i].is_active) {
            (*active_targets)++;
            *avg_range += env->targets[i].range;
            *avg_velocity += env->targets[i].velocity;
        }
    }

    if (*active_targets > 0) {
        *avg_range /= *active_targets;
        *avg_velocity /= *active_targets;
    }

    return true;
}
