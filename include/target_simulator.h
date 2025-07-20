/**
 * @file target_simulator.h
 * @brief Target modeling and simulation functions for radar/LiDAR systems
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This header provides functions for simulating radar and LiDAR targets,
 * including multi-target scenarios, motion modeling, and signal return
 * generation with realistic physics and propagation effects.
 */

#ifndef TARGET_SIMULATOR_H
#define TARGET_SIMULATOR_H

#include "signal_types.h"
#include "radar_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// TARGET SIMULATION STRUCTURES
// ============================================================================

/**
 * @brief Target motion model structure
 * 
 * Defines the motion characteristics and trajectory of a simulated target
 * over time, including position, velocity, and acceleration components.
 */
typedef struct {
    double position[3];         /**< Position vector [x, y, z] in meters */
    double velocity[3];         /**< Velocity vector [vx, vy, vz] in m/s */
    double acceleration[3];     /**< Acceleration vector [ax, ay, az] in m/s² */
    double angular_position[3]; /**< Angular position [roll, pitch, yaw] in radians */
    double angular_velocity[3]; /**< Angular velocity [wx, wy, wz] in rad/s */
    double trajectory_time;     /**< Time along trajectory in seconds */
    bool is_maneuvering;        /**< Flag indicating if target is maneuvering */
} TargetMotion;

/**
 * @brief Multi-target simulation environment
 * 
 * Contains all targets and environmental parameters for a complete
 * radar/LiDAR simulation scenario.
 */
typedef struct {
    TargetParams* targets;          /**< Array of target parameters */
    TargetMotion* target_motions;   /**< Array of target motion models */
    uint32_t target_count;          /**< Number of active targets */
    uint32_t max_targets;           /**< Maximum number of targets */
    double simulation_time;         /**< Current simulation time */
    double time_step;               /**< Simulation time step */
    bool enable_clutter;            /**< Enable clutter simulation */
    double clutter_density;         /**< Clutter density (targets/km²) */
    double atmospheric_loss_db;     /**< Atmospheric loss in dB/km */
    double rain_rate_mm_hr;         /**< Rain rate in mm/hr */
    double temperature_kelvin;      /**< Environmental temperature */
    double humidity_percent;        /**< Relative humidity percentage */
} SimulationEnvironment;

/**
 * @brief Radar cross section model
 * 
 * Defines the RCS characteristics of a target as a function of
 * aspect angle and frequency.
 */
typedef struct {
    double* azimuth_angles;     /**< Azimuth angle samples (radians) */
    double* elevation_angles;   /**< Elevation angle samples (radians) */
    double* frequencies;        /**< Frequency samples (Hz) */
    double*** rcs_pattern;      /**< 3D RCS pattern [freq][az][el] (dBsm) */
    uint32_t azimuth_samples;   /**< Number of azimuth samples */
    uint32_t elevation_samples; /**< Number of elevation samples */
    uint32_t frequency_samples; /**< Number of frequency samples */
    double mean_rcs_dbsm;       /**< Mean RCS in dBsm */
    double rcs_fluctuation_std; /**< RCS fluctuation standard deviation (dB) */
} RCSModel;

/**
 * @brief Clutter model parameters
 * 
 * Defines characteristics of ground clutter, weather clutter,
 * and other interfering signals in the simulation environment.
 */
typedef struct {
    double clutter_power_dbm;       /**< Average clutter power (dBm) */
    double clutter_velocity_spread; /**< Clutter velocity spread (m/s) */
    double correlation_time;        /**< Clutter correlation time (s) */
    double spatial_correlation;     /**< Spatial correlation coefficient */
    bool enable_weather_clutter;    /**< Enable weather clutter */
    double weather_reflectivity;    /**< Weather reflectivity factor (dBZ) */
    double wind_velocity;           /**< Wind velocity for weather motion (m/s) */
} ClutterModel;

// ============================================================================
// TARGET MANAGEMENT FUNCTIONS
// ============================================================================

/**
 * @brief Initialize simulation environment with specified parameters
 * 
 * Sets up the simulation environment structure and allocates memory
 * for the specified number of targets.
 * 
 * @param env Pointer to simulation environment structure
 * @param max_targets Maximum number of targets to support
 * @param time_step Simulation time step in seconds
 * @return true if initialization successful, false otherwise
 */
bool simulation_environment_init(SimulationEnvironment* env, uint32_t max_targets, double time_step);

/**
 * @brief Cleanup simulation environment and free resources
 * 
 * @param env Pointer to simulation environment structure
 */
void simulation_environment_cleanup(SimulationEnvironment* env);

/**
 * @brief Add target to simulation environment
 * 
 * Adds a new target with specified parameters to the simulation.
 * The target is assigned a unique ID and added to the active target list.
 * 
 * @param env Pointer to simulation environment
 * @param target_params Target parameters structure
 * @param motion_params Initial target motion parameters
 * @return Target ID if successful, 0 if failed
 */
uint32_t add_target(SimulationEnvironment* env, const TargetParams* target_params, 
                   const TargetMotion* motion_params);

/**
 * @brief Remove target from simulation
 * 
 * @param env Pointer to simulation environment
 * @param target_id Target ID to remove
 * @return true if target removed successfully, false otherwise
 */
bool remove_target(SimulationEnvironment* env, uint32_t target_id);

/**
 * @brief Update target parameters during simulation
 * 
 * @param env Pointer to simulation environment
 * @param target_id Target ID to update
 * @param new_params Updated target parameters
 * @return true if update successful, false otherwise
 */
bool update_target(SimulationEnvironment* env, uint32_t target_id, const TargetParams* new_params);

/**
 * @brief Get target parameters by ID
 * 
 * @param env Pointer to simulation environment
 * @param target_id Target ID to retrieve
 * @param target_params Output target parameters
 * @return true if target found, false otherwise
 */
bool get_target(const SimulationEnvironment* env, uint32_t target_id, TargetParams* target_params);

// ============================================================================
// SIGNAL RETURN GENERATION
// ============================================================================

/**
 * @brief Generate target echo signal for radar pulse
 * 
 * Simulates the echo signal returned from a target when illuminated
 * by a radar pulse, including range delay, Doppler shift, and attenuation.
 * 
 * @param env Pointer to simulation environment
 * @param target_id Target ID
 * @param tx_signal Transmitted signal
 * @param rx_signal Output received echo signal
 * @param pulse_params Radar pulse parameters
 * @return true if signal generation successful, false otherwise
 */
bool generate_target_echo(const SimulationEnvironment* env, uint32_t target_id,
                         const SignalBuffer* tx_signal, SignalBuffer* rx_signal,
                         const PulseParams* pulse_params);

/**
 * @brief Generate multi-target composite signal
 * 
 * Creates a composite received signal containing echoes from all active
 * targets in the simulation environment.
 * 
 * @param env Pointer to simulation environment
 * @param tx_signal Transmitted signal
 * @param rx_signal Output composite received signal
 * @param pulse_params Radar pulse parameters
 * @return true if signal generation successful, false otherwise
 */
bool generate_multi_target_signal(const SimulationEnvironment* env, const SignalBuffer* tx_signal,
                                 SignalBuffer* rx_signal, const PulseParams* pulse_params);

/**
 * @brief Add clutter to received signal
 * 
 * Adds simulated clutter (ground, weather, etc.) to the received signal
 * based on the clutter model parameters.
 * 
 * @param env Pointer to simulation environment
 * @param rx_signal Signal buffer to add clutter to
 * @param clutter_model Clutter model parameters
 * @return true if clutter addition successful, false otherwise
 */
bool add_clutter_signal(const SimulationEnvironment* env, SignalBuffer* rx_signal,
                       const ClutterModel* clutter_model);

/**
 * @brief Add thermal noise to signal
 * 
 * Adds realistic thermal noise to the signal based on system noise
 * temperature and bandwidth.
 * 
 * @param signal Signal buffer to add noise to
 * @param noise_power_dbm Noise power in dBm
 * @param bandwidth Signal bandwidth in Hz
 * @return true if noise addition successful, false otherwise
 */
bool add_thermal_noise(SignalBuffer* signal, double noise_power_dbm, double bandwidth);

// ============================================================================
// PROPAGATION MODELING
// ============================================================================

/**
 * @brief Calculate free space path loss
 * 
 * Computes the free space path loss between radar and target
 * using the standard radar equation.
 * 
 * @param range Range to target in meters
 * @param frequency Operating frequency in Hz
 * @return Path loss in dB
 */
double calculate_path_loss(double range, double frequency);

/**
 * @brief Calculate atmospheric attenuation
 * 
 * Computes atmospheric attenuation due to absorption and scattering
 * based on frequency, range, and atmospheric conditions.
 * 
 * @param range Range to target in meters
 * @param frequency Operating frequency in Hz
 * @param temperature Temperature in Kelvin
 * @param humidity Relative humidity (0.0 to 1.0)
 * @param pressure Atmospheric pressure in Pa
 * @return Atmospheric attenuation in dB
 */
double calculate_atmospheric_attenuation(double range, double frequency, double temperature,
                                        double humidity, double pressure);

/**
 * @brief Calculate rain attenuation
 * 
 * Computes signal attenuation due to rain based on ITU-R models.
 * 
 * @param range Range through rain in meters
 * @param frequency Operating frequency in Hz
 * @param rain_rate Rain rate in mm/hr
 * @return Rain attenuation in dB
 */
double calculate_rain_attenuation(double range, double frequency, double rain_rate);

/**
 * @brief Calculate two-way propagation loss
 * 
 * Computes total two-way propagation loss including free space,
 * atmospheric, and weather effects.
 * 
 * @param range Range to target in meters
 * @param frequency Operating frequency in Hz
 * @param env Simulation environment parameters
 * @return Total propagation loss in dB
 */
double calculate_total_propagation_loss(double range, double frequency, 
                                       const SimulationEnvironment* env);

// ============================================================================
// TARGET MOTION MODELING
// ============================================================================

/**
 * @brief Update target motion for next time step
 * 
 * Updates target position, velocity, and acceleration based on the
 * motion model and simulation time step.
 * 
 * @param motion Pointer to target motion structure
 * @param time_step Simulation time step in seconds
 * @return true if update successful, false otherwise
 */
bool update_target_motion(TargetMotion* motion, double time_step);

/**
 * @brief Calculate radial velocity component
 * 
 * Computes the radial velocity component of the target relative
 * to the radar position.
 * 
 * @param target_position Target position vector [x, y, z]
 * @param target_velocity Target velocity vector [vx, vy, vz]
 * @param radar_position Radar position vector [x, y, z]
 * @return Radial velocity in m/s (positive = approaching)
 */
double calculate_radial_velocity(const double target_position[3], const double target_velocity[3],
                                const double radar_position[3]);

/**
 * @brief Calculate range to target
 * 
 * @param target_position Target position vector [x, y, z]
 * @param radar_position Radar position vector [x, y, z]
 * @return Range in meters
 */
double calculate_target_range(const double target_position[3], const double radar_position[3]);

/**
 * @brief Calculate target aspect angles
 * 
 * Computes azimuth and elevation angles from radar to target.
 * 
 * @param target_position Target position vector [x, y, z]
 * @param radar_position Radar position vector [x, y, z]
 * @param azimuth Output azimuth angle in radians
 * @param elevation Output elevation angle in radians
 * @return true if calculation successful, false otherwise
 */
bool calculate_target_angles(const double target_position[3], const double radar_position[3],
                            double* azimuth, double* elevation);

// ============================================================================
// RCS MODELING
// ============================================================================

/**
 * @brief Initialize RCS model with pattern data
 * 
 * @param rcs_model Pointer to RCS model structure
 * @param azimuth_samples Number of azimuth angle samples
 * @param elevation_samples Number of elevation angle samples
 * @param frequency_samples Number of frequency samples
 * @return true if initialization successful, false otherwise
 */
bool rcs_model_init(RCSModel* rcs_model, uint32_t azimuth_samples, 
                   uint32_t elevation_samples, uint32_t frequency_samples);

/**
 * @brief Cleanup RCS model and free memory
 * 
 * @param rcs_model Pointer to RCS model structure
 */
void rcs_model_cleanup(RCSModel* rcs_model);

/**
 * @brief Get target RCS at specific aspect angle and frequency
 * 
 * Interpolates RCS pattern data to get RCS value at the specified
 * aspect angle and frequency.
 * 
 * @param rcs_model Pointer to RCS model
 * @param azimuth Azimuth angle in radians
 * @param elevation Elevation angle in radians
 * @param frequency Operating frequency in Hz
 * @return RCS value in square meters
 */
double get_target_rcs(const RCSModel* rcs_model, double azimuth, double elevation, double frequency);

/**
 * @brief Generate RCS fluctuation based on Swerling model
 * 
 * Generates RCS fluctuation according to specified Swerling model
 * (I, II, III, or IV) for realistic target modeling.
 * 
 * @param base_rcs Base RCS value in square meters
 * @param swerling_model Swerling model type (1-4)
 * @param correlation_time Correlation time in seconds
 * @param current_time Current simulation time in seconds
 * @return Fluctuating RCS value in square meters
 */
double generate_rcs_fluctuation(double base_rcs, int swerling_model, double correlation_time, 
                               double current_time);

// ============================================================================
// SCENARIO GENERATION
// ============================================================================

/**
 * @brief Generate random target scenario
 * 
 * Creates a scenario with randomly distributed targets within specified
 * range and velocity bounds for testing purposes.
 * 
 * @param env Pointer to simulation environment
 * @param num_targets Number of targets to generate
 * @param min_range Minimum target range in meters
 * @param max_range Maximum target range in meters
 * @param max_velocity Maximum target velocity in m/s
 * @return true if scenario generation successful, false otherwise
 */
bool generate_random_scenario(SimulationEnvironment* env, uint32_t num_targets,
                             double min_range, double max_range, double max_velocity);

/**
 * @brief Load target scenario from configuration file
 * 
 * @param env Pointer to simulation environment
 * @param filename Path to scenario configuration file
 * @return true if scenario loaded successfully, false otherwise
 */
bool load_scenario_from_file(SimulationEnvironment* env, const char* filename);

/**
 * @brief Save current scenario to configuration file
 * 
 * @param env Pointer to simulation environment
 * @param filename Path to output scenario file
 * @return true if scenario saved successfully, false otherwise
 */
bool save_scenario_to_file(const SimulationEnvironment* env, const char* filename);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Validate target parameters
 * 
 * @param params Target parameters to validate
 * @return true if parameters are valid, false otherwise
 */
bool validate_target_parameters(const TargetParams* params);

/**
 * @brief Print simulation environment status
 * 
 * @param env Pointer to simulation environment
 */
void print_simulation_status(const SimulationEnvironment* env);

/**
 * @brief Get simulation statistics
 * 
 * @param env Pointer to simulation environment
 * @param total_targets Output total number of targets
 * @param active_targets Output number of active targets
 * @param avg_range Output average target range
 * @param avg_velocity Output average target velocity
 * @return true if statistics computed successfully, false otherwise
 */
bool get_simulation_statistics(const SimulationEnvironment* env, uint32_t* total_targets,
                              uint32_t* active_targets, double* avg_range, double* avg_velocity);

#ifdef __cplusplus
}
#endif

#endif // TARGET_SIMULATOR_H
