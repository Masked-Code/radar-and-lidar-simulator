/**
 * @file main.c
 * @brief Main program for Radar/LiDAR Signal Processing Simulation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This program demonstrates a comprehensive radar and LiDAR signal processing
 * simulation with FFT-based analysis, multi-target detection, and Doppler
 * processing capabilities.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <getopt.h>

// Include all simulation headers
#include "signal_types.h"
#include "radar_config.h"
#include "fft_processor.h"
#include "target_simulator.h"
#include "doppler_analyzer.h"
#include "range_detector.h"
#include "utils.h"

// ============================================================================
// PROGRAM CONFIGURATION
// ============================================================================

/**
 * @brief Program configuration structure
 */
typedef struct {
    SimulationMode mode;                /**< Simulation mode (radar/LiDAR) */
    double carrier_frequency;           /**< Carrier frequency (Hz) */
    double sample_rate;                 /**< Sample rate (Hz) */
    double pulse_width;                 /**< Pulse width (s) */
    double prf;                         /**< Pulse repetition frequency (Hz) */
    double max_range;                   /**< Maximum detection range (m) */
    uint32_t num_targets;               /**< Number of targets to simulate */
    uint32_t num_pulses;                /**< Number of pulses per CPI */
    bool enable_doppler;                /**< Enable Doppler processing */
    bool enable_noise;                  /**< Enable noise simulation */
    bool enable_clutter;                /**< Enable clutter simulation */
    double snr_db;                      /**< Signal-to-noise ratio (dB) */
    char output_file[256];              /**< Output filename */
    char config_file[256];              /**< Configuration filename */
    bool verbose;                       /**< Verbose output */
    bool save_signals;                  /**< Save signal data */
    bool generate_plots;                /**< Generate plot data */
} ProgramConfig;

// Default configuration values
static const ProgramConfig DEFAULT_CONFIG = {
    .mode = MODE_RADAR,
    .carrier_frequency = DEFAULT_RADAR_FREQUENCY,
    .sample_rate = DEFAULT_SAMPLE_RATE,
    .pulse_width = DEFAULT_PULSE_WIDTH,
    .prf = DEFAULT_PRF,
    .max_range = 10000.0,
    .num_targets = 3,
    .num_pulses = 16,
    .enable_doppler = true,
    .enable_noise = true,
    .enable_clutter = false,
    .snr_db = 20.0,
    .output_file = "simulation_results.csv",
    .config_file = "",
    .verbose = false,
    .save_signals = false,
    .generate_plots = false
};

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

static void print_usage(const char* program_name);
static void print_version(void);
static bool parse_command_line(int argc, char* argv[], ProgramConfig* config);
static bool initialize_simulation(const ProgramConfig* config, SystemConfig* sys_config);
static bool run_radar_simulation(const ProgramConfig* config);
static bool run_lidar_simulation(const ProgramConfig* config);
static bool create_test_scenario(SimulationEnvironment* env, const ProgramConfig* config);
static void print_simulation_results(const DetectionResult* detections, uint32_t num_detections);
static void print_config_summary(const ProgramConfig* config);
static void cleanup_and_exit(int exit_code);

// Global variables for cleanup
static FFTProcessor* g_fft_processor = NULL;
static SimulationEnvironment* g_sim_env = NULL;

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char* argv[]) {
    ProgramConfig config = DEFAULT_CONFIG;
    HighResTimer total_timer;
    
    printf("=================================================================\n");
    printf("     Radar/LiDAR Signal Processing Simulation v1.0\n");
    printf("     Advanced Signal Processing and Target Detection\n");
    printf("=================================================================\n\n");

    // Initialize logging system
    if (!log_init(NULL, LOG_LEVEL_INFO)) {
        fprintf(stderr, "Warning: Failed to initialize logging system\n");
    }

    // Parse command line arguments
    if (!parse_command_line(argc, argv, &config)) {
        cleanup_and_exit(EXIT_FAILURE);
    }

    // Print configuration summary if verbose
    if (config.verbose) {
        print_config_summary(&config);
    }

    // Start total execution timer
    timer_start(&total_timer);

    // Run simulation based on mode
    bool success = false;
    switch (config.mode) {
        case MODE_RADAR:
            log_info("Starting radar simulation...");
            success = run_radar_simulation(&config);
            break;
            
        case MODE_LIDAR:
            log_info("Starting LiDAR simulation...");
            success = run_lidar_simulation(&config);
            break;
            
        case MODE_HYBRID:
            log_info("Starting hybrid radar/LiDAR simulation...");
            // Run both radar and LiDAR simulations
            success = run_radar_simulation(&config) && run_lidar_simulation(&config);
            break;
            
        default:
            log_error("Invalid simulation mode: %d", config.mode);
            cleanup_and_exit(EXIT_FAILURE);
    }

    // Stop timer and print execution time
    timer_stop(&total_timer);
    double execution_time = timer_get_elapsed_seconds(&total_timer);
    
    printf("\n=================================================================\n");
    if (success) {
        printf("Simulation completed successfully!\n");
        printf("Total execution time: %.3f seconds\n", execution_time);
        printf("Results saved to: %s\n", config.output_file);
    } else {
        printf("Simulation failed - check error messages above\n");
        cleanup_and_exit(EXIT_FAILURE);
    }
    printf("=================================================================\n");

    cleanup_and_exit(EXIT_SUCCESS);
    return EXIT_SUCCESS;  // Never reached
}

// ============================================================================
// RADAR SIMULATION
// ============================================================================

static bool run_radar_simulation(const ProgramConfig* config) {
    HighResTimer timer;
    timer_start(&timer);
    
    log_info("Initializing radar simulation components...");
    
    // Initialize simulation environment
    g_sim_env = calloc(1, sizeof(SimulationEnvironment));
    if (!g_sim_env) {
        log_error("Failed to allocate simulation environment");
        return false;
    }
    
    if (!simulation_environment_init(g_sim_env, config->num_targets, 0.001)) {
        log_error("Failed to initialize simulation environment");
        return false;
    }

    // Create test scenario
    if (!create_test_scenario(g_sim_env, config)) {
        log_error("Failed to create test scenario");
        return false;
    }

    // Initialize FFT processor
    g_fft_processor = calloc(1, sizeof(FFTProcessor));
    if (!g_fft_processor) {
        log_error("Failed to allocate FFT processor");
        return false;
    }
    
    if (!fft_processor_init(g_fft_processor, DEFAULT_FFT_SIZE, WINDOW_HAMMING)) {
        log_error("Failed to initialize FFT processor");
        return false;
    }

    // Generate transmit signal (linear chirp)
    SignalBuffer tx_signal = {0};
    if (!allocate_signal_buffer(&tx_signal, 
                               (size_t)(config->pulse_width * config->sample_rate),
                               config->sample_rate)) {
        log_error("Failed to allocate transmit signal buffer");
        return false;
    }

    tx_signal.center_frequency = config->carrier_frequency;
    tx_signal.bandwidth = DEFAULT_CHIRP_BANDWIDTH;
    tx_signal.waveform = WAVEFORM_CHIRP_LINEAR;

    double start_freq = config->carrier_frequency - DEFAULT_CHIRP_BANDWIDTH / 2.0;
    double end_freq = config->carrier_frequency + DEFAULT_CHIRP_BANDWIDTH / 2.0;
    
    if (!generate_linear_chirp(&tx_signal, start_freq, end_freq, 1.0)) {
        log_error("Failed to generate transmit chirp signal");
        free_signal_buffer(&tx_signal);
        return false;
    }

    printf("Generated LFM chirp signal:\n");
    printf("  - Duration: %.2f µs\n", config->pulse_width * 1e6);
    printf("  - Bandwidth: %.1f MHz\n", DEFAULT_CHIRP_BANDWIDTH / 1e6);
    printf("  - Samples: %zu\n", tx_signal.length);

    // Generate multi-target received signal
    SignalBuffer rx_signal = {0};
    PulseParams pulse_params = {
        .pulse_width = config->pulse_width,
        .pulse_repetition_freq = config->prf,
        .carrier_frequency = config->carrier_frequency,
        .peak_power = DEFAULT_TX_POWER,
        .duty_cycle = config->pulse_width * config->prf,
        .modulation = WAVEFORM_CHIRP_LINEAR,
        .chirp_bandwidth = DEFAULT_CHIRP_BANDWIDTH,
        .phase_offset = 0.0
    };

    if (!generate_multi_target_signal(g_sim_env, &tx_signal, &rx_signal, &pulse_params)) {
        log_error("Failed to generate multi-target signal");
        free_signal_buffer(&tx_signal);
        return false;
    }

    // Add noise if enabled
    if (config->enable_noise) {
        double noise_power = calculate_signal_power(&rx_signal) / DB_TO_LINEAR(config->snr_db);
        if (!add_awgn_to_signal(&rx_signal, noise_power)) {
            log_warning("Failed to add noise to received signal");
        } else {
            printf("Added AWGN with SNR: %.1f dB\n", config->snr_db);
        }
    }

    // Perform pulse compression
    PulseCompressionFilter pc_filter = {0};
    if (!pulse_compression_init(&pc_filter, &tx_signal, DEFAULT_FFT_SIZE)) {
        log_error("Failed to initialize pulse compression filter");
        free_signal_buffer(&tx_signal);
        free_signal_buffer(&rx_signal);
        return false;
    }

    SignalBuffer compressed_signal = {0};
    if (!apply_pulse_compression(&pc_filter, &rx_signal, &compressed_signal)) {
        log_error("Failed to apply pulse compression");
        pulse_compression_cleanup(&pc_filter);
        free_signal_buffer(&tx_signal);
        free_signal_buffer(&rx_signal);
        return false;
    }

    printf("Applied pulse compression (gain: %.1f dB)\n", 
           calculate_compression_gain(pc_filter.compression_ratio));

    // Generate range profile
    RangeParams range_params = {0};
    if (!range_params_init(&range_params, config->sample_rate, 
                          config->pulse_width, config->max_range)) {
        log_error("Failed to initialize range parameters");
        pulse_compression_cleanup(&pc_filter);
        free_signal_buffer(&tx_signal);
        free_signal_buffer(&rx_signal);
        free_signal_buffer(&compressed_signal);
        return false;
    }

    RangeMeasurement range_measurement = {0};
    if (!generate_range_profile(&compressed_signal, &range_params, &range_measurement)) {
        log_error("Failed to generate range profile");
        pulse_compression_cleanup(&pc_filter);
        free_signal_buffer(&tx_signal);
        free_signal_buffer(&rx_signal);
        free_signal_buffer(&compressed_signal);
        return false;
    }

    // Detect targets using CFAR
    CFARDetector cfar_detector = {0};
    if (!cfar_detector_init(&cfar_detector, 16, 4, 1e-6, 0)) {
        log_error("Failed to initialize CFAR detector");
    } else {
        uint8_t* detections = calloc(range_measurement.profile_length, sizeof(uint8_t));
        double* thresholds = calloc(range_measurement.profile_length, sizeof(double));
        
        if (detections && thresholds) {
            uint32_t num_cfar_detections = apply_cfar_detection(&cfar_detector,
                range_measurement.range_profile, range_measurement.profile_length,
                detections, thresholds);
            
            printf("CFAR detector found %u potential targets\n", num_cfar_detections);
            
            // Extract detection results
            DetectionResult detection_results[MAX_TARGETS];
            uint32_t result_count = 0;
            
            for (uint32_t i = 0; i < range_measurement.profile_length && result_count < MAX_TARGETS; i++) {
                if (detections[i]) {
                    detection_results[result_count].target_id = result_count + 1;
                    detection_results[result_count].detected_range = range_bin_to_value(i, &range_params);
                    detection_results[result_count].detected_velocity = 0.0;  // No Doppler processing yet
                    detection_results[result_count].snr_db = LINEAR_TO_DB(
                        range_measurement.range_profile[i] / thresholds[i]);
                    detection_results[result_count].confidence = 0.95;
                    detection_results[result_count].is_valid = true;
                    result_count++;
                }
            }
            
            print_simulation_results(detection_results, result_count);
            
            // Save results to file
            if (!save_detections_to_csv(detection_results, result_count, config->output_file)) {
                log_warning("Failed to save detection results to file");
            }
        }
        
        free(detections);
        free(thresholds);
        cfar_detector_cleanup(&cfar_detector);
    }

    // Doppler processing (if enabled and multiple pulses)
    if (config->enable_doppler && config->num_pulses > 1) {
        printf("\nPerforming Doppler analysis...\n");
        
        // Simulate multiple pulse returns for Doppler analysis
        SignalBuffer* pulse_train = calloc(config->num_pulses, sizeof(SignalBuffer));
        if (pulse_train) {
            DopplerParams doppler_params = {0};
            if (doppler_params_init(&doppler_params, config->carrier_frequency, 
                                   config->prf, config->num_pulses)) {
                
                // Generate pulse train (simplified - would normally be from coherent processing interval)
                for (uint32_t p = 0; p < config->num_pulses; p++) {
                    if (!generate_multi_target_signal(g_sim_env, &tx_signal, &pulse_train[p], &pulse_params)) {
                        log_warning("Failed to generate pulse %u for Doppler analysis", p);
                        continue;
                    }
                    
                    if (config->enable_noise) {
                        double noise_power = calculate_signal_power(&pulse_train[p]) / DB_TO_LINEAR(config->snr_db);
                        add_awgn_to_signal(&pulse_train[p], noise_power);
                    }
                }
                
                DopplerAnalysis doppler_analysis = {0};
                if (analyze_doppler(pulse_train, config->num_pulses, &doppler_params, &doppler_analysis)) {
                    printf("Doppler analysis results:\n");
                    printf("  - Peak velocity: %.2f m/s\n", doppler_analysis.peak_velocity);
                    printf("  - Velocity spread: %.2f m/s\n", doppler_analysis.velocity_spread);
                    printf("  - Number of velocity peaks: %u\n", doppler_analysis.num_peaks_detected);
                    
                    doppler_analysis_cleanup(&doppler_analysis);
                } else {
                    log_warning("Doppler analysis failed");
                }
            }
            
            // Clean up pulse train
            for (uint32_t p = 0; p < config->num_pulses; p++) {
                free_signal_buffer(&pulse_train[p]);
            }
            free(pulse_train);
        }
    }

    // Clean up
    pulse_compression_cleanup(&pc_filter);
    range_measurement_cleanup(&range_measurement);
    free_signal_buffer(&tx_signal);
    free_signal_buffer(&rx_signal);
    free_signal_buffer(&compressed_signal);

    timer_stop(&timer);
    printf("\nRadar simulation completed in %.3f seconds\n", timer_get_elapsed_seconds(&timer));
    
    return true;
}

// ============================================================================
// LIDAR SIMULATION
// ============================================================================

static bool run_lidar_simulation(const ProgramConfig* config) {
    HighResTimer timer;
    timer_start(&timer);
    
    log_info("Initializing LiDAR simulation components...");
    printf("LiDAR simulation mode (simplified implementation)\n");
    
    // For LiDAR, we'll use a simpler approach focusing on time-of-flight
    // without the complexity of RF signal processing
    
    if (!g_sim_env) {
        g_sim_env = calloc(1, sizeof(SimulationEnvironment));
        if (!g_sim_env || !simulation_environment_init(g_sim_env, config->num_targets, 0.001)) {
            log_error("Failed to initialize LiDAR simulation environment");
            return false;
        }
        
        if (!create_test_scenario(g_sim_env, config)) {
            log_error("Failed to create LiDAR test scenario");
            return false;
        }
    }

    printf("LiDAR Configuration:\n");
    printf("  - Wavelength: %.1f nm\n", WAVELENGTH(DEFAULT_LIDAR_FREQUENCY) * 1e9);
    printf("  - Pulse width: %.1f ns\n", config->pulse_width * 1e9);
    printf("  - Number of targets: %u\n", g_sim_env->target_count);

    // Simulate LiDAR returns (simplified)
    DetectionResult lidar_results[MAX_TARGETS];
    uint32_t num_lidar_detections = 0;
    
    for (uint32_t i = 0; i < g_sim_env->target_count && num_lidar_detections < MAX_TARGETS; i++) {
        TargetParams* target = &g_sim_env->targets[i];
        if (!target->is_active) continue;
        
        // Calculate LiDAR return strength based on reflectivity and range
        double range_loss_db = 40.0 * log10(target->range / 1000.0);  // 40 dB per decade for LiDAR
        double return_power_db = 10.0 * log10(target->reflectivity) - range_loss_db;
        
        // Simple threshold detection
        if (return_power_db > -60.0) {  // LiDAR detection threshold
            lidar_results[num_lidar_detections].target_id = target->id;
            lidar_results[num_lidar_detections].detected_range = target->range;
            lidar_results[num_lidar_detections].detected_velocity = target->velocity;
            lidar_results[num_lidar_detections].snr_db = return_power_db + 60.0;
            lidar_results[num_lidar_detections].confidence = 0.90;
            lidar_results[num_lidar_detections].is_valid = true;
            lidar_results[num_lidar_detections].range_error = 0.1;  // 10 cm accuracy
            lidar_results[num_lidar_detections].velocity_error = 0.01; // 1 cm/s accuracy
            
            num_lidar_detections++;
        }
    }

    printf("\nLiDAR Detection Results:\n");
    print_simulation_results(lidar_results, num_lidar_detections);
    
    // Save LiDAR results
    char lidar_output[300];
    snprintf(lidar_output, sizeof(lidar_output), "lidar_%s", config->output_file);
    if (!save_detections_to_csv(lidar_results, num_lidar_detections, lidar_output)) {
        log_warning("Failed to save LiDAR detection results");
    }

    timer_stop(&timer);
    printf("LiDAR simulation completed in %.3f seconds\n", timer_get_elapsed_seconds(&timer));
    
    return true;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

static bool create_test_scenario(SimulationEnvironment* env, const ProgramConfig* config) {
    log_info("Creating test scenario with %u targets", config->num_targets);
    
    // Set environmental parameters
    env->temperature_kelvin = STANDARD_TEMPERATURE;
    env->humidity_percent = 50.0;
    env->atmospheric_loss_db = ATMOSPHERIC_LOSS_X_BAND;
    env->enable_clutter = config->enable_clutter;
    
    // Create targets with realistic parameters
    for (uint32_t i = 0; i < config->num_targets; i++) {
        TargetParams target_params = {0};
        TargetMotion target_motion = {0};
        
        // Generate target parameters
        target_params.id = i + 1;
        target_params.range = random_uniform(1000.0, config->max_range * 0.8);
        target_params.velocity = random_uniform(-100.0, 100.0);  // m/s
        target_params.azimuth = random_uniform(-M_PI/4, M_PI/4);  // ±45 degrees
        target_params.elevation = random_uniform(-M_PI/12, M_PI/12);  // ±15 degrees
        target_params.rcs = DB_TO_LINEAR(random_uniform(-10.0, 20.0));  // -10 to +20 dBsm
        target_params.reflectivity = random_uniform(0.1, 0.9);
        target_params.is_active = true;
        target_params.snr_db = config->snr_db + random_uniform(-5.0, 5.0);
        
        // Set initial motion
        target_motion.position[0] = target_params.range * cos(target_params.elevation) * cos(target_params.azimuth);
        target_motion.position[1] = target_params.range * cos(target_params.elevation) * sin(target_params.azimuth);
        target_motion.position[2] = target_params.range * sin(target_params.elevation);
        
        // Add some velocity components
        double speed = fabs(target_params.velocity);
        target_motion.velocity[0] = speed * cos(target_params.azimuth + M_PI);  // Approaching/receding
        target_motion.velocity[1] = random_uniform(-10.0, 10.0);  // Cross-range
        target_motion.velocity[2] = random_uniform(-5.0, 5.0);    // Altitude
        
        uint32_t target_id = add_target(env, &target_params, &target_motion);
        if (target_id == 0) {
            log_error("Failed to add target %u", i + 1);
            return false;
        }
        
        if (config->verbose) {
            printf("Target %u: Range=%.1f m, Velocity=%.1f m/s, RCS=%.1f dBsm\n",
                   target_id, target_params.range, target_params.velocity,
                   LINEAR_TO_DB(target_params.rcs));
        }
    }
    
    return true;
}

static void print_simulation_results(const DetectionResult* detections, uint32_t num_detections) {
    printf("\n======== DETECTION RESULTS ========\n");
    printf("Number of detections: %u\n\n", num_detections);
    
    if (num_detections == 0) {
        printf("No targets detected.\n");
        return;
    }
    
    printf("ID    Range (m)  Velocity (m/s)  SNR (dB)  Confidence\n");
    printf("--  -----------  -------------  --------  ----------\n");
    
    for (uint32_t i = 0; i < num_detections; i++) {
        const DetectionResult* det = &detections[i];
        printf("%2u  %11.1f  %13.2f  %8.1f  %10.3f\n",
               det->target_id, det->detected_range, det->detected_velocity,
               det->snr_db, det->confidence);
    }
    printf("=====================================\n");
}

static void print_config_summary(const ProgramConfig* config) {
    printf("\nSimulation Configuration:\n");
    printf("  Mode: %s\n", (config->mode == MODE_RADAR) ? "Radar" : 
                          (config->mode == MODE_LIDAR) ? "LiDAR" : "Hybrid");
    printf("  Carrier frequency: %.3f GHz\n", config->carrier_frequency / 1e9);
    printf("  Sample rate: %.1f MHz\n", config->sample_rate / 1e6);
    printf("  Pulse width: %.1f µs\n", config->pulse_width * 1e6);
    printf("  PRF: %.1f Hz\n", config->prf);
    printf("  Max range: %.1f km\n", config->max_range / 1000.0);
    printf("  Number of targets: %u\n", config->num_targets);
    printf("  SNR: %.1f dB\n", config->snr_db);
    printf("  Doppler processing: %s\n", config->enable_doppler ? "Enabled" : "Disabled");
    printf("  Output file: %s\n", config->output_file);
    printf("\n");
}

static bool parse_command_line(int argc, char* argv[], ProgramConfig* config) {
    static struct option long_options[] = {
        {"help",        no_argument,       0, 'h'},
        {"version",     no_argument,       0, 'V'},
        {"mode",        required_argument, 0, 'm'},
        {"frequency",   required_argument, 0, 'f'},
        {"sample-rate", required_argument, 0, 's'},
        {"pulse-width", required_argument, 0, 'p'},
        {"prf",         required_argument, 0, 'r'},
        {"range",       required_argument, 0, 'R'},
        {"targets",     required_argument, 0, 't'},
        {"pulses",      required_argument, 0, 'P'},
        {"snr",         required_argument, 0, 'S'},
        {"output",      required_argument, 0, 'o'},
        {"config",      required_argument, 0, 'c'},
        {"verbose",     no_argument,       0, 'v'},
        {"no-doppler",  no_argument,       0, 'D'},
        {"no-noise",    no_argument,       0, 'N'},
        {"clutter",     no_argument,       0, 'C'},
        {0, 0, 0, 0}
    };

    int c;
    int option_index = 0;

    while ((c = getopt_long(argc, argv, "hVm:f:s:p:r:R:t:P:S:o:c:vDNC", 
                           long_options, &option_index)) != -1) {
        switch (c) {
            case 'h':
                print_usage(argv[0]);
                exit(EXIT_SUCCESS);
                
            case 'V':
                print_version();
                exit(EXIT_SUCCESS);
                
            case 'm':
                if (strcmp(optarg, "radar") == 0) {
                    config->mode = MODE_RADAR;
                } else if (strcmp(optarg, "lidar") == 0) {
                    config->mode = MODE_LIDAR;
                } else if (strcmp(optarg, "hybrid") == 0) {
                    config->mode = MODE_HYBRID;
                } else {
                    fprintf(stderr, "Error: Invalid mode '%s'. Use radar, lidar, or hybrid.\n", optarg);
                    return false;
                }
                break;
                
            case 'f':
                config->carrier_frequency = atof(optarg);
                if (config->carrier_frequency <= 0) {
                    fprintf(stderr, "Error: Invalid frequency value\n");
                    return false;
                }
                break;
                
            case 's':
                config->sample_rate = atof(optarg);
                if (config->sample_rate <= 0) {
                    fprintf(stderr, "Error: Invalid sample rate value\n");
                    return false;
                }
                break;
                
            case 'p':
                config->pulse_width = atof(optarg);
                if (config->pulse_width <= 0) {
                    fprintf(stderr, "Error: Invalid pulse width value\n");
                    return false;
                }
                break;
                
            case 'r':
                config->prf = atof(optarg);
                if (config->prf <= 0) {
                    fprintf(stderr, "Error: Invalid PRF value\n");
                    return false;
                }
                break;
                
            case 'R':
                config->max_range = atof(optarg);
                if (config->max_range <= 0) {
                    fprintf(stderr, "Error: Invalid range value\n");
                    return false;
                }
                break;
                
            case 't':
                config->num_targets = (uint32_t)atoi(optarg);
                if (config->num_targets == 0 || config->num_targets > MAX_TARGETS) {
                    fprintf(stderr, "Error: Number of targets must be 1-%d\n", MAX_TARGETS);
                    return false;
                }
                break;
                
            case 'P':
                config->num_pulses = (uint32_t)atoi(optarg);
                if (config->num_pulses == 0) {
                    fprintf(stderr, "Error: Number of pulses must be positive\n");
                    return false;
                }
                break;
                
            case 'S':
                config->snr_db = atof(optarg);
                break;
                
            case 'o':
                strncpy(config->output_file, optarg, sizeof(config->output_file) - 1);
                config->output_file[sizeof(config->output_file) - 1] = '\0';
                break;
                
            case 'c':
                strncpy(config->config_file, optarg, sizeof(config->config_file) - 1);
                config->config_file[sizeof(config->config_file) - 1] = '\0';
                break;
                
            case 'v':
                config->verbose = true;
                break;
                
            case 'D':
                config->enable_doppler = false;
                break;
                
            case 'N':
                config->enable_noise = false;
                break;
                
            case 'C':
                config->enable_clutter = true;
                break;
                
            case '?':
                fprintf(stderr, "Use --help for usage information.\n");
                return false;
                
            default:
                return false;
        }
    }

    return true;
}

static void print_usage(const char* program_name) {
    printf("Usage: %s [OPTIONS]\n\n", program_name);
    printf("Radar/LiDAR Signal Processing Simulation\n\n");
    printf("OPTIONS:\n");
    printf("  -h, --help              Show this help message\n");
    printf("  -V, --version           Show version information\n");
    printf("  -m, --mode MODE         Simulation mode: radar, lidar, hybrid (default: radar)\n");
    printf("  -f, --frequency FREQ    Carrier frequency in Hz (default: %.0e)\n", DEFAULT_RADAR_FREQUENCY);
    printf("  -s, --sample-rate RATE  Sample rate in Hz (default: %.0e)\n", DEFAULT_SAMPLE_RATE);
    printf("  -p, --pulse-width WIDTH Pulse width in seconds (default: %.0e)\n", DEFAULT_PULSE_WIDTH);
    printf("  -r, --prf PRF          Pulse repetition frequency in Hz (default: %.0f)\n", DEFAULT_PRF);
    printf("  -R, --range RANGE      Maximum detection range in meters (default: 10000)\n");
    printf("  -t, --targets NUM      Number of targets to simulate (default: 3)\n");
    printf("  -P, --pulses NUM       Number of pulses per CPI (default: 16)\n");
    printf("  -S, --snr SNR          Signal-to-noise ratio in dB (default: 20.0)\n");
    printf("  -o, --output FILE      Output filename (default: simulation_results.csv)\n");
    printf("  -c, --config FILE      Configuration file to load\n");
    printf("  -v, --verbose          Enable verbose output\n");
    printf("  -D, --no-doppler       Disable Doppler processing\n");
    printf("  -N, --no-noise         Disable noise simulation\n");
    printf("  -C, --clutter          Enable clutter simulation\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s --mode radar --targets 5 --snr 15\n", program_name);
    printf("  %s --mode lidar --frequency 3.3e14 --targets 3\n", program_name);
    printf("  %s --verbose --output my_results.csv\n", program_name);
    printf("\n");
}

static void print_version(void) {
    printf("Radar/LiDAR Signal Processing Simulation v1.0.0\n");
    printf("Built on %s at %s\n", __DATE__, __TIME__);
    printf("Copyright (C) 2025 Radar/LiDAR Simulation Team\n");
    printf("This is free software; see the source for copying conditions.\n");
}

static void cleanup_and_exit(int exit_code) {
    log_info("Cleaning up resources...");
    
    if (g_fft_processor) {
        fft_processor_cleanup(g_fft_processor);
        free(g_fft_processor);
        g_fft_processor = NULL;
    }
    
    if (g_sim_env) {
        simulation_environment_cleanup(g_sim_env);
        free(g_sim_env);
        g_sim_env = NULL;
    }
    
    log_cleanup();
    exit(exit_code);
}
