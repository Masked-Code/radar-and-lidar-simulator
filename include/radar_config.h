/**
 * @file radar_config.h
 * @brief Configuration constants and default parameters for radar/LiDAR simulation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This header contains system-wide configuration constants, default parameter
 * values, and compile-time options for the radar/LiDAR simulation system.
 */

#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================

/** Speed of light in vacuum (m/s) */
#define SPEED_OF_LIGHT 299792458.0

/** Boltzmann constant (J/K) */
#define BOLTZMANN_CONSTANT 1.380649e-23

/** Standard atmospheric pressure (Pa) */
#define STANDARD_PRESSURE 101325.0

/** Standard temperature (K) */
#define STANDARD_TEMPERATURE 293.15

/** Permittivity of free space (F/m) */
#define EPSILON_0 8.854187817e-12

/** Permeability of free space (H/m) */
#define MU_0 (4.0 * M_PI * 1e-7)

// ============================================================================
// SYSTEM CONFIGURATION DEFAULTS
// ============================================================================

/** Default radar operating frequency (Hz) - X-band */
#define DEFAULT_RADAR_FREQUENCY 10.0e9

/** Default LiDAR operating frequency (Hz) - 905nm wavelength */
#define DEFAULT_LIDAR_FREQUENCY 3.3e14

/** Default system sample rate (Hz) */
#define DEFAULT_SAMPLE_RATE 100e6

/** Default FFT size for spectral analysis */
#define DEFAULT_FFT_SIZE 1024

/** Default pulse width (seconds) */
#define DEFAULT_PULSE_WIDTH 1e-6

/** Default pulse repetition frequency (Hz) */
#define DEFAULT_PRF 1000.0

/** Default transmit power (watts) */
#define DEFAULT_TX_POWER 1000.0

/** Default antenna gain (dB) */
#define DEFAULT_ANTENNA_GAIN 30.0

/** Default system noise temperature (K) */
#define DEFAULT_NOISE_TEMPERATURE 290.0

/** Default receiver noise figure (dB) */
#define DEFAULT_NOISE_FIGURE 3.0

// ============================================================================
// SIGNAL PROCESSING PARAMETERS
// ============================================================================

/** Default chirp bandwidth (Hz) */
#define DEFAULT_CHIRP_BANDWIDTH 100e6

/** Maximum supported chirp bandwidth (Hz) */
#define MAX_CHIRP_BANDWIDTH 1e9

/** Default window overlap ratio for STFT */
#define DEFAULT_WINDOW_OVERLAP 0.5

/** Default zero-padding factor for FFT */
#define DEFAULT_ZERO_PAD_FACTOR 2

/** Detection threshold above noise floor (dB) */
#define DEFAULT_DETECTION_THRESHOLD 13.0

/** Default number of coherent integration pulses */
#define DEFAULT_COHERENT_INTEGRATION 16

/** Default number of non-coherent integration pulses */
#define DEFAULT_NONCOHERENT_INTEGRATION 4

// ============================================================================
// TARGET SIMULATION DEFAULTS
// ============================================================================

/** Default target range (meters) */
#define DEFAULT_TARGET_RANGE 1000.0

/** Default target velocity (m/s) */
#define DEFAULT_TARGET_VELOCITY 50.0

/** Default target RCS (m²) */
#define DEFAULT_TARGET_RCS 1.0

/** Default target reflectivity for LiDAR */
#define DEFAULT_TARGET_REFLECTIVITY 0.5

/** Maximum simulation range (meters) */
#define MAX_SIMULATION_RANGE 100000.0

/** Maximum target velocity (m/s) */
#define MAX_TARGET_VELOCITY 1000.0

// ============================================================================
// PROCESSING LIMITS AND CONSTRAINTS
// ============================================================================

/** Maximum number of FFT points */
#define MAX_FFT_SIZE 65536

/** Maximum signal buffer size (samples) */
#define MAX_BUFFER_SIZE 1048576

/** Maximum number of processing threads */
#define MAX_THREADS 16

/** Minimum SNR for reliable detection (dB) */
#define MIN_DETECTION_SNR -20.0

/** Maximum SNR for simulation (dB) */
#define MAX_SIMULATION_SNR 60.0

/** Range gate width safety factor */
#define RANGE_GATE_SAFETY_FACTOR 1.2

// ============================================================================
// NUMERICAL PRECISION SETTINGS
// ============================================================================

/** Floating point comparison tolerance */
#define FLOAT_TOLERANCE 1e-12

/** Minimum non-zero value for calculations */
#define MIN_NONZERO_VALUE 1e-15

/** Maximum number of iterations for convergence algorithms */
#define MAX_ITERATIONS 1000

/** Convergence threshold for iterative algorithms */
#define CONVERGENCE_THRESHOLD 1e-9

// ============================================================================
// FREQUENCY BAND DEFINITIONS
// ============================================================================

/** L-band frequency range (Hz) */
#define L_BAND_MIN 1.0e9
#define L_BAND_MAX 2.0e9

/** S-band frequency range (Hz) */
#define S_BAND_MIN 2.0e9
#define S_BAND_MAX 4.0e9

/** C-band frequency range (Hz) */
#define C_BAND_MIN 4.0e9
#define C_BAND_MAX 8.0e9

/** X-band frequency range (Hz) */
#define X_BAND_MIN 8.0e9
#define X_BAND_MAX 12.0e9

/** Ku-band frequency range (Hz) */
#define KU_BAND_MIN 12.0e9
#define KU_BAND_MAX 18.0e9

/** K-band frequency range (Hz) */
#define K_BAND_MIN 18.0e9
#define K_BAND_MAX 27.0e9

/** Ka-band frequency range (Hz) */
#define KA_BAND_MIN 27.0e9
#define KA_BAND_MAX 40.0e9

// ============================================================================
// ATMOSPHERIC MODELING PARAMETERS
// ============================================================================

/** Standard atmospheric attenuation at X-band (dB/km) */
#define ATMOSPHERIC_LOSS_X_BAND 0.01

/** Rain attenuation factor (dB/km per mm/hr) */
#define RAIN_ATTENUATION_FACTOR 0.1

/** Fog attenuation coefficient for LiDAR (dB/km per g/m³) */
#define FOG_ATTENUATION_LIDAR 1.0

/** Molecular scattering coefficient */
#define MOLECULAR_SCATTERING_COEFF 3.9e-6

// ============================================================================
// ERROR MODELING PARAMETERS
// ============================================================================

/** Phase noise standard deviation (radians) */
#define DEFAULT_PHASE_NOISE_STD 0.1

/** Amplitude noise standard deviation (linear) */
#define DEFAULT_AMPLITUDE_NOISE_STD 0.01

/** Timing jitter standard deviation (seconds) */
#define DEFAULT_TIMING_JITTER_STD 1e-12

/** Frequency stability (fractional) */
#define DEFAULT_FREQUENCY_STABILITY 1e-9

// ============================================================================
// OUTPUT FORMATTING OPTIONS
// ============================================================================

/** Default output precision for floating point values */
#define OUTPUT_PRECISION 6

/** CSV field separator character */
#define CSV_SEPARATOR ","

/** Default output file buffer size */
#define OUTPUT_BUFFER_SIZE 65536

// ============================================================================
// WINDOW FUNCTION TYPES
// ============================================================================

typedef enum {
    WINDOW_RECTANGULAR = 0,     /**< Rectangular (no) window */
    WINDOW_HAMMING,             /**< Hamming window */
    WINDOW_HANNING,             /**< Hanning window */
    WINDOW_BLACKMAN,            /**< Blackman window */
    WINDOW_KAISER,              /**< Kaiser window */
    WINDOW_GAUSSIAN,            /**< Gaussian window */
    WINDOW_TUKEY,               /**< Tukey window */
    WINDOW_BARTLETT,            /**< Bartlett window */
    WINDOW_COUNT                /**< Number of window types */
} WindowType;

// ============================================================================
// DOPPLER PROCESSING CONFIGURATION
// ============================================================================

/** Maximum unambiguous Doppler frequency factor */
#define MAX_DOPPLER_FACTOR 0.5

/** Doppler bank size for processing */
#define DOPPLER_BANK_SIZE 64

/** MTI filter order */
#define MTI_FILTER_ORDER 3

/** Clutter rejection threshold (dB) */
#define CLUTTER_REJECTION_THRESHOLD 40.0

// ============================================================================
// BEAMFORMING PARAMETERS
// ============================================================================

/** Default number of array elements */
#define DEFAULT_ARRAY_ELEMENTS 16

/** Default element spacing (wavelengths) */
#define DEFAULT_ELEMENT_SPACING 0.5

/** Sidelobe level target (dB) */
#define TARGET_SIDELOBE_LEVEL -40.0

/** Beamforming weight calculation tolerance */
#define BEAMFORMING_TOLERANCE 1e-6

// ============================================================================
// CALIBRATION PARAMETERS
// ============================================================================

/** Calibration target RCS (dBsm) */
#define CALIBRATION_TARGET_RCS 0.0

/** Calibration accuracy requirement (dB) */
#define CALIBRATION_ACCURACY 0.1

/** Temperature coefficient for frequency (ppm/°C) */
#define TEMP_COEFF_FREQUENCY 25.0

// ============================================================================
// UTILITY MACROS
// ============================================================================

/** Convert dB to linear scale */
#define DB_TO_LINEAR(db) pow(10.0, (db) / 10.0)

/** Convert linear to dB scale */
#define LINEAR_TO_DB(linear) (10.0 * log10(linear))

/** Convert dBm to watts */
#define DBM_TO_WATTS(dbm) (1e-3 * pow(10.0, (dbm) / 10.0))

/** Convert watts to dBm */
#define WATTS_TO_DBM(watts) (10.0 * log10((watts) / 1e-3))

/** Convert degrees to radians */
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)

/** Convert radians to degrees */
#define RAD_TO_DEG(rad) ((rad) * 180.0 / M_PI)

/** Calculate wavelength from frequency */
#define WAVELENGTH(freq) (SPEED_OF_LIGHT / (freq))

/** Calculate frequency from wavelength */
#define FREQUENCY(wavelength) (SPEED_OF_LIGHT / (wavelength))

/** Check if value is power of 2 */
#define IS_POWER_OF_2(n) ((n) && !((n) & ((n) - 1)))

/** Round up to next power of 2 */
#define NEXT_POWER_OF_2(n) ((n) <= 1 ? 1 : (1 << ((int)ceil(log2(n)))))

// ============================================================================
// VALIDATION MACROS
// ============================================================================

/** Validate frequency range */
#define VALIDATE_FREQUENCY(f) ((f) > 0 && (f) < 1e15)

/** Validate sample rate */
#define VALIDATE_SAMPLE_RATE(sr) ((sr) >= MIN_SAMPLE_RATE && (sr) <= MAX_SAMPLE_RATE)

/** Validate range */
#define VALIDATE_RANGE(r) ((r) >= 0 && (r) <= MAX_SIMULATION_RANGE)

/** Validate velocity */
#define VALIDATE_VELOCITY(v) (fabs(v) <= MAX_TARGET_VELOCITY)

/** Validate SNR */
#define VALIDATE_SNR(snr) ((snr) >= MIN_DETECTION_SNR && (snr) <= MAX_SIMULATION_SNR)

#ifdef __cplusplus
}
#endif

#endif // RADAR_CONFIG_H
