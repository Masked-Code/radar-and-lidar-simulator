/**
 * @file test_runner.c
 * @brief Main test runner for radar/LiDAR simulation tests
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 * 
 * This file contains the main test runner that executes all unit tests
 * for the radar/LiDAR simulation system.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

// Test framework macros
#define TEST_ASSERT(condition) \
    do { \
        if (!(condition)) { \
            printf("FAIL: %s:%d - Assertion failed: %s\n", __FILE__, __LINE__, #condition); \
            return false; \
        } \
    } while(0)

#define TEST_ASSERT_EQUAL(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            printf("FAIL: %s:%d - Expected %g, got %g\n", __FILE__, __LINE__, (double)(expected), (double)(actual)); \
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

// Test function pointer type
typedef bool (*TestFunction)(void);

// Test structure
typedef struct {
    const char* name;
    TestFunction function;
    const char* category;
} TestCase;

// Test statistics
typedef struct {
    int total_tests;
    int passed_tests;
    int failed_tests;
    double total_time;
    int skipped_tests;
} TestStats;

// Global test statistics
static TestStats g_test_stats = {0, 0, 0, 0.0, 0};

// ============================================================================
// TEST DECLARATIONS
// ============================================================================

// FFT Tests
extern bool test_fft_initialization(void);
extern bool test_fft_forward_transform(void);
extern bool test_fft_inverse_transform(void);
extern bool test_fft_real_forward(void);
extern bool test_fft_window_functions(void);
extern bool test_fft_spectral_analysis(void);
extern bool test_fft_cross_correlation(void);
extern bool test_fft_convolution(void);

// Signal Generation Tests
extern bool test_signal_buffer_allocation(void);
extern bool test_sine_wave_generation(void);
extern bool test_chirp_generation(void);
extern bool test_noise_generation(void);
extern bool test_pulse_generation(void);
extern bool test_signal_operations(void);
extern bool test_complex_arithmetic(void);

// Target Simulator Tests
extern bool test_simulation_environment_init(void);
extern bool test_target_addition_removal(void);
extern bool test_multi_target_signal_generation(void);
extern bool test_propagation_loss_calculation(void);
extern bool test_target_motion_modeling(void);
extern bool test_rcs_modeling(void);
extern bool test_clutter_simulation(void);

// Range Detector Tests
extern bool test_range_params_initialization(void);
extern bool test_basic_range_detection(void);
extern bool test_pulse_compression(void);
extern bool test_cfar_detection(void);
extern bool test_range_tracking(void);
extern bool test_range_profile_generation(void);
extern bool test_range_ambiguity_resolution(void);

// Doppler Analyzer Tests
extern bool test_doppler_params_initialization(void);
extern bool test_velocity_doppler_conversion(void);
extern bool test_doppler_spectrum_analysis(void);
extern bool test_mti_filtering(void);
extern bool test_range_doppler_processing(void);
extern bool test_velocity_ambiguity_resolution(void);
extern bool test_pulse_doppler_processing(void);

// Utilities Tests
extern bool test_memory_management(void);
extern bool test_mathematical_functions(void);
extern bool test_file_io_operations(void);
extern bool test_string_utilities(void);
extern bool test_timing_functions(void);
extern bool test_logging_system(void);

// ============================================================================
// TEST REGISTRY
// ============================================================================

static TestCase test_cases[] = {
    // FFT Tests
    {"FFT Initialization", test_fft_initialization, "FFT"},
    {"FFT Forward Transform", test_fft_forward_transform, "FFT"},
    {"FFT Inverse Transform", test_fft_inverse_transform, "FFT"},
    {"FFT Real Forward", test_fft_real_forward, "FFT"},
    {"FFT Window Functions", test_fft_window_functions, "FFT"},
    {"FFT Spectral Analysis", test_fft_spectral_analysis, "FFT"},
    {"FFT Cross Correlation", test_fft_cross_correlation, "FFT"},
    {"FFT Convolution", test_fft_convolution, "FFT"},
    
    // Signal Generation Tests
    {"Signal Buffer Allocation", test_signal_buffer_allocation, "Signal"},
    {"Sine Wave Generation", test_sine_wave_generation, "Signal"},
    {"Chirp Generation", test_chirp_generation, "Signal"},
    {"Noise Generation", test_noise_generation, "Signal"},
    {"Pulse Generation", test_pulse_generation, "Signal"},
    {"Signal Operations", test_signal_operations, "Signal"},
    {"Complex Arithmetic", test_complex_arithmetic, "Signal"},
    
    // Target Simulator Tests
    {"Simulation Environment Init", test_simulation_environment_init, "Target"},
    {"Target Addition/Removal", test_target_addition_removal, "Target"},
    {"Multi-Target Signal Generation", test_multi_target_signal_generation, "Target"},
    {"Propagation Loss Calculation", test_propagation_loss_calculation, "Target"},
    {"Target Motion Modeling", test_target_motion_modeling, "Target"},
    {"RCS Modeling", test_rcs_modeling, "Target"},
    {"Clutter Simulation", test_clutter_simulation, "Target"},
    
    // Range Detector Tests
    {"Range Params Initialization", test_range_params_initialization, "Range"},
    {"Basic Range Detection", test_basic_range_detection, "Range"},
    {"Pulse Compression", test_pulse_compression, "Range"},
    {"CFAR Detection", test_cfar_detection, "Range"},
    {"Range Tracking", test_range_tracking, "Range"},
    {"Range Profile Generation", test_range_profile_generation, "Range"},
    {"Range Ambiguity Resolution", test_range_ambiguity_resolution, "Range"},
    
    // Doppler Analyzer Tests
    {"Doppler Params Initialization", test_doppler_params_initialization, "Doppler"},
    {"Velocity-Doppler Conversion", test_velocity_doppler_conversion, "Doppler"},
    {"Doppler Spectrum Analysis", test_doppler_spectrum_analysis, "Doppler"},
    {"MTI Filtering", test_mti_filtering, "Doppler"},
    {"Range-Doppler Processing", test_range_doppler_processing, "Doppler"},
    {"Velocity Ambiguity Resolution", test_velocity_ambiguity_resolution, "Doppler"},
    {"Pulse-Doppler Processing", test_pulse_doppler_processing, "Doppler"},
    
    // Utilities Tests
    {"Memory Management", test_memory_management, "Utils"},
    {"Mathematical Functions", test_mathematical_functions, "Utils"},
    {"File I/O Operations", test_file_io_operations, "Utils"},
    {"String Utilities", test_string_utilities, "Utils"},
    {"Timing Functions", test_timing_functions, "Utils"},
    {"Logging System", test_logging_system, "Utils"},
};

static const int num_test_cases = sizeof(test_cases) / sizeof(test_cases[0]);

// ============================================================================
// TEST RUNNER FUNCTIONS
// ============================================================================

/**
 * @brief Run a single test case
 */
static bool run_test_case(const TestCase* test) {
    printf("Running test: %s ... ", test->name);
    fflush(stdout);
    
    clock_t start_time = clock();
    bool result = test->function();
    clock_t end_time = clock();
    
    double elapsed = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
    g_test_stats.total_time += elapsed;
    
    if (result) {
        printf("PASS (%.3fs)\n", elapsed);
        g_test_stats.passed_tests++;
        return true;
    } else {
        printf("FAIL (%.3fs)\n", elapsed);
        g_test_stats.failed_tests++;
        return false;
    }
}

/**
 * @brief Run all tests in a specific category
 */
static void run_category_tests(const char* category) {
    printf("\n=== %s Tests ===\n", category);
    
    int category_passed = 0;
    int category_total = 0;
    
    for (int i = 0; i < num_test_cases; i++) {
        if (strcmp(test_cases[i].category, category) == 0) {
            category_total++;
            if (run_test_case(&test_cases[i])) {
                category_passed++;
            }
        }
    }
    
    printf("\n%s Tests Summary: %d/%d passed (%.1f%%)\n", 
           category, category_passed, category_total,
           category_total > 0 ? (100.0 * category_passed / category_total) : 0.0);
}

/**
 * @brief Print test summary
 */
static void print_test_summary(void) {
    printf("\n" "================================================================\n");
    printf("TEST SUMMARY\n");
    printf("================================================================\n");
    printf("Total tests:    %d\n", g_test_stats.total_tests);
    printf("Passed:         %d\n", g_test_stats.passed_tests);
    printf("Failed:         %d\n", g_test_stats.failed_tests);
    printf("Skipped:        %d\n", g_test_stats.skipped_tests);
    printf("Success rate:   %.1f%%\n", 
           g_test_stats.total_tests > 0 ? 
           (100.0 * g_test_stats.passed_tests / g_test_stats.total_tests) : 0.0);
    printf("Total time:     %.3f seconds\n", g_test_stats.total_time);
    printf("Average time:   %.3f seconds per test\n",
           g_test_stats.total_tests > 0 ? 
           (g_test_stats.total_time / g_test_stats.total_tests) : 0.0);
    printf("================================================================\n");
}

/**
 * @brief Print usage information
 */
static void print_usage(const char* program_name) {
    printf("Usage: %s [OPTIONS]\n\n", program_name);
    printf("Radar/LiDAR Simulation Test Runner\n\n");
    printf("OPTIONS:\n");
    printf("  --all                 Run all tests (default)\n");
    printf("  --category CATEGORY   Run tests in specific category\n");
    printf("  --list                List all available tests\n");
    printf("  --list-categories     List all test categories\n");
    printf("  --verbose             Enable verbose output\n");
    printf("  --help                Show this help message\n");
    printf("\n");
    printf("CATEGORIES:\n");
    printf("  FFT         - FFT processing tests\n");
    printf("  Signal      - Signal generation tests\n");
    printf("  Target      - Target simulation tests\n");
    printf("  Range       - Range detection tests\n");
    printf("  Doppler     - Doppler analysis tests\n");
    printf("  Utils       - Utility function tests\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s --all\n", program_name);
    printf("  %s --category FFT\n", program_name);
    printf("  %s --category Signal --verbose\n", program_name);
    printf("\n");
}

/**
 * @brief List all available tests
 */
static void list_tests(void) {
    printf("Available Tests:\n");
    printf("================\n");
    
    const char* current_category = "";
    for (int i = 0; i < num_test_cases; i++) {
        if (strcmp(current_category, test_cases[i].category) != 0) {
            current_category = test_cases[i].category;
            printf("\n%s Tests:\n", current_category);
        }
        printf("  - %s\n", test_cases[i].name);
    }
    printf("\nTotal: %d tests\n", num_test_cases);
}

/**
 * @brief List all test categories
 */
static void list_categories(void) {
    printf("Available Test Categories:\n");
    printf("=========================\n");
    
    // Count tests per category
    const char* categories[] = {"FFT", "Signal", "Target", "Range", "Doppler", "Utils"};
    const int num_categories = sizeof(categories) / sizeof(categories[0]);
    
    for (int i = 0; i < num_categories; i++) {
        int count = 0;
        for (int j = 0; j < num_test_cases; j++) {
            if (strcmp(test_cases[j].category, categories[i]) == 0) {
                count++;
            }
        }
        printf("  %-10s - %d tests\n", categories[i], count);
    }
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char* argv[]) {
    printf("Radar/LiDAR Simulation Test Runner v1.0\n");
    printf("========================================\n");
    
    // Initialize test statistics
    g_test_stats.total_tests = num_test_cases;
    
    // Parse command line arguments
    bool run_all = true;
    bool verbose = false;
    const char* category_filter = NULL;
    
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--list") == 0) {
            list_tests();
            return 0;
        } else if (strcmp(argv[i], "--list-categories") == 0) {
            list_categories();
            return 0;
        } else if (strcmp(argv[i], "--all") == 0) {
            run_all = true;
        } else if (strcmp(argv[i], "--category") == 0) {
            if (i + 1 < argc) {
                category_filter = argv[++i];
                run_all = false;
            } else {
                printf("Error: --category requires a category name\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else {
            printf("Error: Unknown option '%s'\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    
    // Run tests
    clock_t start_time = clock();
    
    if (run_all) {
        printf("Running all tests...\n");
        
        // Run tests by category for better organization
        const char* categories[] = {"FFT", "Signal", "Target", "Range", "Doppler", "Utils"};
        const int num_categories = sizeof(categories) / sizeof(categories[0]);
        
        for (int i = 0; i < num_categories; i++) {
            run_category_tests(categories[i]);
        }
    } else if (category_filter) {
        // Validate category
        bool valid_category = false;
        for (int i = 0; i < num_test_cases; i++) {
            if (strcmp(test_cases[i].category, category_filter) == 0) {
                valid_category = true;
                break;
            }
        }
        
        if (!valid_category) {
            printf("Error: Unknown category '%s'\n", category_filter);
            list_categories();
            return 1;
        }
        
        run_category_tests(category_filter);
        
        // Update totals for category-only run
        g_test_stats.total_tests = g_test_stats.passed_tests + g_test_stats.failed_tests;
    }
    
    clock_t end_time = clock();
    g_test_stats.total_time = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
    
    // Print summary
    print_test_summary();
    
    // Return appropriate exit code
    if (g_test_stats.failed_tests > 0) {
        printf("\nSome tests failed. Please check the output above.\n");
        return 1;
    } else {
        printf("\nAll tests passed successfully!\n");
        return 0;
    }
}
