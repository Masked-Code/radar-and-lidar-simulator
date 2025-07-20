/**
 * @file fft_processor.c
 * @brief FFT processing functions implementation
 * @author Radar/LiDAR Simulation Team
 * @version 1.0
 * @date 2025
 */

#include "fft_processor.h"
#include "utils.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Simple FFT implementation (for systems without FFTW)
static void simple_fft(ComplexSample* data, int n, bool inverse);
static void generate_hamming_window(double* window, uint32_t length);

bool fft_processor_init(FFTProcessor* processor, uint32_t fft_size, WindowType window_type) {
    if (!processor || !IS_POWER_OF_2(fft_size) || fft_size == 0) {
        return false;
    }

    memset(processor, 0, sizeof(FFTProcessor));

    // Allocate buffers
    processor->input_buffer = calloc(fft_size, sizeof(ComplexSample));
    processor->output_buffer = calloc(fft_size, sizeof(ComplexSample));
    processor->window_function = calloc(fft_size, sizeof(double));

    if (!processor->input_buffer || !processor->output_buffer || !processor->window_function) {
        fft_processor_cleanup(processor);
        return false;
    }

    processor->fft_size = fft_size;
    processor->window_type = window_type;
    processor->scale_factor = 1.0 / sqrt((double)fft_size);

    // Generate window function
    if (!generate_window(window_type, fft_size, processor->window_function, 0.0)) {
        fft_processor_cleanup(processor);
        return false;
    }

    processor->is_initialized = true;
    return true;
}

void fft_processor_cleanup(FFTProcessor* processor) {
    if (!processor) return;

    free(processor->input_buffer);
    free(processor->output_buffer);
    free(processor->window_function);
    
    memset(processor, 0, sizeof(FFTProcessor));
}

bool fft_forward(FFTProcessor* processor, const SignalBuffer* input, SignalBuffer* output) {
    if (!processor || !processor->is_initialized || !input || !output) {
        return false;
    }

    if (!allocate_signal_buffer(output, processor->fft_size, input->sample_rate)) {
        return false;
    }

    // Copy input to working buffer and apply window
    size_t copy_length = (input->length < processor->fft_size) ? input->length : processor->fft_size;
    
    for (size_t i = 0; i < copy_length; i++) {
        double window_val = processor->window_function[i];
        processor->input_buffer[i].real = input->samples[i].real * window_val;
        processor->input_buffer[i].imag = input->samples[i].imag * window_val;
    }

    // Zero pad if necessary
    for (size_t i = copy_length; i < processor->fft_size; i++) {
        processor->input_buffer[i].real = 0.0;
        processor->input_buffer[i].imag = 0.0;
    }

    // Perform FFT (using simple implementation)
    simple_fft(processor->input_buffer, processor->fft_size, false);

    // Copy result to output buffer
    for (size_t i = 0; i < processor->fft_size; i++) {
        output->samples[i].real = processor->input_buffer[i].real * processor->scale_factor;
        output->samples[i].imag = processor->input_buffer[i].imag * processor->scale_factor;
    }

    output->center_frequency = input->center_frequency;
    output->bandwidth = input->sample_rate;

    return true;
}

bool generate_window(WindowType window_type, uint32_t window_length, double* coefficients) {
    if (!coefficients || window_length == 0) {
        return false;
    }

    switch (window_type) {
        case WINDOW_RECTANGULAR:
            for (uint32_t i = 0; i < window_length; i++) {
                coefficients[i] = 1.0;
            }
            break;

        case WINDOW_HAMMING:
            generate_hamming_window(coefficients, window_length);
            break;

        case WINDOW_HANNING:
            for (uint32_t i = 0; i < window_length; i++) {
                coefficients[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (window_length - 1)));
            }
            break;

        case WINDOW_BLACKMAN:
            for (uint32_t i = 0; i < window_length; i++) {
                double n = (double)i / (window_length - 1);
                coefficients[i] = 0.42 - 0.5 * cos(2.0 * M_PI * n) + 0.08 * cos(4.0 * M_PI * n);
            }
            break;

        default:
            // Default to rectangular window
            for (uint32_t i = 0; i < window_length; i++) {
                coefficients[i] = 1.0;
            }
            break;
    }

    return true;
}

bool apply_window(SignalBuffer* signal, const double* window_coeffs, uint32_t window_length) {
    if (!signal || !window_coeffs || !signal->is_allocated) {
        return false;
    }

    uint32_t apply_length = (signal->length < window_length) ? signal->length : window_length;

    for (uint32_t i = 0; i < apply_length; i++) {
        signal->samples[i].real *= window_coeffs[i];
        signal->samples[i].imag *= window_coeffs[i];
    }

    return true;
}

bool is_valid_fft_size(uint32_t size) {
    return IS_POWER_OF_2(size) && size >= 2 && size <= MAX_FFT_SIZE;
}

uint32_t get_next_fft_size(uint32_t min_size) {
    return NEXT_POWER_OF_2(min_size);
}

double bin_to_frequency(uint32_t bin, double sample_rate, uint32_t fft_size) {
    if (fft_size == 0) return 0.0;
    
    double freq = (double)bin * sample_rate / fft_size;
    
    // Handle negative frequencies (second half of spectrum)
    if (bin > fft_size / 2) {
        freq -= sample_rate;
    }
    
    return freq;
}

uint32_t frequency_to_bin(double frequency, double sample_rate, uint32_t fft_size) {
    if (sample_rate <= 0.0 || fft_size == 0) return 0;
    
    double normalized_freq = frequency * fft_size / sample_rate;
    
    // Handle negative frequencies
    if (normalized_freq < 0) {
        normalized_freq += fft_size;
    }
    
    return (uint32_t)(normalized_freq + 0.5); // Round to nearest bin
}

void spectral_analysis_cleanup(SpectralAnalysis* analysis) {
    if (!analysis) return;
    
    free(analysis->frequency_bins);
    free(analysis->magnitude_spectrum);
    free(analysis->phase_spectrum);
    free(analysis->power_spectrum);
    
    memset(analysis, 0, sizeof(SpectralAnalysis));
}

// Helper functions
static void generate_hamming_window(double* window, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        window[i] = 0.54 - 0.46 * cos(2.0 * M_PI * i / (length - 1));
    }
}

static void simple_fft(ComplexSample* data, int n, bool inverse) {
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
        double angle = (inverse ? 2.0 : -2.0) * M_PI / len;
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

    if (inverse) {
        for (int i = 0; i < n; i++) {
            data[i] = complex_scale(data[i], 1.0 / n);
        }
    }
}
