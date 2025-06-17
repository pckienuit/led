/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FFT Audio Spectrum Analyzer - 200Hz to 4000Hz
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "lcd_parallel.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LED 60
#define USE_BRIGHTNESS 1
#define DEFAULT_BRIGHTNESS 10

// FFT Configuration - Tối ưu cho dải 200Hz-4000Hz
#define FFT_SIZE 128     // Tăng lên 128 để có độ phân giải tốt hơn
#define FFT_SIZE_LOG2 7    // log2(128) = 7
#define SAMPLE_RATE 10000  // 10kHz sampling để cover 4000Hz (Nyquist)
#define FREQ_BANDS 10      // 10 cột LED
#define NOISE_FLOOR 25     // Ngưỡng nhiễu thấp hơn

// Target frequency range
#define MIN_FREQ 200       // 200Hz
#define MAX_FREQ 4000      // 4000Hz
#define FREQ_RESOLUTION (SAMPLE_RATE / FFT_SIZE)  // ~78.125 Hz per bin

// Calculate bin indices for target frequency range
#define MIN_BIN (MIN_FREQ / FREQ_RESOLUTION)      // Bin ~2.56 ≈ 3
#define MAX_BIN (MAX_FREQ / FREQ_RESOLUTION)      // Bin ~51.2 ≈ 51
#define USEFUL_BINS (MAX_BIN - MIN_BIN + 1)       // ~48 bins

// Matrix LED definitions
#define MATRIX_ROWS 3
#define MATRIX_COLS 10

// Processing parameters
#define SMOOTHING_FACTOR 0.7f  // Tăng để mượt hơn
#define AGC_FACTOR 0.8f         // AGC mạnh hơn
#define PEAK_HOLD_TIME 300      // Giảm thời gian hold peak

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
uint32_t var = 0;

// FFT Variables - Tăng kích thước cho độ phân giải tốt hơn
int16_t adc_buffer[FFT_SIZE];
int16_t fft_real[FFT_SIZE];
int16_t fft_imag[FFT_SIZE];
uint16_t magnitude[FFT_SIZE/2];
uint16_t freq_bands[FREQ_BANDS];
uint16_t prev_bands[FREQ_BANDS];
uint16_t peak_bands[FREQ_BANDS];
uint32_t peak_time[FREQ_BANDS];

// Sampling variables
uint8_t sample_index = 0;
volatile uint8_t fft_ready = 0;
uint16_t max_magnitude = 0;

// AGC variables
uint16_t gain_factor = 256;   // Q8.8 format
uint16_t signal_rms = 0;

// LED Control
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];
uint16_t pwmData[(24*MAX_LED)+50];
int datasentflag = 0;
volatile int current_brightness = 50;

// Bit-reverse lookup table for FFT 128
const uint8_t bit_reverse_7[128] = {
    0, 64, 32, 96, 16, 80, 48, 112, 8, 72, 40, 104, 24, 88, 56, 120,
    4, 68, 36, 100, 20, 84, 52, 116, 12, 76, 44, 108, 28, 92, 60, 124,
    2, 66, 34, 98, 18, 82, 50, 114, 10, 74, 42, 106, 26, 90, 58, 122,
    6, 70, 38, 102, 22, 86, 54, 118, 14, 78, 46, 110, 30, 94, 62, 126,
    1, 65, 33, 97, 17, 81, 49, 113, 9, 73, 41, 105, 25, 89, 57, 121,
    5, 69, 37, 101, 21, 85, 53, 117, 13, 77, 45, 109, 29, 93, 61, 125,
    3, 67, 35, 99, 19, 83, 51, 115, 11, 75, 43, 107, 27, 91, 59, 123,
    7, 71, 39, 103, 23, 87, 55, 119, 15, 79, 47, 111, 31, 95, 63, 127
};

// Extended sine/cosine lookup tables for 128-point FFT
const int16_t sin_table[32] = {
    0, 6393, 12539, 18204, 23170, 27245, 30273, 32137,
    32767, 32137, 30273, 27245, 23170, 18204, 12539, 6393,
    0, -6393, -12539, -18204, -23170, -27245, -30273, -32137,
    -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6393
};

const int16_t cos_table[32] = {
    32767, 32137, 30273, 27245, 23170, 18204, 12539, 6393,
    0, -6393, -12539, -18204, -23170, -27245, -30273, -32137,
    -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6393,
    0, 6393, 12539, 18204, 23170, 27245, 30273, 32137
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* FFT Functions
void fft_bit_reverse(void);
void fft_butterfly(uint8_t stage, uint8_t step);
void perform_fft(void);
void calculate_magnitude(void);
void process_frequency_bands(void);
void apply_agc(void);

// LED Functions
void Set_LED(int LEDnum, int Red, int Green, int Blue);
void Set_LED_Matrix(int row, int col, int Red, int Green, int Blue);
void Set_LED_Col(int col_num, int Red, int Green, int Blue);
void Set_Brightness(int brightness);
void WS2812_Send(void);

// Audio Processing
void sample_audio(void);
void display_spectrum(void);
void get_band_color(uint8_t band, uint8_t intensity, uint8_t* r, uint8_t* g, uint8_t* b);
*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Fast integer square root for magnitude calculation
uint16_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    
    uint16_t x = n;
    uint16_t y = (x + 1) / 2;
    
    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

// Bit-reverse for 128-point FFT
void fft_bit_reverse(void) {
    for(uint8_t i = 0; i < FFT_SIZE; i++) {
        uint8_t j = bit_reverse_7[i];
        if(i < j) {
            // Swap real parts
            int16_t temp = fft_real[i];
            fft_real[i] = fft_real[j];
            fft_real[j] = temp;
            
            // Swap imaginary parts
            temp = fft_imag[i];
            fft_imag[i] = fft_imag[j];
            fft_imag[j] = temp;
        }
    }
}

// Get twiddle factors from extended lookup table
void get_twiddle(uint8_t angle_idx, int16_t* cos_val, int16_t* sin_val) {
    *cos_val = cos_table[angle_idx & 31];
    *sin_val = sin_table[angle_idx & 31];
}

// FFT butterfly operation
void fft_butterfly(uint8_t stage, uint8_t step) {
    uint8_t half_step = step >> 1;
    
    for(uint8_t i = 0; i < FFT_SIZE; i += step) {
        for(uint8_t j = 0; j < half_step; j++) {
            uint8_t idx1 = i + j;
            uint8_t idx2 = idx1 + half_step;
            
            // Calculate twiddle factor index
            uint8_t angle_idx = (j << (FFT_SIZE_LOG2 - stage - 1)) & 31;
            int16_t cos_val, sin_val;
            get_twiddle(angle_idx, &cos_val, &sin_val);
            
            // Complex multiplication with Q15 arithmetic
            int32_t temp_real = ((int32_t)fft_real[idx2] * cos_val + 
                                (int32_t)fft_imag[idx2] * sin_val) >> 15;
            int32_t temp_imag = ((int32_t)fft_imag[idx2] * cos_val - 
                                (int32_t)fft_real[idx2] * sin_val) >> 15;
            
            // Butterfly operation
            int16_t real1 = fft_real[idx1];
            int16_t imag1 = fft_imag[idx1];
            
            fft_real[idx1] = real1 + (int16_t)temp_real;
            fft_imag[idx1] = imag1 + (int16_t)temp_imag;
            fft_real[idx2] = real1 - (int16_t)temp_real;
            fft_imag[idx2] = imag1 - (int16_t)temp_imag;
        }
    }
}

// Main FFT computation
void perform_fft(void) {
    // Bit-reverse input
    fft_bit_reverse();
    
    // Perform FFT stages
    for(uint8_t stage = 0; stage < FFT_SIZE_LOG2; stage++) {
        uint8_t step = 1 << (stage + 1);
        fft_butterfly(stage, step);
    }
}

// Calculate magnitude using optimized method
void calculate_magnitude(void) {
    max_magnitude = 0;
    
    for(uint8_t i = 1; i < FFT_SIZE/2; i++) { // Skip DC component
        int32_t real = fft_real[i];
        int32_t imag = fft_imag[i];
        
        // Fast magnitude approximation
        int32_t abs_real = (real < 0) ? -real : real;
        int32_t abs_imag = (imag < 0) ? -imag : imag;
        
        // |z| ≈ max(|a|,|b|) + 0.5*min(|a|,|b|)
        int32_t max_val = (abs_real > abs_imag) ? abs_real : abs_imag;
        int32_t min_val = (abs_real < abs_imag) ? abs_real : abs_imag;
        
        magnitude[i] = (uint16_t)(max_val + (min_val >> 1));
        
        if(magnitude[i] > max_magnitude) {
            max_magnitude = magnitude[i];
        }
    }
}

// Apply automatic gain control
void apply_agc(void) {
    if(max_magnitude > 0) {
        // Target amplitude (Q15)
        uint16_t target = 16384; // 50% of max
        
        // Calculate new gain factor
        if(max_magnitude > target) {
            gain_factor = (gain_factor * AGC_FACTOR * target) / max_magnitude / AGC_FACTOR;
        } else if(max_magnitude < target/2) {
            gain_factor = (gain_factor * target) / max_magnitude;
        }
        
        // Limit gain factor
        if(gain_factor > 2048) gain_factor = 2048; // Max 8x gain
        if(gain_factor < 64) gain_factor = 64;     // Min 0.25x gain
        
        // Apply gain to magnitude array
        for(uint8_t i = 1; i < FFT_SIZE/2; i++) {
            uint32_t temp = (uint32_t)magnitude[i] * gain_factor;
            magnitude[i] = (temp >> 8); // Q8.8 to Q16
            if(magnitude[i] > 32767) magnitude[i] = 32767;
        }
    }
}

// Process frequency bands optimized for 200Hz-4000Hz
void process_frequency_bands(void) {
    uint32_t current_time = HAL_GetTick();
    
    // Define frequency band ranges for 200Hz-4000Hz (logarithmic distribution)
    const uint8_t band_ranges[FREQ_BANDS][2] = {
        {3, 5},    // 200-390Hz     (Low Bass)
        {5, 7},    // 390-547Hz     (Bass)
        {7, 10},   // 547-781Hz     (Low Mid)
        {10, 14},  // 781-1094Hz    (Mid)
        {14, 19},  // 1094-1484Hz   (High Mid)
        {19, 25},  // 1484-1953Hz   (Presence)
        {25, 32},  // 1953-2500Hz   (High)
        {32, 40},  // 2500-3125Hz   (Treble)
        {40, 48},  // 3125-3750Hz   (High Treble)
        {48, 51}   // 3750-4000Hz   (Air)
    };
    
    for(uint8_t band = 0; band < FREQ_BANDS; band++) {
        uint32_t band_sum = 0;
        uint8_t bin_count = 0;
        
        // Sum magnitudes in frequency band
        for(uint8_t bin = band_ranges[band][0]; bin <= band_ranges[band][1]; bin++) {
            if(bin < FFT_SIZE/2) {
                // Apply frequency weighting for better visualization
                uint16_t weighted_mag = magnitude[bin];
                
                // Boost mid frequencies (1-3kHz) slightly
                if(bin >= 13 && bin <= 38) { // ~1000-3000Hz
                    weighted_mag = (weighted_mag * 120) / 100;
                }
                
                band_sum += weighted_mag;
                bin_count++;
            }
        }
        
        // Average and normalize
        uint16_t avg_magnitude = bin_count > 0 ? band_sum / bin_count : 0;
        
        // Apply noise floor
        if(avg_magnitude < NOISE_FLOOR) {
            avg_magnitude = 0;
        }
        
        // Logarithmic scaling for better dynamic range
        if(avg_magnitude > 0) {
            // Simple log approximation: log2(x) ≈ MSB position
            uint16_t log_mag = 0;
            uint16_t temp = avg_magnitude;
            while(temp >>= 1) {
                log_mag++;
            }
            avg_magnitude = (log_mag * avg_magnitude) / 16;
        }
        
        // Enhanced smoothing with previous values
        freq_bands[band] = (uint16_t)(SMOOTHING_FACTOR * avg_magnitude + 
                          (1.0f - SMOOTHING_FACTOR) * prev_bands[band]);
        
        // Peak hold with faster decay
        if(freq_bands[band] > peak_bands[band]) {
            peak_bands[band] = freq_bands[band];
            peak_time[band] = current_time;
        } else if(current_time - peak_time[band] > PEAK_HOLD_TIME) {
            // Exponential decay for smoother peak fall
            if(peak_bands[band] > 0) {
                peak_bands[band] = (peak_bands[band] * 95) / 100;
            }
        }
        
        prev_bands[band] = freq_bands[band];
    }
}

// Enhanced color mapping for 200Hz-4000Hz spectrum
void get_band_color(uint8_t band, uint8_t intensity, uint8_t* r, uint8_t* g, uint8_t* b) {
    // Enhanced color mapping for audio spectrum
    switch(band) {
        case 0: // 200-390Hz - Deep Purple (Sub Bass)
            *r = intensity / 2;
            *g = 0;
            *b = intensity;
            break;
            
        case 1: // 390-547Hz - Blue (Bass)
            *r = 0;
            *g = intensity / 4;
            *b = intensity;
            break;
            
        case 2: // 547-781Hz - Cyan (Low Mid)
            *r = 0;
            *g = intensity / 2;
            *b = intensity;
            break;
            
        case 3: // 781-1094Hz - Light Blue
            *r = 0;
            *g = (intensity * 3) / 4;
            *b = intensity;
            break;
            
        case 4: // 1094-1484Hz - Green (Mid)
            *r = 0;
            *g = intensity;
            *b = intensity / 3;
            break;
            
        case 5: // 1484-1953Hz - Yellow Green
            *r = intensity / 3;
            *g = intensity;
            *b = 0;
            break;
            
        case 6: // 1953-2500Hz - Yellow (Presence)
            *r = (intensity * 2) / 3;
            *g = intensity;
            *b = 0;
            break;
            
        case 7: // 2500-3125Hz - Orange
            *r = intensity;
            *g = (intensity * 2) / 3;
            *b = 0;
            break;
            
        case 8: // 3125-3750Hz - Red Orange
            *r = intensity;
            *g = intensity / 3;
            *b = 0;
            break;
            
        case 9: // 3750-4000Hz - Red (High)
            *r = intensity;
            *g = 0;
            *b = intensity / 4;
            break;
    }
}

// Sample audio data with optimized timing for 10kHz
void sample_audio(void) {
    static uint32_t last_sample = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Sample at ~10kHz (every 0.1ms = 100μs)
    // Using HAL_GetTick() provides 1ms resolution, so we sample as fast as possible
    if(current_time != last_sample || (HAL_GetTick() == last_sample)) {
        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
            uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Stop(&hadc1);
            
            // Convert to signed and apply pre-emphasis for high frequencies
            int16_t sample = (int16_t)(adc_val - 2048);
            
            // Simple high-pass filter to remove DC and enhance mid-high frequencies
            static int16_t prev_sample = 0;
            sample = sample - (prev_sample >> 4); // Mild high-pass
            prev_sample = sample;
            
            adc_buffer[sample_index] = sample;
            sample_index++;
            
            // When buffer is full, trigger FFT processing
            if(sample_index >= FFT_SIZE) {
                sample_index = 0;
                fft_ready = 1;
            }
        }
        last_sample = current_time;
    }
}

// Enhanced spectrum display with better scaling
void display_spectrum(void) {
    // Clear all LEDs
    for(uint8_t i = 0; i < MAX_LED; i++) {
        Set_LED(i, 0, 0, 0);
    }
    
    // Display each frequency band
    for(uint8_t band = 0; band < FREQ_BANDS; band++) {
        // Enhanced scaling for better visualization
        uint16_t scaled = freq_bands[band] >> 4; // Divide by 16 for scaling
        if(scaled > 255) scaled = 255;
        
        uint8_t intensity = (uint8_t)scaled;
        
        if(intensity > 10) { // Lower threshold for more sensitivity
            // Calculate number of LEDs to light in this column (0-3)
            uint8_t num_leds = (intensity * MATRIX_ROWS) / 255;
            if(num_leds > MATRIX_ROWS) num_leds = MATRIX_ROWS;
            
            // Get color for this band
            uint8_t r, g, b;
            get_band_color(band, intensity, &r, &g, &b);
            
            // Light LEDs from bottom up with gradient effect
            for(uint8_t led = 0; led < num_leds; led++) {
                uint8_t row = MATRIX_ROWS - 1 - led;
                
                // Apply brightness gradient (bottom brighter than top)
                uint8_t gradient = 255 - (led * 60); // Reduce brightness by 60 for each level
                
                Set_LED_Matrix(row, band, 
                              (r * gradient) / 255,
                              (g * gradient) / 255,
                              (b * gradient) / 255);
            }
            
            // Show peak with white color
            uint16_t peak_scaled = peak_bands[band] >> 4;
            if(peak_scaled > 255) peak_scaled = 255;
            uint8_t peak_leds = (peak_scaled * MATRIX_ROWS) / 255;
            
            if(peak_leds < MATRIX_ROWS && peak_leds > num_leds) {
                Set_LED_Matrix(MATRIX_ROWS - 1 - peak_leds, band, 255, 255, 255);
            }
        }
    }
    
    Set_Brightness(current_brightness);
    WS2812_Send();
}

void Set_LED(int LEDnum, int Red, int Green, int Blue) {
    if(LEDnum >= 0 && LEDnum < MAX_LED) {
        LED_Data[LEDnum][0] = LEDnum;
        LED_Data[LEDnum][1] = Green;
        LED_Data[LEDnum][2] = Red;
        LED_Data[LEDnum][3] = Blue;
    }
}

void Set_LED_Matrix(int row, int col, int Red, int Green, int Blue) {
    if(row >= 0 && row < 3 && col >= 0 && col < 10) {
        const int LED_Matrix[3][10] = {
            {40, 39, 38, 37, 36, 35, 34, 33, 32, 31},
            {16, 17, 18, 19, 20, 21, 22, 23, 24, 25},
            {11, 10,  9,  8,  7,  6,  5,  4,  3,  2},
        };
        Set_LED(LED_Matrix[row][col], Red, Green, Blue);
    }
}

void Set_LED_Col(int col_num, int Red, int Green, int Blue) {
    if(col_num >= 0 && col_num < 10) {
        Set_LED(40-col_num, Red, Green, Blue); //top, from right
        Set_LED(16+col_num, Red, Green, Blue); //mid, from left
        Set_LED(11-col_num, Red, Green, Blue); //bot, from right
    }
}

void Set_Brightness(int brightness) {
    #if USE_BRIGHTNESS
    if (brightness > 100) brightness = 100;
    if (brightness < 0) brightness = 0;

    for (int i = 0; i < MAX_LED; i++) {
        LED_Mod[i][0] = LED_Data[i][0];
        for (int j = 1; j < 4; j++) {
            LED_Mod[i][j] = (LED_Data[i][j] * brightness) / 100;
        }
    }
    #endif
}

void WS2812_Send(void) {
    uint32_t indx = 0;
    uint32_t color;

    for (int i = 0; i < MAX_LED; i++) {
        color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));

        for (int j = 23; j >= 0; j--) {
            pwmData[indx++] = (color & (1 << j)) ? 60 : 30;
        }
    }

    for (int i = 0; i < 50; i++) {
        pwmData[indx++] = 0;
    }

    datasentflag = 0;
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);

    uint32_t timeout = 100000;
    while(!datasentflag && timeout--);

    if(!timeout) {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
        datasentflag = 1;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        datasentflag = 1;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LCD
  LCD_Parallel_Init();
  LCD_Parallel_Clear();
  LCD_Parallel_Print("Audio 200-4000Hz");
  HAL_Delay(1000);

  // Initialize variables
  for(uint8_t i = 0; i < FREQ_BANDS; i++) {
      freq_bands[i] = 0;
      prev_bands[i] = 0;
      peak_bands[i] = 0;
      peak_time[i] = 0;
  }
  
  sample_index = 0;
  fft_ready = 0;
  gain_factor = 256;

  // Clear LED strip
  for(uint8_t i = 0; i < MAX_LED; i++) {
      Set_LED(i, 0, 0, 0);
  }
  Set_Brightness(0);
  WS2812_Send();

//  LCD_Parallel_Clear();
//  LCD_Parallel_SetCursor(0, 0);
//  LCD_Parallel_Print("Range:200-4000Hz");
//  LCD_Parallel_SetCursor(1, 0);
//  LCD_Parallel_Print("Res:78Hz/bin");
//  HAL_Delay(1000);
  
  LCD_Parallel_Clear();
  LCD_Parallel_Print("Ready!");
  //HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Sample audio continuously at high rate
    sample_audio();
    
    // Process FFT when buffer is full
    if(fft_ready) {
        fft_ready = 0;
        
        // Copy ADC data to FFT input and clear imaginary
        for(uint8_t i = 0; i < FFT_SIZE; i++) {
            fft_real[i] = adc_buffer[i];
            fft_imag[i] = 0;
        }
        
        // Perform FFT
        perform_fft();
        
        // Calculate magnitude
        calculate_magnitude();
        
        // Apply AGC
        apply_agc();
        
        // Process frequency bands for 200Hz-4000Hz
        process_frequency_bands();
        
        // Display on LEDs
        display_spectrum();
    }
    
    // No delay for maximum sampling rate
    
    /* USER CODE END WHILE */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 90-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.RepetitionCounter = 0;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim3, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
