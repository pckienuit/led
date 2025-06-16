/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LED 60
#define USE_BRIGHTNESS 1
#define DEFAULT_BRIGHTNESS 10

// Keypad defines
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

// Effect states - Updated for 5 modes: fade, rainbow, run, flashing, music
#define EFFECT_FADE      0  // Fade effect
#define EFFECT_RAINBOW   1  // Rainbow effect (no color change needed)
#define EFFECT_RUN       2  // Pixel run effect
#define EFFECT_FLASHING  3  // Flashing effect
#define EFFECT_OFF       4
#define EFFECT_MUSIC     5  // Music reactive effect
#define MAX_EFFECTS      6

// Color definitions
#define COLOR_BLUE   0  // Blue
#define COLOR_RED    1  // Red
#define COLOR_PINK   2  // Pink
#define COLOR_GREEN  3  // Green
#define COLOR_PURPLE 4  // Purple
#define COLOR_YELLOW 5  // Yellow
#define COLOR_CYAN   6  // Cyan
#define COLOR_WHITE  7  // White
#define COLOR_ORANGE 8  // Orange
#define MAX_COLORS   9

// Color values [R, G, B]
int color_values[MAX_COLORS][3] = {
    {0, 100, 255},    // Blue
    {255, 0, 0},      // Red
    {255, 20, 147},   // Pink
    {0, 255, 0},      // Green
    {128, 0, 128},    // Purple
    {255, 255, 0},    // Yellow
    {0, 255, 255},    // Cyan
    {255, 255, 255},  // White
    {255, 165, 0}     // Orange
};

// GYMAX4466 Sound Detection
#define MUSIC_OUT_PORT     GPIOC
#define MUSIC_OUT_PIN      GPIO_PIN_15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
uint32_t var = 0;
// LCD display variables
volatile int lcd_update_needed = 1;
uint32_t last_lcd_update = 0;
#define LCD_UPDATE_INTERVAL 500  // Update LCD every 500ms

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to convert integer to string
char* int_to_string(int num, char* str) {
    int i = 0;
    int is_negative = 0;
    
    // Handle negative numbers
    if (num < 0) {
        is_negative = 1;
        num = -num;
    }
    
    // Handle zero
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }
    
    // Convert number to string in reverse order
    while (num > 0) {
        str[i++] = num % 10 + '0';
        num = num / 10;
    }
    
    // Add negative sign if needed
    if (is_negative) {
        str[i++] = '-';
    }
    
    // Reverse the string
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
    
    // Add null terminator
    str[i] = '\0';
    return str;
}

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];

int datasentflag = 0;

// Keypad variables - Reversed column order to avoid PA3 issue
GPIO_TypeDef* keypad_row_ports[KEYPAD_ROWS] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t keypad_row_pins[KEYPAD_ROWS] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7}; // Row 0,1,2,3
GPIO_TypeDef* keypad_col_ports[KEYPAD_COLS] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t keypad_col_pins[KEYPAD_COLS] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3}; // Col 0,1,2,3 (PA0,PA1,PA2,PA3)

// Keypad layout - Updated for reversed column order
char keypad_layout[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'3', '2', '1', '0'},    // S3, S2, S1, S0 (reversed)
    {'7', '6', '5', '4'},    // S7, S6, S5, S4
    {'B', 'A', '9', '8'},    // S11, S10, S9, S8
    {'F', 'E', 'D', 'C'}     // S15, S14, S13, S12
};

// Effect control variables
volatile int current_effect = EFFECT_FLASHING;
volatile int effect_changed = 1;
volatile int effect_running = 0;

// Color and speed control
volatile int current_color = COLOR_BLUE;  // Start with blue
volatile int current_speed = 5;           // Speed from 1 (slow) to 10 (fast)
volatile int current_brightness = 50;     // Brightness from 1 (dim) to 100 (bright)

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag = 1;
}

// Keypad functions
void Keypad_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure row pins as output
    for(int i = 0; i < KEYPAD_ROWS; i++) {
        GPIO_InitStruct.Pin = keypad_row_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(keypad_row_ports[i], &GPIO_InitStruct);
        HAL_GPIO_WritePin(keypad_row_ports[i], keypad_row_pins[i], GPIO_PIN_SET);
    }

    // Configure column pins as input with pull-up
    for(int i = 0; i < KEYPAD_COLS; i++) {
        GPIO_InitStruct.Pin = keypad_col_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(keypad_col_ports[i], &GPIO_InitStruct);
    }
}

void Update_LCD_Display(void) {
    // Effect names
    const char* effect_names[] = {"Fade", "Rainbow", "Run", "Flash", "Off", "Music"};
    const char* color_names[] = {"Blue", "Red", "Pink", "Green", "Purple", "Yellow", "Cyan", "White", "Orange"};

    // Display current effect
    if(current_effect < MAX_EFFECTS) {
        LCD_Parallel_DisplayEffect(effect_names[current_effect]);
    }

    // Display status (color, speed, brightness)
    if(current_effect == EFFECT_RAINBOW) {
        LCD_Parallel_DisplayStatus("Rainbow", current_speed, current_brightness);
    } else if(current_effect == EFFECT_OFF) {
        LCD_Parallel_DisplayStatus("OFF", 0, 0);
    } else if(current_color < MAX_COLORS) {
        LCD_Parallel_DisplayStatus(color_names[current_color], current_speed, current_brightness);
    }
}

char Keypad_Read(void) {
    for(int row = 0; row < KEYPAD_ROWS; row++) {
        // Set current row LOW, others HIGH
        for(int i = 0; i < KEYPAD_ROWS; i++) {
            if(i == row) {
                HAL_GPIO_WritePin(keypad_row_ports[i], keypad_row_pins[i], GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(keypad_row_ports[i], keypad_row_pins[i], GPIO_PIN_SET);
            }
        }

        HAL_Delay(2);

        // Read columns
        for(int col = 0; col < KEYPAD_COLS; col++) {
            if(HAL_GPIO_ReadPin(keypad_col_ports[col], keypad_col_pins[col]) == GPIO_PIN_RESET) {
                // Button pressed, wait for release
                while(HAL_GPIO_ReadPin(keypad_col_ports[col], keypad_col_pins[col]) == GPIO_PIN_RESET) {
                    HAL_Delay(10);
                }
                HAL_Delay(50); // Debounce delay
                return keypad_layout[row][col];
            }
        }
    }
    return 0; // No key pressed
}

void Process_Keypad_Input(char key) {
    switch(key) {
        // Effects
        case '1':  // S1 - Fade effect
            current_effect = EFFECT_FADE;
            effect_changed = 1;
            lcd_update_needed = 1;
            break;
        case '5':  // S5 - Rainbow effect
            current_effect = EFFECT_RAINBOW;
            effect_changed = 1;
            lcd_update_needed = 1;
            break;
        case '9':  // S9 - Run effect
            current_effect = EFFECT_RUN;
            effect_changed = 1;
            lcd_update_needed = 1;
            break;
        case 'D':  // S13 - Flashing effect
            current_effect = EFFECT_FLASHING;
            effect_changed = 1;
            lcd_update_needed = 1;
            break;

        // Speed control
        case '2':  // S2 - Decrease speed
            if(current_speed > 1) current_speed--;
            lcd_update_needed = 1;
            break;
        case '3':  // S3 - Increase speed
            if(current_speed < 10) current_speed++;
            lcd_update_needed = 1;
            break;

        // Brightness control
        case '6':  // S6 - Decrease brightness
            current_brightness -= 10;
            if(current_brightness < 1) current_brightness = 1;
            lcd_update_needed = 1;
            break;
        case '7':  // S7 - Increase brightness
            current_brightness += 10;
            if(current_brightness > 100) current_brightness = 100;
            lcd_update_needed = 1;
            break;

        // Color and control
        case 'A':  // S10 - Change color (not for Rainbow)
            if(current_effect != EFFECT_RAINBOW) {
                current_color = (current_color + 1) % MAX_COLORS;
                lcd_update_needed = 1;
            }
            break;
        case 'E':  // S14 - Music mode
            current_effect = EFFECT_MUSIC;
            effect_changed = 1;
           // music_mode_active = 1;
            lcd_update_needed = 1;
            break;
        case 'F':  // S15 - Turn off
            current_effect = EFFECT_OFF;
            effect_changed = 1;
            lcd_update_needed = 1;
            break;

        // Disabled problematic buttons
        case '0':  // S0 - Disabled (problematic)
        case '4':  // S4 - Disabled (problematic)
        case '8':  // S8 - Disabled (problematic)
        case 'C':  // S12 - Disabled (problematic)
        default:
            // Do nothing for problematic keys and others
            break;
    }
}

void Set_LED (int LEDnum, int Red, int Green, int Blue) {
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

// Function to set all LEDs to the same color
void Set_All_LEDs_Same_Color(int Red, int Green, int Blue) {
    for (int i = 0; i < MAX_LED; i++) {
        Set_LED(i, Red, Green, Blue);
    }
}

void Set_LED_Col (int col_num, int Red, int Green, int Blue) {
	Set_LED(40-col_num, Red, Green, Blue); //top, from right
	Set_LED(16+col_num, Red, Green, Blue); //mid, from left
	Set_LED(11-col_num, Red, Green, Blue);  //bot, from right
}

void Set_LED_Row (int row_num, int Red, int Green, int Blue) {
	switch(row_num){
	case 0:
		for (int i = 40; i >= 31; --i)
			Set_LED(i, Red, Green, Blue);
		break;
	case 1:
		for (int i = 16; i <= 25; ++i)
			Set_LED(i, Red, Green, Blue);
		break;
	case 2:
		for (int i = 11; i >= 2; --i)
			Set_LED(i, Red, Green, Blue);
		break;
	}
}

void Set_LED_Matrix (int row, int col, int Red, int Green, int Blue) {
	int LED_Matrix[3][10] = {
			{40, 39, 38, 37, 36, 35, 34, 33, 32, 31},
			{16, 17, 18, 19, 20, 21, 22, 23, 24, 25},
			{11, 10,  9,  8,  7,  6,  5,  4,  3,  2},
	};
	Set_LED(LED_Matrix[row][col], Red, Green, Blue);
}

void Set_Brightness (int brightness) {
#if USE_BRIGHTNESS

	if (brightness > 100) brightness = 100;
	if (brightness < 0) brightness = 0;

	for (int i=0; i < MAX_LED; i++) {
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j = 1; j < 4; ++j) {
			// Simple percentage calculation: brightness from 0-100%
			LED_Mod[i][j] = (LED_Data[i][j] * brightness) / 100;
		}
	}

#endif
}

uint16_t pwmData[(24*MAX_LED)+50];

//void send(int Green, int Red, int Blue) {
//	uint32_t data = (Green<<16) | (Red<<8) | Blue;
//
//	for (int i = 23; i >= 0; i--) {
//		if (data&(1<<i)) pwmData[i] = 60;
//		else pwmData[i] = 30;
//	}
//
//	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, 24);
//}


void WS2812_Send (void) {
	uint32_t indx = 0;
	uint32_t color;

	for (int i = 0; i < MAX_LED; ++i) {
		color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));

		for (int j = 23; j >= 0; j--) {
			if (color&(1<<j)) {
				pwmData[indx] = 60;
			} else {
				pwmData[indx] = 30;
			}

			indx++;
		}
	}

	for (int i = 0; i < 50; ++i) {
		pwmData[indx] = 0;
		indx++;
	}

	// Reset flag before starting DMA
	datasentflag = 0;

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);

	// Add timeout to prevent infinite loop
	uint32_t timeout = 1000000;  // Adjust timeout value
	while(!datasentflag && timeout > 0) {
		timeout--;
	}

	// Force stop if timeout occurred
	if (timeout == 0) {
		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
		datasentflag = 1;
	}

	datasentflag = 0;
}

#define LED_DELAY 10

void Fade_Effect (int Red, int Green, int Blue, int speed) { //speed 1 -> 10
	if (speed > 10) speed = 10;
	if (speed < 1)	speed = 1;

	Set_All_LEDs_Same_Color(Red, Green, Blue);

	for (int i = 0; i <= current_brightness; ++i) {  // Use current_brightness instead of 100
		Set_Brightness(i);
		WS2812_Send();
		HAL_Delay(LED_DELAY/speed);
	}

	for (int i = current_brightness; i >= 0; --i) {  // Use current_brightness instead of 100
		Set_Brightness(i);
		WS2812_Send();
		HAL_Delay(LED_DELAY/speed);
	}
}

void Rainbow_Effect_Single (int led_num, int speed) {
	if (speed > 10) speed = 10;
	if (speed < 1)	speed = 1;
//	for (int r = 0; r <= 255; r+=10) {
//		for (int g = 0; g <= 255; g+=10){
//			for (int b = 0; b <= 255; b+=10) {
//				Set_LED(1, r, g, b);
//				Set_Brightness(100);
//				WS2812_Send();
//				HAL_Delay(10);
//			}
//		}
//	}

	//use HSV -> RGB method

	// H (hue): 0 -> 360 (int)
	// S (saturation): 0 -> 1 (float)
	// V (value): 0 -> 1 (float)

	//C (chroma) = VxS
	//H' (hue prime) = H/60
	//X = Cx(1-fabs(H%2-1))

	//0 <= H' < 1: (C, X, 0)
	//1 <= H' < 2: (X, C, 0)
	//2 <= H' < 3: (0, C, X)
	//3 <= H' < 4: (0, X, C)
	//4 <= H' < 5: (X, 0, C)
	//5 <= H' < 6: (C, 0, X)

	for (int hue = 0; hue < 360; hue += 2) {
		int red, green, blue;

        int sector = hue / 60;
        int remainder = hue % 60;

        switch(sector) {
            case 0: red = 255; green = (remainder * 255) / 60; blue = 0; break;
            case 1: red = 255 - ((remainder * 255) / 60); green = 255; blue = 0; break;
            case 2: red = 0; green = 255; blue = (remainder * 255) / 60; break;
            case 3: red = 0; green = 255 - ((remainder * 255) / 60); blue = 255; break;
            case 4: red = (remainder * 255) / 60; green = 0; blue = 255; break;
            case 5: red = 255; green = 0; blue = 255 - ((remainder * 255) / 60); break;
            default: red = 255; green = 0; blue = 0; break;
        }

		//Set_All_LEDs_Same_Color(red, green, blue);

		Set_LED(led_num, red, green, blue);
		Set_Brightness(DEFAULT_BRIGHTNESS);
		WS2812_Send();
		HAL_Delay(LED_DELAY/speed);
	}
}

void Rainbow_Effect (int speed) {
	if (speed > 10) speed = 10;
	if (speed < 1)	speed = 1;

	for (int hue = 0; hue < 360; hue += 36) {
		int red, green, blue;

		for (int led_num = 0; led_num < MAX_LED; ++led_num) {
			if (hue > 359) hue = 0;
			hue += led_num;
			int sector = hue / 60;
			int remainder = hue % 60;

			switch(sector) {
				case 0: red = 255; green = (remainder * 255) / 60; blue = 0; break;
				case 1: red = 255 - ((remainder * 255) / 60); green = 255; blue = 0; break;
				case 2: red = 0; green = 255; blue = (remainder * 255) / 60; break;
				case 3: red = 0; green = 255 - ((remainder * 255) / 60); blue = 255; break;
				case 4: red = (remainder * 255) / 60; green = 0; blue = 255; break;
				case 5: red = 255; green = 0; blue = 255 - ((remainder * 255) / 60); break;
				default: red = 255; green = 0; blue = 0; break;
			}

			//Set_All_LEDs_Same_Color(red, green, blue);

			Set_LED(led_num, red, green, blue);
			HAL_Delay(1);
		}

		Set_Brightness(current_brightness);
		WS2812_Send();
		HAL_Delay(LED_DELAY/speed);

		// Check keypad during rainbow effect
		char key_rainbow = Keypad_Read();
		if(key_rainbow != 0) {
			Process_Keypad_Input(key_rainbow);
			Update_LCD_Display();
		}
	}
}

void Pixel_Run_Effect (int speed, int red, int green, int blue) {
	if (speed > 10) speed = 10;
	if (speed < 1)	speed = 1;

	Set_LED(MAX_LED-1, 0, 0 , 0);
	for (int i = 1; i < MAX_LED; ++i) {
		Set_LED(i, red, green, blue);
		Set_LED(i-1, 0, 0 , 0);
		Set_Brightness(current_brightness);
		WS2812_Send();
		HAL_Delay((LED_DELAY*3)/speed);

		// Check keypad during run effect
		char key_run = Keypad_Read();
		if(key_run != 0) {
			Process_Keypad_Input(key_run);
			Update_LCD_Display();
		}
	}
}

void Flashing_Effect (int speed, int red, int green, int blue) {
	if (speed > 10) speed = 10;
	if (speed < 1)	speed = 1;

	Set_All_LEDs_Same_Color(red, green, blue);
	Set_Brightness(DEFAULT_BRIGHTNESS);
	WS2812_Send();
	HAL_Delay((LED_DELAY*15)/speed);
	Set_Brightness(0);
	WS2812_Send();
	HAL_Delay((LED_DELAY*15)/speed);
}

// Test LCD function - để kiểm tra kết nối
void LCD_Test(void) {
    LCD_Parallel_Clear();
    LCD_Parallel_SetCursor(0, 0);
    LCD_Parallel_Print("Hello STM32!");
    LCD_Parallel_SetCursor(1, 0);
    LCD_Parallel_Print("LCD Working!");
    HAL_Delay(2000);
}

// All LEDs off (effect OFF)
void All_LEDs_Off(void) {
    // Tắt tất cả LED và đặt độ sáng về 0
    for(int i = 0; i < MAX_LED; i++) {
        Set_LED(i, 0, 0, 0);
    }
    Set_Brightness(0);
    WS2812_Send();
}

// Function to generate random color based on HAL_GetTick
void get_random_color(uint8_t* red, uint8_t* green, uint8_t* blue) {
    uint32_t tick = HAL_GetTick();
    // Use different parts of the tick for RGB
    *red = (tick >> 8) & 0xFF;
    *green = (tick >> 16) & 0xFF;
    *blue = tick & 0xFF;
}

// Function to convert HSV to RGB
void hsv_to_rgb(float h, float s, float v, uint8_t* r, uint8_t* g, uint8_t* b) {
    float c = v * s;
    float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    float m = v - c;
    
    float r1, g1, b1;
    if (h >= 0 && h < 60) {
        r1 = c; g1 = x; b1 = 0;
    } else if (h >= 60 && h < 120) {
        r1 = x; g1 = c; b1 = 0;
    } else if (h >= 120 && h < 180) {
        r1 = 0; g1 = c; b1 = x;
    } else if (h >= 180 && h < 240) {
        r1 = 0; g1 = x; b1 = c;
    } else if (h >= 240 && h < 300) {
        r1 = x; g1 = 0; b1 = c;
    } else {
        r1 = c; g1 = 0; b1 = x;
    }
    
    *r = (uint8_t)((r1 + m) * 255);
    *g = (uint8_t)((g1 + m) * 255);
    *b = (uint8_t)((b1 + m) * 255);
}

// Function to get temperature-based color (blue to red)
void get_temperature_color(float intensity, uint8_t* r, uint8_t* g, uint8_t* b) {
    // Map intensity (0-255) to temperature color
    // Blue (cold) -> Cyan -> Green -> Yellow -> Red (hot)
    if (intensity < 51) {  // 0-50: Blue to Cyan
        *r = 0;
        *g = intensity * 5;
        *b = 255;
    } else if (intensity < 102) {  // 51-101: Cyan to Green
        *r = 0;
        *g = 255;
        *b = 255 - (intensity - 51) * 5;
    } else if (intensity < 153) {  // 102-152: Green to Yellow
        *r = (intensity - 102) * 5;
        *g = 255;
        *b = 0;
    } else if (intensity < 204) {  // 153-203: Yellow to Orange
        *r = 255;
        *g = 255 - (intensity - 153) * 5;
        *b = 0;
    } else {  // 204-255: Orange to Red
        *r = 255;
        *g = 0;
        *b = 0;
    }
}

// Function to check if a position is valid and not adjacent to any lit LED
int is_valid_position(int row, int col, int* lit_positions, int num_lit) {
    // Check if position is within bounds
    if (row < 0 || row >= 3 || col < 0 || col >= 10) {
        return 0;
    }
    
    // Check if position is adjacent to any lit LED
    for (int i = 0; i < num_lit; i++) {
        int lit_row = lit_positions[i] / 10;
        int lit_col = lit_positions[i] % 10;
        
        // Check if positions are adjacent (including diagonals)
        if (abs(row - lit_row) <= 1 && abs(col - lit_col) <= 1) {
            return 0;
        }
    }
    
    return 1;
}

// Function to control LED based on sound frequency with random flashing effect
void Sound_Reactive_LED(uint32_t sound_value) {
    static uint32_t last_update = 0;
    static int lit_positions[30];  // Store positions of lit LEDs
    static int num_lit = 0;
    
    // Ignore sound below 400
    if (sound_value < 650) {
        Set_All_LEDs_Same_Color(0, 0, 0);
        Set_Brightness(current_brightness);
        WS2812_Send();
        return;
    }
    
    // Map sound value to LED intensity (400-4095 to 0-255)
    uint8_t intensity = ((sound_value - 650) * 255) / (4095 - 400);
    
    // Update effect every 50ms
    if (HAL_GetTick() - last_update > 10) {
        last_update = HAL_GetTick();
        
        // Calculate number of LEDs to light based on intensity
        int num_leds = (intensity * 15) / 255;  // Max 15 LEDs
        if (num_leds < 1) num_leds = 1;
        if (num_leds > 15) num_leds = 15;
        
        // Clear all LEDs
        Set_All_LEDs_Same_Color(0, 0, 0);
        
        // Reset lit positions
        num_lit = 0;
        
        // Try to light up new LEDs
        int attempts = 0;
        while (num_lit < num_leds && attempts < 100) {
            attempts++;
            
            // Generate random position
            int row = rand() % 3;
            int col = rand() % 10;
            
            // Check if position is valid
            if (is_valid_position(row, col, lit_positions, num_lit)) {
                // Store position
                lit_positions[num_lit] = row * 10 + col;
                num_lit++;
                
                // Get color based on intensity
                uint8_t r, g, b;
                get_temperature_color(intensity, &r, &g, &b);
                
                // Light up the LED
                Set_LED_Matrix(row, col, r, g, b);
            }
        }
    }
    
    // Apply brightness
    Set_Brightness(current_brightness);
    WS2812_Send();
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize keypad
  Keypad_Init();

  // Initialize LCD Parallel
  LCD_Parallel_Init();

  // Turn off all LEDs initially
  Set_All_LEDs_Same_Color(0, 0, 0);
  Set_Brightness(0);
  WS2812_Send();


  // Update LCD with initial status
  //Update_LCD_Display();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int mn = 5000, mx = 0, avg = 0, cnt = 0;
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_ADC_Start(&hadc1);
    HAL_Delay(10);
    var = HAL_ADC_GetValue(&hadc1);

    if (var > mx) mx = var;
    if (var < mn) mn = var;
    avg += var;
    cnt++;

    // Call the sound reactive LED function
    Sound_Reactive_LED(var);

    char t[20], mnt[10], mxt[10], avgt[10];
    LCD_Parallel_SetCursor(0, 0);
    LCD_Parallel_Print(int_to_string(var, t));

    LCD_Parallel_SetCursor(1, 0);
    LCD_Parallel_Print(int_to_string(mn, mnt));

    LCD_Parallel_SetCursor(1, 5);
    LCD_Parallel_Print(int_to_string(mx, mxt));

    LCD_Parallel_SetCursor(0, 5);
    LCD_Parallel_Print(int_to_string(avg/cnt, avgt));

    LCD_Parallel_Clear();

    HAL_ADC_Stop(&hadc1);
    /* USER CODE BEGIN 3 */
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
