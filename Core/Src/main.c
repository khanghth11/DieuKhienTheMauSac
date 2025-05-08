/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for SmartHome Control via Color Cards
  ******************************************************************************
  * @attention
  *
  * This code implements a system to control two relays using colored cards
  * detected by TCS34725 (color) and VL53L0X (distance). It features
  * automatic (card-based) and manual (button-based) modes, with status
  * displayed on an ST7789 LCD.
  *
  * Requirements Summary:
  * - VL53L0X detects card presence (< 100mm).
  * - TCS34725 reads RGB when card present (Auto mode).
  * - ST7789 LCD displays status, mode, sensor data, relay states.
  * - Auto Mode: Green toggles R1, Red toggles R2, Yellow toggles R1 & R2.
  * - Manual Mode: Button 2 (PA0 long press) toggles R1, Button 3 (PA1 long press) toggles R2.
  * - Button 1 (PC13/B1 short press) switches between Auto and Manual modes.
  * - Uses provided libraries for sensors and display.
  * - Runs on STM32F401RE using STM32CubeIDE (HAL).
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "st7789.h"         // LCD Driver
#include "VL53L0X.h"        // Distance Sensor Driver
#include "TCS34725.h"       // Color Sensor Driver
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>           // Optional, maybe for advanced color processing later
#include "fonts.h"          // LCD Fonts

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_AUTOMATIC,
    MODE_MANUAL
} OperatingMode_t;

typedef enum {
    STATE_IDLE,             // Waiting for card (Auto) or button (Manual)
    STATE_SCANNING,         // Card detected, reading color (Auto only)
    STATE_PROCESSING,       // Color data available, determining action (Auto only)
    STATE_ACTION_DONE,      // Action performed, waiting for card removal or next scan (Auto only)
    STATE_ERROR,            // Error state (e.g., color unknown, sensor fail)
    STATE_MANUAL_INPUT_WAIT,// Waiting for button press in manual mode
    STATE_MANUAL_BTN2_HELD, // Button 2 is being held (Manual)
    STATE_MANUAL_BTN3_HELD  // Button 3 is being held (Manual)
} SystemState_t;

typedef enum {
    COLOR_UNKNOWN,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_YELLOW,
    // Add more colors if needed (e.g., BLUE, BROWN, PURPLE based on old reqs?)
    // For now, only implementing Green, Red, Yellow for actions
} CardColor_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    uint32_t      pressStartTime;
    bool          isPressed;
    bool          waitForRelease; // Flag to prevent multiple triggers on hold
    bool          shortPressDetected;
    bool          longPressDetected;
} ButtonInfo_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CARD_PRESENT_THRESHOLD_MM 100   // Distance threshold for card detection
#define CARD_ABSENT_THRESHOLD_MM  100   // Distance threshold for card removal (can be same or slightly higher)

// --- Color Recognition Thresholds (REQUIRES TUNING) ---
// These are starting points. Adjust based on your specific cards and lighting.
// Removed 'Clear' value dependency. Focus on dominant color and relative values.
#define COLOR_DOMINANCE_FACTOR 1.3f // How much stronger dominant color should be
#define GREEN_G_MIN 20          // Min Green value
#define GREEN_R_MAX 70          // Max Red value for Green card
#define GREEN_B_MAX 70          // Max Blue value for Green card
#define RED_R_MIN   20
#define RED_G_MAX   60
#define RED_B_MAX   60
#define YELLOW_R_MIN 80
#define YELLOW_G_MIN 80
#define YELLOW_B_MAX 70

// --- Timing Intervals ---
#define SENSOR_READ_INTERVAL_MS     50  // How often to read sensors
#define BUTTON_CHECK_INTERVAL_MS    20  // How often to check buttons
#define STATE_MACHINE_INTERVAL_MS   50  // How often to run state logic
#define LCD_UPDATE_INTERVAL_MS      200 // How often to update LCD
#define BUTTON_DEBOUNCE_MS          50  // Debounce time for buttons
#define BUTTON_LONG_PRESS_MS        3000 // Hold time for manual relay toggle

// --- LCD Layout ---
#define LCD_LINE_HEIGHT     Font_11x18.height
#define LCD_MODE_LINE_Y     5
#define LCD_STATE_LINE_Y    (LCD_MODE_LINE_Y + LCD_LINE_HEIGHT + 2)
#define LCD_RELAY_LINE_Y    (LCD_STATE_LINE_Y + LCD_LINE_HEIGHT + 2)
#define LCD_DIST_LINE_Y     (LCD_RELAY_LINE_Y + LCD_LINE_HEIGHT + 5)
#define LCD_RGB_LINE_Y      (LCD_DIST_LINE_Y + LCD_LINE_HEIGHT + 2)
#define LCD_LOG_LINE_Y      (LCD_RGB_LINE_Y + LCD_LINE_HEIGHT + 5) // Added Log Line
#define LCD_LOG_BUFFER_SIZE 40



// --- TCS34725 ADC Enable/Disable Delays ---
#define TCS_ENABLE_DELAY_MS 5           // Small delay after enabling PON/AEN
#define TCS_ADC_READ_DELAY  30          // Delay after AEN before reading ADC registers (adjust based on ATIME)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1; // For TCS34725
I2C_HandleTypeDef hi2c3; // For VL53L0X
SPI_HandleTypeDef hspi2; // For ST7789 LCD
UART_HandleTypeDef huart2; // For Debug (Optional)

/* USER CODE BEGIN PV */
// --- System State & Mode ---
volatile SystemState_t g_currentState = STATE_IDLE;
volatile OperatingMode_t g_currentMode = MODE_AUTOMATIC;

// --- Relay States ---
bool g_relay1State = false;
bool g_relay2State = false;

// --- Sensor Data & Status ---
bool g_cardPresent = false;
bool g_tcsAvailable = false;      // Flag if TCS34725 chip ID was read correctly
bool g_tcsEnabled = false;        // Flag if TCS ADC (AEN) is currently enabled
uint16_t g_latestDistance = 0xFFFF; // 0xFFFF indicates invalid/error
uint16_t g_latestR = 0, g_latestG = 0, g_latestB = 0;
bool g_vl53l0x_timeout_occurred = false;
bool g_vl53l0x_available = false; // Flag if VL53L0X initialized correctly
CardColor_t g_lastDetectedColor = COLOR_UNKNOWN;

// --- LCD Update Control ---
char g_lcdLogBuffer[LCD_LOG_BUFFER_SIZE] = "System Booting...";
bool g_lcdLogUpdated = true; // Flag to force log redraw initially
bool g_forceLcdUpdate = true; // Flag to force full redraw

// --- Timing ---
uint32_t tgDocCamBienCuoi = 0;
uint32_t tgXuLyNutNhanCuoi = 0;
uint32_t tgCapNhatLogicCuoi = 0;
uint32_t tgCapNhatLCDCuoi = 0;

// --- Button Handling ---
ButtonInfo_t g_buttonMode = {BUTTON1_MODE_PORT, BUTTON1_MODE_PIN, 0, false, false, false, false};
ButtonInfo_t g_buttonManR1 = {BUTTON2_MANR1_PORT, BUTTON2_MANR1_PIN, 0, false, false, false, false};
ButtonInfo_t g_buttonManR2 = {BUTTON3_MANR2_PORT, BUTTON3_MANR2_PIN, 0, false, false, false, false};

// --- Card Presence Debounce ---
uint8_t demCoThe = 0;
uint8_t demKoThe = 0;
const uint8_t DEM_XAC_NHAN_CO = 3; // Need 3 consecutive reads below threshold
const uint8_t DEM_XAC_NHAN_KO = 3; // Need 3 consecutive reads above threshold

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
// --- Core Logic ---
void khoiTaoHeThong(void);
void xuLyCamBien(void);
void xuLyNutNhan(ButtonInfo_t *button); // Process individual button
void capNhatTrangThaiVaCheDo(void);
void capNhatManHinhLCD(void);

// --- Helpers ---
void setRelayState(uint8_t relayNum, bool state);
CardColor_t processColorReading(uint16_t r, uint16_t g, uint16_t b);
const char* colorToString(CardColor_t color);
const char* modeToString(OperatingMode_t mode);
const char* stateToString(SystemState_t state); // For Debug/Logging
void logToLCD(const char* message);
void toggleRelay(uint8_t relayNum);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- Logging Function ---
void logToLCD(const char* message) {
    strncpy(g_lcdLogBuffer, message, LCD_LOG_BUFFER_SIZE - 1);
    g_lcdLogBuffer[LCD_LOG_BUFFER_SIZE - 1] = '\0'; // Ensure null termination
    g_lcdLogUpdated = true; // Signal LCD update needed
    // Optional: Print to UART for debugging
    // printf("LOG: %s\r\n", message);
}

// --- Relay Control ---
void setRelayState(uint8_t relayNum, bool state) {
    GPIO_PinState pinState = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    char logMsg[30];
    bool changed = false;

    if (relayNum == 1) {
        if (g_relay1State != state) {
            HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, pinState);
            g_relay1State = state;
            changed = true;
        }
    } else if (relayNum == 2) {
        if (g_relay2State != state) {
            HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, pinState);
            g_relay2State = state;
            changed = true;
        }
    }

    if (changed) {
        snprintf(logMsg, sizeof(logMsg), "Relay %d -> %s", relayNum, state ? "ON" : "OFF");
        logToLCD(logMsg);
        g_forceLcdUpdate = true; // Force redraw if relay changes
    }
}

void toggleRelay(uint8_t relayNum) {
    if (relayNum == 1) {
        setRelayState(1, !g_relay1State);
    } else if (relayNum == 2) {
        setRelayState(2, !g_relay2State);
    }
}


// --- Color Processing ---
CardColor_t processColorReading(uint16_t r, uint16_t g, uint16_t b) {
    // Basic check: if all values are very low, it's likely dark/no card or error
    if (r < 18 && g < 18 && b < 18) return COLOR_UNKNOWN;

    // Check for Green
    if (g > GREEN_G_MIN && g > r * COLOR_DOMINANCE_FACTOR && g > b * COLOR_DOMINANCE_FACTOR && r < GREEN_R_MAX && b < GREEN_B_MAX) {
        return COLOR_GREEN;
    }
    // Check for Red
    if (r > RED_R_MIN && r > g * COLOR_DOMINANCE_FACTOR && r > b * COLOR_DOMINANCE_FACTOR && g < RED_G_MAX && b < RED_B_MAX) {
        return COLOR_RED;
    }
    // Check for Yellow (High R and G, low B)
    if (r > YELLOW_R_MIN && g > YELLOW_G_MIN && b < YELLOW_B_MAX && (r + g) > (b * 2 * COLOR_DOMINANCE_FACTOR)) {
         return COLOR_YELLOW;
    }

    // Add checks for other colors here if needed (Blue, Bro                                                    wn, Purple...)

    return COLOR_UNKNOWN; // No match
}

// --- Enum to String Converters ---
const char* colorToString(CardColor_t color) {
    switch (color) {
        case COLOR_RED: return "DO";
        case COLOR_GREEN: return "XANH LA";
        case COLOR_YELLOW: return "VANG";
        default: return "KHONG XD";
    }
}

const char* modeToString(OperatingMode_t mode) {
    return (mode == MODE_AUTOMATIC) ? "TU DONG" : "THU CONG";
}

// Optional: For debugging state machine
const char* stateToString(SystemState_t state) {
     switch(state) {
         case STATE_IDLE: return "IDLE";
         case STATE_SCANNING: return "SCANNING";
         case STATE_PROCESSING: return "PROCESSING";
         case STATE_ACTION_DONE: return "ACTION_DONE";
         case STATE_ERROR: return "ERROR";
         case STATE_MANUAL_INPUT_WAIT: return "MANUAL_WAIT";
         case STATE_MANUAL_BTN2_HELD: return "MANUAL_B2_HELD";
         case STATE_MANUAL_BTN3_HELD: return "MANUAL_B3_HELD";
         default: return "UNKNOWN_STATE";
     }
}

// --- System Initialization ---
void khoiTaoHeThong() {
    logToLCD("System Init...");
    g_forceLcdUpdate = true; // Force initial draw

    // 1. Initialize Relays (Off)
    setRelayState(1, false);
    setRelayState(2, false);
    logToLCD("Relays Init OK"); HAL_Delay(50);

    // 2. Initialize LCD
    ST7789_Init();
    ST7789_Fill_Color(BLACK);
    logToLCD("LCD Init OK"); HAL_Delay(50);

    // 3. Initialize VL53L0X Distance Sensor (using I2C3)
    logToLCD("VL53L0X Init..."); HAL_Delay(10);
    if (initVL53L0X(true, &hi2c3)) { // Assuming 2.8V IO, pass I2C3 handle
        setTimeout(500); // Set I/O timeout
        startContinuous(0); // Start continuous back-to-back mode
        g_vl53l0x_available = true;
        logToLCD("VL53L0X Started");
    } else {
        g_vl53l0x_available = false;
        logToLCD("VL53L0X FAIL!");
        // Consider entering an error state or halting
        // Error_Handler(); // Or handle more gracefully
    }
    HAL_Delay(100);

    // 4. Initialize TCS34725 Color Sensor (using I2C1)
    logToLCD("TCS34725 Init..."); HAL_Delay(10);
    uint8_t chipId = I2C_Read8BIT(TCS34725_ID); // Read Chip ID register (0x12)
    // Valid IDs are typically 0x44 for TCS34725 or 0x4D for TCS34727
    if (chipId == 0x44 || chipId == 0x4D) {
        char msg[25];
        snprintf(msg, sizeof(msg), "TCS OK (ID:0x%02X)", chipId);
        logToLCD(msg);

        // Basic Configuration:
        I2C_Write8BIT(ENABLE_REG, 0x01); // PON = 1 (Power ON), AEN = 0 (ADC Disabled initially)
        g_tcsEnabled = false;
        HAL_Delay(TCS_ENABLE_DELAY_MS);
        I2C_Write8BIT(ATIME_REG, 0xF6); // Integration Time: 24ms * (256 - 246) = 240ms -> ~4 cycles/sec. Adjust if needed. (0xF6 = 24ms)
        I2C_Write8BIT(CONTROL_REG, 0x00); // Gain: 1x (0x00), 4x (0x01), 16x (0x02), 60x (0x03)
        g_tcsAvailable = true;
        logToLCD("TCS Configured");
    } else {
        char msg[25];
        snprintf(msg, sizeof(msg), "TCS FAIL! ID:0x%02X", chipId);
        logToLCD(msg);
        g_tcsAvailable = false;
        // Error_Handler(); // Or handle more gracefully
    }
    HAL_Delay(100);

    // 5. Set Initial State & Mode
    g_currentMode = MODE_AUTOMATIC;
    g_currentState = STATE_IDLE;
    logToLCD("System Ready");
    g_forceLcdUpdate = true; // Force redraw after init
}

// --- Sensor Reading Logic ---
void xuLyCamBien(void) {
    uint16_t distance;
    statInfo_t_VL53L0X distanceStats; // Optional stats struct
    bool theTruoc = g_cardPresent;

    // 1. Read Distance Sensor
    if (g_vl53l0x_available) {
        distance = readRangeContinuousMillimeters(&distanceStats); // Pass NULL if stats not needed
        g_vl53l0x_timeout_occurred = timeoutOccurred();

        // Handle distance reading errors/timeouts
        if (g_vl53l0x_timeout_occurred || distance == 8191 || distance == 65535) { // 8191: Sigma Fail, 65535: Timeout
            g_latestDistance = 0xFFFF; // Indicate error
            distance = CARD_ABSENT_THRESHOLD_MM + 1; // Treat as card absent
        } else {
            g_latestDistance = distance;
        }

        // Debounce card presence/absence
        bool coThe_TucThoi = (distance < CARD_PRESENT_THRESHOLD_MM);
        if (coThe_TucThoi) {
            demKoThe = 0;
            if (demCoThe < DEM_XAC_NHAN_CO) demCoThe++;
        } else {
            demCoThe = 0;
            if (demKoThe < DEM_XAC_NHAN_KO) demKoThe++;
        }

        if (demCoThe >= DEM_XAC_NHAN_CO) g_cardPresent = true;
        else if (demKoThe >= DEM_XAC_NHAN_KO) g_cardPresent = false;

    } else {
        // VL53L0X not available, assume no card
        g_cardPresent = false;
        g_latestDistance = 0xFFFF;
    }

    // 2. Control and Read Color Sensor based on presence and mode
    if (g_currentMode == MODE_AUTOMATIC) {
        if (g_cardPresent && !theTruoc) {
            // Card just inserted
            if (g_tcsAvailable && !g_tcsEnabled) {
                I2C_Write8BIT(ENABLE_REG, 0x03); // PON=1, AEN=1 (Enable ADC)
                g_tcsEnabled = true;
                HAL_Delay(TCS_ADC_READ_DELAY); // Wait for ADC integration
                logToLCD("TCS Enabled");
                g_currentState = STATE_SCANNING; // Trigger state change
                 g_forceLcdUpdate = true;
            }
        } else if (!g_cardPresent && theTruoc) {
            // Card just removed
            if (g_tcsAvailable && g_tcsEnabled) {
                I2C_Write8BIT(ENABLE_REG, 0x01); // PON=1, AEN=0 (Disable ADC)
                g_tcsEnabled = false;
                logToLCD("TCS Disabled");
            }
            g_latestR = g_latestG = g_latestB = 0;
            g_lastDetectedColor = COLOR_UNKNOWN;
             g_currentState = STATE_IDLE; // Go back to idle when card removed
             g_forceLcdUpdate = true;
        }

        // Read RGB only if card is present, TCS is available/enabled, and state requires it
        if (g_cardPresent && g_tcsAvailable && g_tcsEnabled && (g_currentState == STATE_SCANNING || g_currentState == STATE_PROCESSING)) {
             // Read RGB values
             g_latestR = I2C_Read16BIT(RED);
             g_latestG = I2C_Read16BIT(GREEN);
             g_latestB = I2C_Read16BIT(BLUE);
             // If we are in scanning state, transition to processing
             if(g_currentState == STATE_SCANNING) {
                 g_currentState = STATE_PROCESSING;
                  g_forceLcdUpdate = true;
             }
        }
    } else { // Manual Mode
        // Keep TCS ADC disabled in manual mode to prevent unnecessary reads
        if (g_tcsAvailable && g_tcsEnabled) {
            I2C_Write8BIT(ENABLE_REG, 0x01); // PON=1, AEN=0
            g_tcsEnabled = false;
            logToLCD("TCS Disabled (Manual)");
        }
         g_latestR = g_latestG = g_latestB = 0; // Clear RGB values
    }
}

// --- Button Handling Logic ---
void xuLyNutNhan(ButtonInfo_t *button) {
    uint32_t now = HAL_GetTick();
    bool pinState = HAL_GPIO_ReadPin(button->port, button->pin) == GPIO_PIN_RESET; // Assuming active low buttons (pressed = LOW)

    // Reset flags at the start of each check cycle
    button->shortPressDetected = false;
    button->longPressDetected = false;

    if (pinState && !button->isPressed) { // Button newly pressed
        // Start debouncing/press timing only if not waiting for release
        if (!button->waitForRelease) {
            button->isPressed = true;
            button->pressStartTime = now;
        }
    } else if (!pinState && button->isPressed) { // Button newly released
        if (now - button->pressStartTime >= BUTTON_DEBOUNCE_MS) {
            // Check for short press upon release (after debounce)
            if (!button->waitForRelease) { // Make sure long press didn't already trigger
                 if (now - button->pressStartTime < BUTTON_LONG_PRESS_MS) {
                    button->shortPressDetected = true;
                 }
            }
        }
        button->isPressed = false;
        button->waitForRelease = false; // Reset wait flag on release
    } else if (pinState && button->isPressed) { // Button held down
        // Check for long press only if not already detected and waiting for release
        if (!button->waitForRelease && (now - button->pressStartTime >= BUTTON_LONG_PRESS_MS)) {
            button->longPressDetected = true;
            button->waitForRelease = true; // Prevent further triggers until released
        }
    }
}

// --- Main State Machine & Mode Logic ---
void capNhatTrangThaiVaCheDo(void) {
     SystemState_t ttTruoc = g_currentState;
     OperatingMode_t modeTruoc = g_currentMode;

     // 1. Process Button Inputs
     xuLyNutNhan(&g_buttonMode);
     xuLyNutNhan(&g_buttonManR1);
     xuLyNutNhan(&g_buttonManR2);

     // 2. Handle Mode Switching (Button 1 Short Press)
     if (g_buttonMode.shortPressDetected) {
         if (g_currentMode == MODE_AUTOMATIC) {
             g_currentMode = MODE_MANUAL;
             g_currentState = STATE_MANUAL_INPUT_WAIT; // Go to manual waiting state
             logToLCD("Mode -> Manual");
         } else {
             g_currentMode = MODE_AUTOMATIC;
             g_currentState = STATE_IDLE; // Go back to auto idle state
             logToLCD("Mode -> Automatic");
         }
          g_forceLcdUpdate = true;
     }

     // 3. State Machine Logic
     switch (g_currentMode) {
         case MODE_AUTOMATIC:
             switch (g_currentState) {
                 case STATE_IDLE:
                     // Transition handled by xuLyCamBien when card detected -> STATE_SCANNING
                     // Or by mode switch -> STATE_MANUAL_INPUT_WAIT
                     if (!g_cardPresent) { /* Remain IDLE */ }
                     break;

                 case STATE_SCANNING:
                     // Transition handled by xuLyCamBien when data ready -> STATE_PROCESSING
                     // Or when card removed -> STATE_IDLE
                     if (!g_cardPresent) g_currentState = STATE_IDLE;
                     break;

                 case STATE_PROCESSING:
                     g_lastDetectedColor = processColorReading(g_latestR, g_latestG, g_latestB);
                     if (g_lastDetectedColor != COLOR_UNKNOWN) {
                         // Perform Action based on color
                         switch (g_lastDetectedColor) {
                             case COLOR_GREEN: toggleRelay(1); break;
                             case COLOR_RED:   toggleRelay(2); break;
                             case COLOR_YELLOW: toggleRelay(1); toggleRelay(2); break;
                             default: logToLCD("Color OK, no action"); break; // Other valid colors but no action defined
                         }
                         g_currentState = STATE_ACTION_DONE;
                     } else {
                         logToLCD("Error: Mau Khong XD");
                         g_currentState = STATE_ERROR; // Or back to SCANNING? Maybe ERROR is better.
                     }
                     g_forceLcdUpdate = true;
                     break;

                 case STATE_ACTION_DONE:
                 case STATE_ERROR:
                     // Wait here until card is removed
                     if (!g_cardPresent) {
                         g_currentState = STATE_IDLE;
                          g_forceLcdUpdate = true;
                     } else {
                         // Optional: Could go back to SCANNING after a delay if card stays
                         // For now, just waits for removal.
                     }
                     break;

                 default: // Handle unexpected states in Auto mode
                    if(g_currentState != STATE_IDLE && g_currentState != STATE_SCANNING && g_currentState != STATE_PROCESSING && g_currentState != STATE_ACTION_DONE && g_currentState != STATE_ERROR)
                    {
                         logToLCD("Invalid Auto State!");
                         g_currentState = STATE_IDLE;
                         g_forceLcdUpdate = true;
                    }
                    break;
             }
             break; // End MODE_AUTOMATIC

         case MODE_MANUAL:
             switch (g_currentState) {
                 case STATE_MANUAL_INPUT_WAIT:
                     if (g_buttonManR1.isPressed && !g_buttonManR1.waitForRelease) { // Check if button 2 starts being held
                         g_currentState = STATE_MANUAL_BTN2_HELD;
                         logToLCD("Nut 2 Giu...");
                         g_forceLcdUpdate = true;
                     } else if (g_buttonManR2.isPressed && !g_buttonManR2.waitForRelease) { // Check if button 3 starts being held
                          g_currentState = STATE_MANUAL_BTN3_HELD;
                          logToLCD("Nut 3 Giu...");
                          g_forceLcdUpdate = true;
                     }
                     // Mode switch handled earlier
                     break;

                 case STATE_MANUAL_BTN2_HELD:
                     if (g_buttonManR1.longPressDetected) {
                         toggleRelay(1); // Action on long press
                         g_currentState = STATE_MANUAL_INPUT_WAIT; // Go back to waiting after action (button needs release)
                     } else if (!g_buttonManR1.isPressed) { // Released before long press
                          g_currentState = STATE_MANUAL_INPUT_WAIT;
                          logToLCD("Nut 2 Thả");
                          g_forceLcdUpdate = true;
                     }
                     // Mode switch handled earlier
                     break;

                 case STATE_MANUAL_BTN3_HELD:
                     if (g_buttonManR2.longPressDetected) {
                         toggleRelay(2); // Action on long press
                         g_currentState = STATE_MANUAL_INPUT_WAIT; // Go back to waiting after action
                     } else if (!g_buttonManR2.isPressed) { // Released before long press
                          g_currentState = STATE_MANUAL_INPUT_WAIT;
                          logToLCD("Nut 3 Thả");
                          g_forceLcdUpdate = true;
                     }
                      // Mode switch handled earlier
                     break;

                 default: // Handle unexpected states in Manual mode
                    if(g_currentState != STATE_MANUAL_INPUT_WAIT && g_currentState != STATE_MANUAL_BTN2_HELD && g_currentState != STATE_MANUAL_BTN3_HELD)
                    {
                         logToLCD("Invalid Manual State!");
                         g_currentState = STATE_MANUAL_INPUT_WAIT;
                         g_forceLcdUpdate = true;
                    }
                     break;
             }
             break; // End MODE_MANUAL
     }

     // Log state change for debugging if needed
     if (g_currentState != ttTruoc || g_currentMode != modeTruoc) {
          // char msg[40];
          // snprintf(msg, sizeof(msg),"Mode:%s St:%s", modeToString(g_currentMode), stateToString(g_currentState));
          // printf("%s\r\n", msg); // UART log
          // logToLCD(stateToString(g_currentState)); // Log state name to LCD
          g_forceLcdUpdate = true;
     }
}

// --- LCD Update Logic ---
void capNhatManHinhLCD(void) {
    // Static variables to track last drawn values and reduce flickering
    static OperatingMode_t lastDrawnMode = -1;
    static SystemState_t lastDrawnState = -1;
    static bool lastDrawnRly1 = false;
    static bool lastDrawnRly2 = false;
    static uint16_t lastDrawnDist = 0xFFFF;
    static bool lastDrawnCardPresent = false;
    static uint16_t lastDrawnR = 0, lastDrawnG = 0, lastDrawnB = 0;
    static char lastDrawnLog[LCD_LOG_BUFFER_SIZE] = "";

    bool modeChanged = (g_currentMode != lastDrawnMode) || g_forceLcdUpdate;
    bool stateChanged = (g_currentState != lastDrawnState) || g_forceLcdUpdate;
    bool relaysChanged = (g_relay1State != lastDrawnRly1) || (g_relay2State != lastDrawnRly2) || g_forceLcdUpdate;
    bool distChanged = (g_latestDistance != lastDrawnDist) || (g_cardPresent != lastDrawnCardPresent) || g_forceLcdUpdate;
    bool rgbChanged = (g_latestR != lastDrawnR) || (g_latestG != lastDrawnG) || (g_latestB != lastDrawnB) || g_forceLcdUpdate;
    bool logChanged = g_lcdLogUpdated || g_forceLcdUpdate;

    // If nothing changed, skip redraw
    if (!modeChanged && !stateChanged && !relaysChanged && !distChanged && !rgbChanged && !logChanged) {
        return;
    }

    char lineBuffer[45]; // Buffer for formatting lines

    // 1. Draw Mode Line
    if (modeChanged || stateChanged) { // Redraw if mode or state changes context
        snprintf(lineBuffer, sizeof(lineBuffer), "CHE DO: %s", modeToString(g_currentMode));
        ST7789_Fill(0, LCD_MODE_LINE_Y, ST7789_WIDTH - 1, LCD_MODE_LINE_Y + LCD_LINE_HEIGHT, BLACK); // Clear line
        ST7789_WriteString(5, LCD_MODE_LINE_Y, lineBuffer, Font_11x18, WHITE, BLACK);
        lastDrawnMode = g_currentMode;
    }

    // 2. Draw State/Instruction Line
    if (stateChanged || modeChanged) { // Redraw if state or mode changes context
        ST7789_Fill(0, LCD_STATE_LINE_Y, ST7789_WIDTH - 1, LCD_STATE_LINE_Y + LCD_LINE_HEIGHT, BLACK); // Clear line
        uint16_t stateColor = WHITE;
        switch (g_currentState) {
            case STATE_IDLE:
                snprintf(lineBuffer, sizeof(lineBuffer), "%s", (g_currentMode == MODE_AUTOMATIC) ? "XIN MOI QUET THE" : "CHO NUT NHAN...");
                stateColor = GREENTFT;
                break;
            case STATE_SCANNING:
                snprintf(lineBuffer, sizeof(lineBuffer), "DANG DOC MAU THE...");
                stateColor = YELLOW;
                break;
            case STATE_PROCESSING:
                snprintf(lineBuffer, sizeof(lineBuffer), "DANG XU LY MAU...");
                stateColor = YELLOW;
                break;
            case STATE_ACTION_DONE:
                snprintf(lineBuffer, sizeof(lineBuffer), "MAU: %s -> XONG", colorToString(g_lastDetectedColor));
                stateColor = CYAN;
                break;
            case STATE_ERROR:
                snprintf(lineBuffer, sizeof(lineBuffer), "LOI: MAU KHONG XD");
                stateColor = REDTFT;
                break;
             case STATE_MANUAL_INPUT_WAIT:
                 snprintf(lineBuffer, sizeof(lineBuffer),"NHAN GIU NUT R1/R2");
                 stateColor = WHITE;
                 break;
             case STATE_MANUAL_BTN2_HELD:
                 snprintf(lineBuffer, sizeof(lineBuffer),"GIU NUT R1 (3s)...");
                 stateColor = YELLOW;
                 break;
             case STATE_MANUAL_BTN3_HELD:
                  snprintf(lineBuffer, sizeof(lineBuffer),"GIU NUT R2 (3s)...");
                 stateColor = YELLOW;
                 break;
            default:
                snprintf(lineBuffer, sizeof(lineBuffer), "TRANG THAI LOI");
                 stateColor = REDTFT;
                break;
        }
        ST7789_WriteString(5, LCD_STATE_LINE_Y, lineBuffer, Font_11x18, stateColor, BLACK);
        lastDrawnState = g_currentState;
    }

    // 3. Draw Relay Status Line
    if (relaysChanged) {
        snprintf(lineBuffer, sizeof(lineBuffer), "TB1:%s  TB2:%s", g_relay1State ? "ON " : "OFF", g_relay2State ? "ON " : "OFF");
        ST7789_Fill(0, LCD_RELAY_LINE_Y, ST7789_WIDTH - 1, LCD_RELAY_LINE_Y + LCD_LINE_HEIGHT, BLACK); // Clear line
        ST7789_WriteString(5, LCD_RELAY_LINE_Y, lineBuffer, Font_11x18, YELLOW, BLACK);
        lastDrawnRly1 = g_relay1State;
        lastDrawnRly2 = g_relay2State;
    }

    // 4. Draw Distance Line
    if (distChanged) {
        if (g_latestDistance == 0xFFFF) {
            snprintf(lineBuffer, sizeof(lineBuffer), "Dist: N/A %s", g_cardPresent ? "[IN]":"[OUT]");
        } else {
            snprintf(lineBuffer, sizeof(lineBuffer), "Dist: %4u mm %s", g_latestDistance, g_cardPresent ? "[IN]":"[OUT]");
        }
        ST7789_Fill(0, LCD_DIST_LINE_Y, ST7789_WIDTH - 1, LCD_DIST_LINE_Y + LCD_LINE_HEIGHT, BLACK); // Clear line
        ST7789_WriteString(5, LCD_DIST_LINE_Y, lineBuffer, Font_11x18, WHITE, BLACK);
        lastDrawnDist = g_latestDistance;
        lastDrawnCardPresent = g_cardPresent;
    }

    // 5. Draw RGB Line
    if (rgbChanged || (g_currentMode == MODE_MANUAL && modeChanged)) { // Redraw if RGB changes or entering manual mode
        if (g_currentMode == MODE_AUTOMATIC && g_cardPresent && g_tcsEnabled) {
            snprintf(lineBuffer, sizeof(lineBuffer), "R:%-4u G:%-4u B:%-4u", g_latestR, g_latestG, g_latestB);
        } else {
            snprintf(lineBuffer, sizeof(lineBuffer), "R: --- G: --- B: ---");
        }
        ST7789_Fill(0, LCD_RGB_LINE_Y, ST7789_WIDTH - 1, LCD_RGB_LINE_Y + LCD_LINE_HEIGHT, BLACK); // Clear line
        ST7789_WriteString(5, LCD_RGB_LINE_Y, lineBuffer, Font_11x18, WHITE, BLACK);
        lastDrawnR = g_latestR; lastDrawnG = g_latestG; lastDrawnB = g_latestB;
    }

     // 6. Draw Log Line
    if (logChanged) {
        ST7789_Fill(0, LCD_LOG_LINE_Y, ST7789_WIDTH - 1, LCD_LOG_LINE_Y + LCD_LINE_HEIGHT, BLACK); // Clear line
        ST7789_WriteString(5, LCD_LOG_LINE_Y, g_lcdLogBuffer, Font_11x18, GRAY, BLACK);
        strncpy(lastDrawnLog, g_lcdLogBuffer, LCD_LOG_BUFFER_SIZE);
        g_lcdLogUpdated = false; // Mark log as drawn
    }


    g_forceLcdUpdate = false; // Reset force flag after a full draw cycle checks everything
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
  MX_USART2_UART_Init(); // Optional for debug
  MX_I2C1_Init();      // For TCS34725
  MX_I2C3_Init();      // For VL53L0X
  MX_SPI2_Init();      // For ST7789
  /* USER CODE BEGIN 2 */
  printf("--- System Start ---\r\n"); // Optional UART Print

  khoiTaoHeThong();

  uint32_t now = HAL_GetTick();
  tgDocCamBienCuoi = now;
  tgXuLyNutNhanCuoi = now;
  tgCapNhatLogicCuoi = now;
  tgCapNhatLCDCuoi = now;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now = HAL_GetTick();

    // --- Task Scheduling (Non-Blocking) ---

    // 1. Read Sensors Periodically
    if (now - tgDocCamBienCuoi >= SENSOR_READ_INTERVAL_MS) {
        xuLyCamBien();
        tgDocCamBienCuoi = now;
    }

    // 2. Check Buttons Periodically
    if (now - tgXuLyNutNhanCuoi >= BUTTON_CHECK_INTERVAL_MS) {
        // Processing is done inside capNhatTrangThaiVaCheDo
        tgXuLyNutNhanCuoi = now; // Just update time, actual check is below
    }

    // 3. Update State Machine Periodically
    if (now - tgCapNhatLogicCuoi >= STATE_MACHINE_INTERVAL_MS) {
        capNhatTrangThaiVaCheDo(); // This function now also processes button inputs
        tgCapNhatLogicCuoi = now;
    }

    // 4. Update LCD Periodically
    if (now - tgCapNhatLCDCuoi >= LCD_UPDATE_INTERVAL_MS) {
        capNhatManHinhLCD();
        tgCapNhatLCDCuoi = now;
    }

     // Small delay or yield (optional, depends on loop execution time)
     // HAL_Delay(1); // Use with caution, can introduce blocking
      __NOP(); // Prevent empty loop optimization issues

    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000; // Standard mode for TCS34725
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000; // VL53L0X also typically uses 100kHz or 400kHz
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW; // Check ST7789 datasheet/library needs
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;    // Check ST7789 datasheet/library needs
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // Adjust for speed (e.g., _2, _4, _8)
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
    // Enable printf via UART
    #ifdef __GNUC__
        setvbuf(stdout, NULL, _IONBF, 0);
    #endif
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE(); // Often needed even if no pins used directly
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE(); // For SPI2 pins

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELAY1_PIN|RELAY2_PIN|ST7789_DC_PIN|ST7789_RST_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // LD2 is PA5 on Nucleo-F401RE

  /*Configure GPIO pin : B1_Pin (PC13) - Button 1 (Mode) */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Use GPIO_MODE_INPUT, read manually for debounce/hold
  GPIO_InitStruct.Pull = GPIO_PULLUP;     // Assuming button connects pin to GND when pressed
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY1_PIN(PC0) RELAY2_PIN(PC1) ST7789_DC_PIN(PC6) ST7789_RST_PIN(PC8) */
  GPIO_InitStruct.Pin = RELAY1_PIN|RELAY2_PIN|ST7789_DC_PIN|ST7789_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin (PA5) */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pins : PA0 PA1 - Button 2 & 3 (Manual Control) */
  GPIO_InitStruct.Pin = BUTTON2_MANR1_PIN|BUTTON3_MANR2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Assuming buttons connect pins to GND when pressed
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Add implementations for helper functions if needed outside main scope,
// but most are already defined above.

// Override _write for printf redirection to UART2
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    (void)file; // Prevent unused warning
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
#endif
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
  logToLCD("!!! SYSTEM HALTED !!!"); // Log error to LCD if possible
  printf("!!! SYSTEM HALTED - Error_Handler() called !!!\r\n");
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Turn on LD2
  while (1)
  {
      // Blink LD2 rapidly to indicate hard fault
       HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
       HAL_Delay(100);
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
     printf("Assert failed: file %s on line %lu\r\n", (char*)file, line);
     Error_Handler(); // Go to error handler on assert fail
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
