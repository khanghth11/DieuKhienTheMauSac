#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void LCD_Log(const char* msg);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA

#define RELAY1_PORT GPIOC
#define RELAY1_PIN  GPIO_PIN_0
#define RELAY2_PORT GPIOC
#define RELAY2_PIN  GPIO_PIN_1

#define ST7789_RST_PORT GPIOC
#define ST7789_RST_PIN  GPIO_PIN_8
#define ST7789_DC_PORT  GPIOC
#define ST7789_DC_PIN   GPIO_PIN_6
// #define ST7789_CS_PORT  GPIOC
// #define ST7789_CS_PIN   GPIO_PIN_5

#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
// --- Button Pin Definitions (Update if different) ---
#define BUTTON1_MODE_PORT   B1_GPIO_Port // PC13
#define BUTTON1_MODE_PIN    B1_Pin       // PC13
#define BUTTON2_MANR1_PORT  GPIOA        // Example: PA0
#define BUTTON2_MANR1_PIN   GPIO_PIN_0   // Example: PA0
#define BUTTON3_MANR2_PORT  GPIOA        // Example: PA1
#define BUTTON3_MANR2_PIN   GPIO_PIN_1   // Example: PA1
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
