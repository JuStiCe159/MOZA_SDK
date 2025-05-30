/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for VolantGT MOZA R9 compatible firmware
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
#include <string.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_mpu.h"
#include <stdint.h>  /* Pour les types uint8_t, int8_t, etc. */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t buttonState[2];  // 16 buttons total (8 per byte)
  int8_t encoder1;         // Encoder 1 relative position
  int8_t encoder2;         // Encoder 2 relative position
} WheelState_t;

typedef enum {
  MOZA_IDLE,
  MOZA_RECEIVING,
  MOZA_TRANSMITTING,
  MOZA_ERROR
} MozaState_t;

typedef enum {
  PACKET_HEADER,
  PACKET_TYPE,
  PACKET_LENGTH,
  PACKET_DATA,
  PACKET_CHECKSUM
} PacketState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOZA_UART_BAUDRATE 115200
#define MOZA_PACKET_SIZE 64
#define MOZA_HEADER_BYTE 0xA5
#define MOZA_MAX_DATA_SIZE 56
#define MOZA_PACKET_TIMEOUT 100  // ms
#define MOZA_ERROR_TIMEOUT 1000  // ms
#define MOZA_TX_INTERVAL 20      // ms
#define BUTTON_DEBOUNCE_TIME 10  // ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;  // UART for MOZA communication

TIM_HandleTypeDef htim1;    // Timer for encoder reading
TIM_HandleTypeDef htim2;    // Timer for debouncing

/* USER CODE BEGIN PV */
// Wheel state variables
WheelState_t wheelState = {0};
WheelState_t prevWheelState = {0};
WheelState_t debouncedState = {0};
uint32_t lastButtonReadTime = 0;

// MOZA communication variables
uint8_t mozaRxBuffer[MOZA_PACKET_SIZE];
uint8_t mozaTxBuffer[MOZA_PACKET_SIZE];
uint8_t mozaRxData[MOZA_MAX_DATA_SIZE];
uint8_t mozaRxIndex = 0;
uint8_t mozaRxLength = 0;
uint8_t mozaRxType = 0;
uint8_t mozaRxChecksum = 0;
uint8_t mozaCalculatedChecksum = 0;
MozaState_t mozaState = MOZA_IDLE;
PacketState_t packetState = PACKET_HEADER;
uint32_t lastPacketTime = 0;
uint32_t lastErrorTime = 0;
uint32_t lastTransmitTime = 0;
uint32_t packetErrorCount = 0;
uint32_t successfulPackets = 0;
bool connectionActive = false;

// System status
bool stateChanged = false;
bool systemInitialized = false;
uint32_t ledBlinkTime = 0;
uint8_t ledBlinkCount = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void ReadButtons(void);
static void ReadEncoders(void);
static void ProcessMozaData(void);
static void PrepareMozaPacket(void);
static bool CompareWheelStates(const WheelState_t* state1, const WheelState_t* state2);
static void UpdateLEDs(void);
static void ResetMozaReceiver(void);
static uint8_t CalculateChecksum(const uint8_t* data, uint16_t length);
static void HandleMozaError(void);
static void DebounceButtons(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Read button states and update wheelState
  * @retval None
  */
static void ReadButtons(void)
{
  // Read raw button states
  uint8_t rawButtonState0 = 0;
  uint8_t rawButtonState1 = 0;
  
  // Read buttons 1-8 (first byte)
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) == GPIO_PIN_RESET) ? (1 << 0) : 0;
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_2_GPIO_Port, BUTTON_2_Pin) == GPIO_PIN_RESET) ? (1 << 1) : 0;
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_3_GPIO_Port, BUTTON_3_Pin) == GPIO_PIN_RESET) ? (1 << 2) : 0;
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_4_GPIO_Port, BUTTON_4_Pin) == GPIO_PIN_RESET) ? (1 << 3) : 0;
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_5_GPIO_Port, BUTTON_5_Pin) == GPIO_PIN_RESET) ? (1 << 4) : 0;
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_6_GPIO_Port, BUTTON_6_Pin) == GPIO_PIN_RESET) ? (1 << 5) : 0;
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_7_GPIO_Port, BUTTON_7_Pin) == GPIO_PIN_RESET) ? (1 << 6) : 0;
  rawButtonState0 |= (HAL_GPIO_ReadPin(BUTTON_8_GPIO_Port, BUTTON_8_Pin) == GPIO_PIN_RESET) ? (1 << 7) : 0;
  
  // Store raw button states for debouncing
  wheelState.buttonState[0] = rawButtonState0;
  wheelState.buttonState[1] = rawButtonState1;
  
  // Apply debouncing
  DebounceButtons();
}

/**
  * @brief  Debounce button inputs
  * @retval None
  */
static void DebounceButtons(void)
{
  uint32_t currentTime = HAL_GetTick();
  
  // Only debounce at specified intervals
  if (currentTime - lastButtonReadTime >= BUTTON_DEBOUNCE_TIME) {
    lastButtonReadTime = currentTime;
    
    // Simple debouncing - only register a button press if it's stable for multiple reads
    static uint8_t stableCount[2] = {0, 0};
    static uint8_t lastState[2] = {0, 0};
    
    for (int i = 0; i < 2; i++) {
      if (wheelState.buttonState[i] == lastState[i]) {
        stableCount[i]++;
        if (stableCount[i] >= 3) { // Button is stable for 3 consecutive reads
          debouncedState.buttonState[i] = wheelState.buttonState[i];
          stableCount[i] = 3; // Cap the count
        }
      } else {
        lastState[i] = wheelState.buttonState[i];
        stableCount[i] = 0;
      }
    }
  }
}

/**
  * @brief  Read encoder states and update wheelState
  * @retval None
  */
static void ReadEncoders(void)
{
  static uint8_t enc1_last_state = 0;
  static uint8_t enc2_last_state = 0;
  
  // Read current encoder states
  uint8_t enc1_a = HAL_GPIO_ReadPin(ENC1_A_GPIO_Port, ENC1_A_Pin);
  uint8_t enc1_b = HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin);
  uint8_t enc2_a = HAL_GPIO_ReadPin(ENC2_A_GPIO_Port, ENC2_A_Pin);
  uint8_t enc2_b = HAL_GPIO_ReadPin(ENC2_B_GPIO_Port, ENC2_B_Pin);
  
  // Combine pins to get current state
  uint8_t enc1_state = (enc1_a << 1) | enc1_b;
  uint8_t enc2_state = (enc2_a << 1) | enc2_b;
  
  // Encoder 1 state machine with error checking
  if (enc1_state != enc1_last_state) {
    // Check for valid transitions
    bool validTransition = false;
    int8_t direction = 0;
    
    switch (enc1_last_state) {
      case 0:
        if (enc1_state == 1) { direction = 1; validTransition = true; }
        else if (enc1_state == 2) { direction = -1; validTransition = true; }
        break;
      case 1:
        if (enc1_state == 3) { direction = 1; validTransition = true; }
        else if (enc1_state == 0) { direction = -1; validTransition = true; }
        break;
      case 2:
        if (enc1_state == 0) { direction = 1; validTransition = true; }
        else if (enc1_state == 3) { direction = -1; validTransition = true; }
        break;
      case 3:
        if (enc1_state == 2) { direction = 1; validTransition = true; }
        else if (enc1_state == 1) { direction = -1; validTransition = true; }
        break;
    }
    
    if (validTransition) {
      wheelState.encoder1 += direction;
    }
    
    enc1_last_state = enc1_state;
  }
  
  // Encoder 2 state machine with error checking
  if (enc2_state != enc2_last_state) {
    // Check for valid transitions
    bool validTransition = false;
    int8_t direction = 0;
    
    switch (enc2_last_state) {
      case 0:
        if (enc2_state == 1) { direction = 1; validTransition = true; }
        else if (enc2_state == 2) { direction = -1; validTransition = true; }
        break;
      case 1:
        if (enc2_state == 3) { direction = 1; validTransition = true; }
        else if (enc2_state == 0) { direction = -1; validTransition = true; }
        break;
      case 2:
        if (enc2_state == 0) { direction = 1; validTransition = true; }
        else if (enc2_state == 3) { direction = -1; validTransition = true; }
        break;
      case 3:
        if (enc2_state == 2) { direction = 1; validTransition = true; }
        else if (enc2_state == 1) { direction = -1; validTransition = true; }
        break;
    }
    
    if (validTransition) {
      wheelState.encoder2 += direction;
    }
    
    enc2_last_state = enc2_state;
  }
}

/**
  * @brief  Process data received from MOZA base
  * @retval None
  */
static void ProcessMozaData(void)
{
  // Reset connection timeout
  lastPacketTime = HAL_GetTick();
  connectionActive = true;
  successfulPackets++;
  
  // Process based on packet type
  switch (mozaRxType) {
    case 0x01: // Force feedback data
      if (mozaRxLength >= 1) {
        // Process force feedback data
        // mozaRxData[0] might contain force magnitude
        
        // Toggle green LED to indicate valid packet received
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
      }
      break;
      
    case 0x02: // Configuration data
      if (mozaRxLength >= 2) {
        // Process configuration data
        // Example: mozaRxData[0] might be a configuration parameter ID
        // Example: mozaRxData[1] might be the value
      }
      break;
      
    case 0x03: // Heartbeat/status request
      // Respond immediately with a status packet
      if (mozaState == MOZA_IDLE) {
        PrepareMozaPacket();
        mozaState = MOZA_TRANSMITTING;
        HAL_UART_Transmit_IT(&huart3, mozaTxBuffer, 8);
      }
      break;
      
    default:
      // Unknown packet type, ignore
      break;
  }
}

/**
  * @brief  Prepare packet to send to MOZA base
  * @retval None
  */
static void PrepareMozaPacket(void)
{
  // Clear buffer
  memset(mozaTxBuffer, 0, MOZA_PACKET_SIZE);
  
  // Header
  mozaTxBuffer[0] = MOZA_HEADER_BYTE;
  
  // Packet type (button data)
  mozaTxBuffer[1] = 0x02;
  
  // Length
  mozaTxBuffer[2] = 4; // 2 bytes for buttons, 2 bytes for encoders
  
  // Button data - use debounced state
  mozaTxBuffer[3] = debouncedState.buttonState[0];
  mozaTxBuffer[4] = debouncedState.buttonState[1];
  
  // Encoder data
  mozaTxBuffer[5] = wheelState.encoder1;
  mozaTxBuffer[6] = wheelState.encoder2;
  
  // Simple checksum (XOR of all bytes)
  uint8_t checksum = 0;
  for (int i = 0; i < 7; i++) {
    checksum ^= mozaTxBuffer[i];
  }
  mozaTxBuffer[7] = checksum;
}

/**
  * @brief  Compare two wheel states to detect changes
  * @param  state1: First wheel state
  * @param  state2: Second wheel state
  * @retval bool: true if states are different, false if identical
  */
static bool CompareWheelStates(const WheelState_t* state1, const WheelState_t* state2)
{
  if (state1->buttonState[0] != state2->buttonState[0] ||
      state1->buttonState[1] != state2->buttonState[1] ||
      state1->encoder1 != 0 ||
      state1->encoder2 != 0) {
    return true;
  }
  return false;
}

/**
  * @brief  Reset MOZA receiver state machine
  * @retval None
  */
static void ResetMozaReceiver(void)
{
  packetState = PACKET_HEADER;
  mozaRxIndex = 0;
  mozaRxLength = 0;
  mozaRxType = 0;
  mozaRxChecksum = 0;
  mozaCalculatedChecksum = 0;
}

/**
  * @brief  Calculate checksum for MOZA packet
  * @param  data: Pointer to data
  * @param  length: Length of data
  * @retval Calculated checksum
  */
static uint8_t CalculateChecksum(const uint8_t* data, uint16_t length)
{
  uint8_t checksum = 0;
  for (uint16_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

/**
  * @brief  Handle MOZA communication error
  * @retval None
  */
static void HandleMozaError(void)
{
  // Set error state
  mozaState = MOZA_ERROR;
  packetErrorCount++;
  
  // Reset receiver
  ResetMozaReceiver();
  
  // Visual indication of error
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  
  // Record error time
  lastErrorTime = HAL_GetTick();
}

/**
  * @brief  Update LED status based on system state
  * @retval None
  */
static void UpdateLEDs(void)
{
  uint32_t currentTime = HAL_GetTick();
  
  // Handle error state recovery
  if (mozaState == MOZA_ERROR && (currentTime - lastErrorTime > MOZA_ERROR_TIMEOUT)) {
    mozaState = MOZA_IDLE;
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }
  
  // Connection status indication
  if (connectionActive) {
    if (currentTime - lastPacketTime > MOZA_PACKET_TIMEOUT) {
      connectionActive = false;
      // Blink green LED to indicate connection lost
      ledBlinkTime = currentTime;
      ledBlinkCount = 0;
    }
  }
  
  // Blink pattern for connection status
  if (!connectionActive && (currentTime - ledBlinkTime > 250)) {
    ledBlinkTime = currentTime;
    ledBlinkCount++;
    
    if (ledBlinkCount % 2 == 0) {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    }
    
    if (ledBlinkCount >= 6) {
      ledBlinkCount = 0;
    }
  }
}

/**
  * @brief  UART RX callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    uint8_t receivedByte = mozaRxBuffer[0];
    lastPacketTime = HAL_GetTick();
    
    // State machine for packet reception
    switch (packetState) {
      case PACKET_HEADER:
        if (receivedByte == MOZA_HEADER_BYTE) {
          mozaState = MOZA_RECEIVING;
          mozaRxIndex = 0;
          mozaCalculatedChecksum = receivedByte;
          packetState = PACKET_TYPE;
        }
        break;
        
      case PACKET_TYPE:
        mozaRxType = receivedByte;
        mozaCalculatedChecksum ^= receivedByte;
        packetState = PACKET_LENGTH;
        break;
        
      case PACKET_LENGTH:
        mozaRxLength = receivedByte;
        mozaCalculatedChecksum ^= receivedByte;
        
        if (mozaRxLength > MOZA_MAX_DATA_SIZE) {
          // Invalid length
          HandleMozaError();
        } else if (mozaRxLength == 0) {
          // No data, go directly to checksum
          packetState = PACKET_CHECKSUM;
        } else {
          // Data follows
          mozaRxIndex = 0;
          packetState = PACKET_DATA;
        }
        break;
        
      case PACKET_DATA:
        mozaRxData[mozaRxIndex++] = receivedByte;
        mozaCalculatedChecksum ^= receivedByte;
        
        if (mozaRxIndex >= mozaRxLength) {
          packetState = PACKET_CHECKSUM;
        }
        break;
        
      case PACKET_CHECKSUM:
        mozaRxChecksum = receivedByte;
        
        if (mozaRxChecksum == mozaCalculatedChecksum) {
          // Valid packet
          ProcessMozaData();
          connectionActive = true;
        } else {
          // Checksum error
          HandleMozaError();
        }
        
        // Reset for next packet
        packetState = PACKET_HEADER;
        mozaState = MOZA_IDLE;
        break;
        
      default:
        // Invalid state, reset
        ResetMozaReceiver();
        break;
    }
    
    // Continue receiving
    HAL_UART_Receive_IT(&huart3, mozaRxBuffer, 1);
  }
}

/**
  * @brief  UART TX callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    // Transmission complete, go back to receiving mode
    mozaState = MOZA_IDLE;
    HAL_UART_Receive_IT(&huart3, mozaRxBuffer, 1);
  }
}

/**
  * @brief  Timer callback for debouncing and periodic tasks
  * @param  htim: Timer handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    // Debouncing timer
    DebounceButtons();
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  // Start timers
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  
  // Start UART reception
  HAL_UART_Receive_IT(&huart3, mozaRxBuffer, 1);
  
  // Initial LED state
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  
  // System initialized
  systemInitialized = true;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read inputs
    ReadButtons();
    ReadEncoders();
    
    // Check if state has changed
    stateChanged = CompareWheelStates(&wheelState, &prevWheelState);
    
    // Update LEDs based on system state
    UpdateLEDs();
    
    // If state changed or it's time for a periodic update
    if (stateChanged || (HAL_GetTick() - lastTransmitTime > MOZA_TX_INTERVAL)) {
      // Prepare and send data to MOZA base
      if (mozaState == MOZA_IDLE) {
        PrepareMozaPacket();
        mozaState = MOZA_TRANSMITTING;
        HAL_UART_Transmit_IT(&huart3, mozaTxBuffer, 8);
        
        // Update last transmit time
        lastTransmitTime = HAL_GetTick();
        
        // Save current state as previous state
        memcpy(&prevWheelState, &wheelState, sizeof(WheelState_t));
        
        // Reset encoder values after sending
        wheelState.encoder1 = 0;
        wheelState.encoder2 = 0;
      }
    }
    
    // Check for timeout in packet reception
    if (mozaState == MOZA_RECEIVING && 
        (HAL_GetTick() - lastPacketTime > MOZA_PACKET_TIMEOUT)) {
      HandleMozaError();
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = MOZA_UART_BAUDRATE;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;  // 1000ms period
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
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;  // 10ms period for debouncing
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin BUTTON_4_Pin
                           BUTTON_5_Pin BUTTON_6_Pin BUTTON_7_Pin BUTTON_8_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin|BUTTON_4_Pin
                          |BUTTON_5_Pin|BUTTON_6_Pin|BUTTON_7_Pin|BUTTON_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_A_Pin ENC1_B_Pin ENC2_A_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC1_A_Pin|ENC1_B_Pin|ENC2_A_Pin|ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  
  // Visual indication of critical error - both LEDs on
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  
  while (1)
  {
    // Blink pattern to indicate error
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

