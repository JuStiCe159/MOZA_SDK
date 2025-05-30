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
#include <stdint.h>  /* Pour les types uint8_t, int8_t, etc. */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal_tim.h"
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

// Définitions pour la broche de données MOZA
#define MOZA_DATA_Pin GPIO_PIN_5
#define MOZA_DATA_GPIO_Port GPIOB
#define MOZA_BIT_TIME_US 8  // Temps pour un bit en microsecondes (ajustez selon le protocole)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;  // UART pour la communication MOZA

TIM_HandleTypeDef htim1;    // Timer pour la lecture des encodeurs
TIM_HandleTypeDef htim2;    // Timer pour le debouncing

/* USER CODE BEGIN PV */
WheelState_t wheelState = {0};
WheelState_t prevWheelState = {0};

uint8_t mozaRxBuffer[MOZA_PACKET_SIZE];
uint8_t mozaTxBuffer[MOZA_PACKET_SIZE];
uint8_t mozaRxIndex = 0;
MozaState_t mozaState = MOZA_IDLE;

bool stateChanged = false;
uint32_t lastTransmitTime = 0;

// Variables de débounce pour les boutons
uint32_t lastButtonReadTime = 0;
uint8_t debouncedState[2] = {0};

// Variables pour la gestion des paquets MOZA
uint32_t lastPacketTime = 0;
bool connectionActive = false;
uint32_t successfulPackets = 0;
uint8_t mozaRxType = 0;
uint8_t mozaRxLength = 0;
uint8_t mozaRxChecksum = 0;
uint8_t mozaCalculatedChecksum = 0;
uint8_t packetState = 0;
uint32_t packetErrorCount = 0;
uint32_t lastErrorTime = 0;

// Variables pour le clignotement des LED
uint32_t ledBlinkTime = 0;
uint8_t ledBlinkCount = 0;

// Variable d'initialisation
bool systemInitialized = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void ReadButtons(void);
static void ReadEncoders(void);
static void ProcessMozaData(uint8_t* data, uint16_t size);
static void PrepareMozaPacket(void);
static bool CompareWheelStates(WheelState_t* state1, WheelState_t* state2);

// Nouvelles fonctions pour la communication MOZA
static void MOZA_SetPinOutput(void);
static void MOZA_SetPinInput(void);
static void MOZA_SendBit(uint8_t bit);
static uint8_t MOZA_ReadBit(void);
static void MOZA_SendByte(uint8_t byte);
static uint8_t MOZA_ReadByte(void);
static void SendMozaPacket(void);
/* USER CODE END PFP */
/**
  * @brief  Process data received from MOZA base
  * @param  data: Pointer to received data
  * @param  size: Size of received data
  * @retval None
  */
static void ProcessMozaData(uint8_t* data, uint16_t size)
{
  // Mise à jour du temps de dernier paquet
  lastPacketTime = HAL_GetTick();
  
  // Vérifier si le paquet a un en-tête valide
  if (size > 0 && data[0] == MOZA_HEADER_BYTE) {
    // Traiter les commandes de la base
    // Cela dépendrait du protocole MOZA spécifique
    
    // Exemple: Vérifier les données de retour de force
    if (size >= 3 && data[1] == 0x01) {
      // Traiter les données de retour de force
      // data[2] pourrait contenir la magnitude de la force
      
      // Basculer la LED verte pour indiquer la réception d'un paquet valide
      HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
      
      // Incrémenter le compteur de paquets réussis
      successfulPackets++;
      
      // Marquer la connexion comme active
      connectionActive = true;
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
  if (huart->Instance == USART2) {
    // Traiter les données MOZA
    ProcessMozaData(mozaRxBuffer, mozaRxIndex);
    mozaRxIndex = 0;
    
    // Recommencer à recevoir
    HAL_UART_Receive_IT(&huart2, &mozaRxBuffer[0], 1);
  }
}

// Définitions pour le protocole MOZA
#define MOZA_BIT_TIME_US 8  // Temps pour un bit en microsecondes (ajustez selon le protocole)

/**
  * @brief  Configure la broche de données en mode sortie
  * @retval None
  */
static void MOZA_SetPinOutput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = MOZA_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // Open-drain
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MOZA_DATA_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  Configure la broche de données en mode entrée
  * @retval None
  */
static void MOZA_SetPinInput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = MOZA_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MOZA_DATA_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  Envoie un bit sur la ligne de données
  * @param  bit: Valeur du bit à envoyer (0 ou 1)
  * @retval None
  */
static void MOZA_SendBit(uint8_t bit)
{
  MOZA_SetPinOutput();
  if (bit) {
    HAL_GPIO_WritePin(MOZA_DATA_GPIO_Port, MOZA_DATA_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(MOZA_DATA_GPIO_Port, MOZA_DATA_Pin, GPIO_PIN_RESET);
  }
  // Délai pour la durée du bit
  HAL_Delay(MOZA_BIT_TIME_US / 1000);
  if (MOZA_BIT_TIME_US % 1000 > 0) {
    // Pour les délais inférieurs à 1ms, utilisez une boucle de délai
    uint32_t startTick = HAL_GetTick();
    while (HAL_GetTick() - startTick < 1) {
      // Attente active
    }
  }
}

/**
  * @brief  Lit un bit depuis la ligne de données
  * @retval Valeur du bit lu (0 ou 1)
  */
static uint8_t MOZA_ReadBit(void)
{
  MOZA_SetPinInput();
  uint8_t bit = HAL_GPIO_ReadPin(MOZA_DATA_GPIO_Port, MOZA_DATA_Pin);
  // Délai pour la durée du bit
  HAL_Delay(MOZA_BIT_TIME_US / 1000);
  if (MOZA_BIT_TIME_US % 1000 > 0) {
    uint32_t startTick = HAL_GetTick();
    while (HAL_GetTick() - startTick < 1) {
      // Attente active
    }
  }
  return bit;
}

/**
  * @brief  Envoie un octet sur la ligne de données
  * @param  byte: Octet à envoyer
  * @retval None
  */
static void MOZA_SendByte(uint8_t byte)
{
  // Bit de départ (toujours 0)
  MOZA_SendBit(0);
  
  // 8 bits de données, LSB en premier
  for (int i = 0; i < 8; i++) {
    MOZA_SendBit((byte >> i) & 0x01);
  }
  
  // Bit de stop (toujours 1)
  MOZA_SendBit(1);
}

/**
  * @brief  Lit un octet depuis la ligne de données
  * @retval Octet lu
  */
static uint8_t MOZA_ReadByte(void)
{
  uint8_t byte = 0;
  
  // Attendre le bit de départ (0)
  while (MOZA_ReadBit() != 0) {
    // Attente active
  }
  
  // Lire 8 bits de données, LSB en premier
  for (int i = 0; i < 8; i++) {
    byte |= (MOZA_ReadBit() << i);
  }
  
  // Ignorer le bit de stop
  MOZA_ReadBit();
  
  return byte;
}

/**
  * @brief  Envoie un paquet à la base MOZA
  * @retval None
  */
static void SendMozaPacket(void)
{
  PrepareMozaPacket();  // Prépare le paquet dans mozaTxBuffer
  
  // Désactiver les interruptions pendant l'envoi
  __disable_irq();
  
  // Envoyer l'en-tête
  MOZA_SendByte(MOZA_HEADER_BYTE);
  
  // Envoyer le type de paquet
  MOZA_SendByte(mozaTxBuffer[1]);
  
  // Envoyer la longueur
  MOZA_SendByte(mozaTxBuffer[2]);
  
  // Envoyer les données
  for (int i = 0; i < mozaTxBuffer[2]; i++) {
    MOZA_SendByte(mozaTxBuffer[3 + i]);
  }
  
  // Envoyer le checksum
  MOZA_SendByte(mozaTxBuffer[3 + mozaTxBuffer[2]]);
  
  // Réactiver les interruptions
  __enable_irq();
  
  // Remettre la broche en mode entrée pour recevoir
  MOZA_SetPinInput();
}




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
  // Initialiser les variables
  systemInitialized = false;
  lastButtonReadTime = 0;
  lastPacketTime = 0;
  connectionActive = false;
  successfulPackets = 0;
  packetErrorCount = 0;
  lastErrorTime = 0;
  ledBlinkTime = 0;
  ledBlinkCount = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  


  // Start timers
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  











  // Initial LED state
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  
  // Initialize MOZA data pin as input
  MOZA_SetPinInput();
  
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
    
    // If state changed or it's time for a periodic update
    uint32_t currentTime = HAL_GetTick();
    if (stateChanged || (currentTime - lastTransmitTime > 50)) {
      // Prepare and send data to MOZA base
      if (mozaState == MOZA_IDLE) {
        SendMozaPacket();
        
        // Update last transmit time
        lastTransmitTime = currentTime;
        
        // Save current state
        memcpy(&prevWheelState, &wheelState, sizeof(WheelState_t));
      }
    }
  }
  /* USER CODE END 3 */
}