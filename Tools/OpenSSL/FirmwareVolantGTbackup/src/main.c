/* Includes */
#include "stm32h7xx_hal.h"
#include "main.h"

/* Variables */
UART_HandleTypeDef huart2;  // Pour la communication série
uint8_t rxBuffer[64];
uint8_t txBuffer[64];

/* Constantes */
#define MOZA_DEVICE_ID 0x42  // ID du périphérique
#define MOZA_PROTOCOL_VERSION 0x01

/* Structure des messages */
typedef struct {
    uint8_t header;
    uint8_t deviceId;
    uint8_t length;
    uint8_t data[60];
    uint8_t checksum;
} MozaMessage;

/* Fonctions */
void SystemClock_Config(void);
void UART2_Init(void);
void SendMozaIdentification(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    UART2_Init();

    while (1)
    {
        SendMozaIdentification();
        HAL_Delay(1000);
    }
}

void SendMozaIdentification(void)
{
    MozaMessage msg = {
        .header = 0xAA,
        .deviceId = MOZA_DEVICE_ID,
        .length = 2,
        .data = {MOZA_PROTOCOL_VERSION, 0x00}
    };
    
    // Calcul checksum
    msg.checksum = msg.deviceId + msg.length + msg.data[0] + msg.data[1];
    
    // Envoi du message
    HAL_UART_Transmit(&huart2, (uint8_t*)&msg, sizeof(MozaMessage), 100);
}