/*
 * mcp21525.h
 *
 *  Created on: Nov 4, 2025
 *      Author: ksushil
 */

#ifndef INC_MCP2515_H_
#define INC_MCP2515_H_

#include "main.h" // Includes your HAL and SPI/GPIO handles

/*
 * CAN_TxHeaderTypeDef - Matches the HAL CAN structure for familiarity
 */

typedef struct
{
  uint32_t StdId;    /*!< Standard identifier. (11 bits) */
  uint32_t ExtId;    /*!< Extended identifier. (29 bits) */
  uint8_t  IDE;      /*!< Identifier type: CAN_ID_STD or CAN_ID_EXT */
  uint8_t  RTR;      /*!< Frame type: CAN_RTR_DATA or CAN_RTR_REMOTE */
  uint8_t  DLC;      /*!< Data Length Code: 0-8 bytes */
} CAN_TxHeaderTypeDef;

/*
 * CAN_RxHeaderTypeDef - Matches the HAL CAN structure
 */
typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint8_t  IDE;
  uint8_t  RTR;
  uint8_t  DLC;
  uint8_t  FilterMatchIndex;
} CAN_RxHeaderTypeDef;

/*
 * MCP2515 Return Status
 */
typedef enum
{
  MCP2515_OK       = 0x00,
  MCP2515_ERROR    = 0x01,
  MCP2515_ALLTXBUSY = 0x02
} MCP2515_StatusTypeDef;

// Identifier Types
#define CAN_ID_STD 0
#define CAN_ID_EXT 1

// Remote Frame Types
#define CAN_RTR_DATA   0
#define CAN_RTR_REMOTE 1

/*
 * Function Prototypes
 */

/**
 * @brief Initializes the MCP2515 controller.
 * @param hspi: Pointer to your SPI_HandleTypeDef.
 * @param cs_port: The GPIO_TypeDef* (e.g., GPIOA) for your CS pin.
 * @param cs_pin: The uint16_t (e.g., GPIO_PIN_4) for your CS pin.
 * @param clock_mhz: The crystal frequency on the MCP2515 board (usually 8 or 16).
 * @param bitrate_kbps: The desired CAN bus speed (e.g., 500 for 500kbps).
 * @retval MCP2515_StatusTypeDef
 */
MCP2515_StatusTypeDef MCP2515_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t clock_mhz, uint16_t bitrate_kbps);

/**
 * @brief Sends a CAN message.
 * @param txHeader: Pointer to the header structure (ID, DLC, etc.).
 * @param txData: Pointer to the 8-byte data array.
 * @retval MCP2515_StatusTypeDef
 */
MCP2515_StatusTypeDef MCP2515_SendMessage(CAN_TxHeaderTypeDef* txHeader, uint8_t* txData);

/**
 * @brief Checks if a message is available.
 * @retval 0 if no message, >0 if messages are pending.
 */
uint8_t MCP2515_CheckReceive(void);

/**
 * @brief Reads a CAN message.
 * @param rxHeader: Pointer to a header structure to be filled.
 * @param rxData: Pointer to an 8-byte array to be filled.
 * @retval MCP2515_StatusTypeDef
 */
MCP2515_StatusTypeDef MCP2515_ReadMessage(CAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData);

#endif /* INC_MCP2515_H_ */
