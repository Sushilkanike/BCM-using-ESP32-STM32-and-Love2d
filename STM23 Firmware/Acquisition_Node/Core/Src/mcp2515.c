#include "mcp2515.h"

/*
 * MCP2515 SPI Commands
 */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_READ_STATUS 0xA0
#define MCP_BITMOD      0x05
#define MCP_LOAD_TXB0   0x40
#define MCP_RTS_TX0     0x81

/*
 * MCP2515 Registers
 */
#define MCP_CANCTRL     0x0F
#define MCP_CANSTAT     0x0E
#define MCP_CNF1        0x2A
#define MCP_CNF2        0x29
#define MCP_CNF3        0x28
#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31
#define MCP_TXB0DLC     0x35
#define MCP_TXB0D0      0x36
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB0DLC     0x65
#define MCP_RXB0D0      0x66
#define MCP_CANINTF     0x2C
#define MCP_RXF0SIDH    0x00

/*
 * Private Variables
 */
static SPI_HandleTypeDef* mcp_hspi;
static GPIO_TypeDef* mcp_cs_port;
static uint16_t           mcp_cs_pin;

/*
 * Private Function Prototypes
 */
static void MCP_Select(void);
static void MCP_Deselect(void);
static void MCP_WriteByte(uint8_t reg, uint8_t data);
static uint8_t MCP_ReadByte(uint8_t reg);
static void MCP_Reset(void);
static void MCP_SetBitrate(uint8_t clock_mhz, uint16_t bitrate_kbps);

/*
 * --- Public Functions ---
 */

MCP2515_StatusTypeDef MCP2515_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t clock_mhz, uint16_t bitrate_kbps)
{
    mcp_hspi = hspi;
    mcp_cs_port = cs_port;
    mcp_cs_pin = cs_pin;

    MCP_Deselect();
    MCP_Reset();

    // Wait for reset
    HAL_Delay(10);

    // Set CAN bitrate
    MCP_SetBitrate(clock_mhz, bitrate_kbps);

    // Enable RXB0 interrupt
    MCP_WriteByte(0x2B, 0x01); // CANINTE: RX0IE = 1

    // Set mode to normal
    MCP_WriteByte(MCP_CANCTRL, 0x00);

    // Verify mode
    if((MCP_ReadByte(MCP_CANSTAT) & 0xE0) != 0x00)
    {
        return MCP2515_ERROR;
    }

    return MCP2515_OK;
}

MCP2515_StatusTypeDef MCP2515_SendMessage(CAN_TxHeaderTypeDef* txHeader, uint8_t* txData)
{
    // Check if TXB0 is free
    if (MCP_ReadByte(MCP_TXB0CTRL) & 0x08) // TXREQ
    {
        return MCP2515_ALLTXBUSY;
    }

    // --- Load TX Buffer 0 ---
    MCP_Select();
    HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){MCP_LOAD_TXB0}, 1, HAL_MAX_DELAY);

    // Set Identifier
    if(txHeader->IDE == CAN_ID_STD)
    {
        HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){(txHeader->StdId >> 3)}, 1, HAL_MAX_DELAY); // SIDH
        HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){(txHeader->StdId << 5)}, 1, HAL_MAX_DELAY); // SIDL
        HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){0x00}, 1, HAL_MAX_DELAY); // EID8
        HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){0x00}, 1, HAL_MAX_DELAY); // EID0
    }
    else // (txHeader->IDE == CAN_ID_EXT)
    {
        // Not fully implemented in this minimal driver
        // You would set StdId and ExtId bits across SIDH, SIDL, EID8, EID0
    }

    // Set DLC and RTR
    uint8_t dlc_byte = txHeader->DLC & 0x0F;
    if(txHeader->RTR == CAN_RTR_REMOTE)
    {
        dlc_byte |= 0x40; // Set RTR bit
    }
    HAL_SPI_Transmit(mcp_hspi, &dlc_byte, 1, HAL_MAX_DELAY);

    // Set Data
    HAL_SPI_Transmit(mcp_hspi, txData, txHeader->DLC, HAL_MAX_DELAY);
    MCP_Deselect();

    // --- Request to Send TX Buffer 0 ---
    MCP_Select();
    HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){MCP_RTS_TX0}, 1, HAL_MAX_DELAY);
    MCP_Deselect();

    return MCP2515_OK;
}

uint8_t MCP2515_CheckReceive(void)
{
    uint8_t status = MCP_ReadByte(MCP_CANSTAT);
    if(status & 0x01) // RX0IF
    {
        return 1;
    }
    return 0;
}

MCP2515_StatusTypeDef MCP2515_ReadMessage(CAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData)
{
    // Check for RX0 interrupt flag
    if(!MCP2515_CheckReceive())
    {
        return MCP2515_ERROR;
    }

    MCP_Select();
    HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){0x90}, 1, HAL_MAX_DELAY); // Read RX Buffer 0

    // Read Identifier
    uint8_t sidh = 0;
    uint8_t sidl = 0;
    HAL_SPI_Receive(mcp_hspi, &sidh, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(mcp_hspi, &sidl, 1, HAL_MAX_DELAY);
    
    // Check IDE
    rxHeader->IDE = (sidl & 0x08) ? CAN_ID_EXT : CAN_ID_STD;

    if(rxHeader->IDE == CAN_ID_STD)
    {
        rxHeader->StdId = (sidh << 3) | (sidl >> 5);
    }
    else
    {
        // Reading EXT ID not implemented in this minimal driver
    }
    
    HAL_SPI_Receive(mcp_hspi, (uint8_t[]){0,0}, 2, HAL_MAX_DELAY); // Skip EID8, EID0

    // Read DLC
    uint8_t dlc_byte = 0;
    HAL_SPI_Receive(mcp_hspi, &dlc_byte, 1, HAL_MAX_DELAY);
    rxHeader->DLC = dlc_byte & 0x0F;
    rxHeader->RTR = (dlc_byte & 0x40) ? CAN_RTR_REMOTE : CAN_RTR_DATA;

    // Read Data
    HAL_SPI_Receive(mcp_hspi, rxData, rxHeader->DLC, HAL_MAX_DELAY);
    MCP_Deselect();

    // Clear RX0IF flag
    MCP_WriteByte(MCP_CANINTF, 0x00); // Clear RX0IF

    return MCP2515_OK;
}


/*
 * --- Private Functions ---
 */

static void MCP_Select(void)
{
    HAL_GPIO_WritePin(mcp_cs_port, mcp_cs_pin, GPIO_PIN_RESET);
}

static void MCP_Deselect(void)
{
    HAL_GPIO_WritePin(mcp_cs_port, mcp_cs_pin, GPIO_PIN_SET);
}

static void MCP_WriteByte(uint8_t reg, uint8_t data)
{
    uint8_t txData[3] = {MCP_WRITE, reg, data};
    MCP_Select();
    HAL_SPI_Transmit(mcp_hspi, txData, 3, HAL_MAX_DELAY);
    MCP_Deselect();
}

static uint8_t MCP_ReadByte(uint8_t reg)
{
    uint8_t txData[2] = {MCP_READ, reg};
    uint8_t rxData = 0;
    MCP_Select();
    HAL_SPI_Transmit(mcp_hspi, txData, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(mcp_hspi, &rxData, 1, HAL_MAX_DELAY);
    MCP_Deselect();
    return rxData;
}

static void MCP_Reset(void)
{
    MCP_Select();
    HAL_SPI_Transmit(mcp_hspi, (uint8_t[]){MCP_RESET}, 1, HAL_MAX_DELAY);
    MCP_Deselect();
}

static void MCP_SetBitrate(uint8_t clock_mhz, uint16_t bitrate_kbps)
{
    if(clock_mhz != 8)
        return; // only 8MHz supported here

    switch(bitrate_kbps)
    {
        case 1000: // 1 Mbps @ 8MHz
            MCP_WriteByte(MCP_CNF1, 0x00);
            MCP_WriteByte(MCP_CNF2, 0x80 | (1<<3) | (0<<0)); // BTLMODE=1, PHSEG1=1, PRSEG=0
            MCP_WriteByte(MCP_CNF3, 0x01); // PHSEG2=1
            break;

        case 500: // 500 kbps @ 8MHz
            MCP_WriteByte(MCP_CNF1, 0x00);
            MCP_WriteByte(MCP_CNF2, 0x90); 
            MCP_WriteByte(MCP_CNF3, 0x02);
            break;

        case 250: // 250 kbps @ 8MHz
            MCP_WriteByte(MCP_CNF1, 0x01);
            MCP_WriteByte(MCP_CNF2, 0x90);
            MCP_WriteByte(MCP_CNF3, 0x02);
            break;

        case 125: // 125 kbps @ 8MHz
            MCP_WriteByte(MCP_CNF1, 0x03);
            MCP_WriteByte(MCP_CNF2, 0x90);
            MCP_WriteByte(MCP_CNF3, 0x02);
            break;

        default:
            // Unsupported bitrate -> do nothing
            break;
    }
}
    // MCP_WriteByte(MCP_CNF3, 0x02); // SOF=0, WAKFIL=0, PHSEG2=2

}
