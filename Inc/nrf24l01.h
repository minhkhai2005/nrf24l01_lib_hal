
/**
 * @brief This the header file of NRF24L01 Library for STM32 Project
 */

#ifndef RADIO_A_NRF24L01_H
#define RADIO_A_NRF24L01_H

#include "stm32f103xb.h"
#include "spi.h"

// macros
# define EMPTY_DATA   (uint8_t *)0xFF

// registers
#define CONFIG        0x00
#define EN_AA         0x01
#define EN_RXADDR     0x02
#define SETUP_AW      0x03
#define SETUP_RETR    0x04
#define RF_CH         0x05
#define RF_SETUP      0x06
#define STATUS        0x07
#define OBSERVE_TX    0x08
#define PRD           0x09
#define RX_ADDR_P0    0x0A
#define RX_ADDR_P1    0x0B
#define RX_ADDR_P2    0x0C
#define RX_ADDR_P3    0x0D
#define RX_ADDR_P4    0x0E
#define RX_ADDR_P5    0x0F
#define TX_ADDR       0x10
#define RX_PW_P0      0x11
#define RX_PW_P1      0x12
#define RX_PW_P2      0x13
#define RX_PW_P3      0x14
#define RX_PW_P4      0x15
#define RX_PW_P5      0x16
#define FIFO_STATUS   0x17
#define DYNPD         0x1C
#define FEATURE       0x1D

// bit positions
/* CONFIG */
#define MASK_RX_DR    6
#define MASK_TX_DS    5
#define MASK_MAX_RT   4
#define EN_CRC        3
#define CRCO          2
#define PWR_UP        1
#define PRIM_RX       0

/* EN_AA */
#define ENAA_P5       5
#define ENAA_P4       4
#define ENAA_P3       3
#define ENAA_P2       2
#define ENAA_P1       1
#define ENAA_P0       0

/* EN_RXADDR */
#define ERX_P5        5
#define ERX_P4        4
#define ERX_P3        3
#define ERX_P2        2
#define ERX_P1        1
#define ERX_P0        0

/* RF_SETUP */
#define CONT_WAVE     7
#define RF_DR_LOW     5
#define PPL_LOCK      4
#define RF_DR_HIGH    3

/* STATUS */
#define RX_DR         6
#define TX_DS         5
#define MAX_RT        4

/* FIFO STATUS */
#define TX_REUSE      6
#define TX_FULL       5
#define TX_EMPTY      4
#define RX_FULL       1
#define RX_EMPTY      0

// commands
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define R_RX_PL_WID   0x60
#define W_ACK_PAYLOAD 0xA8   // bit[2:0] specify pipe
#define W_TX_PAYLOAD_NOACK   0xB0
#define NOP           0xFF  // no operation

typedef enum nrf24l01_rf_data_rate{
    _1Mbps = 0,
    _2Mbps,
    _250Kbps
} NRF24L01_RF_DATA_RATE;

typedef enum nrf24l01_output_power{
    _minus18dBm = 0,
    _minus12dBm,
    _minus6dBm,
    _0dBm
}NRF24L01_OUTPUT_POWER;

typedef enum nrf24l01_address_width{
    addr_width_3B = 3,
    addr_width_4B = 4,
    addr_width_5B = 5,
}NRF24L01_ADDRESS_WIDTH;

typedef enum nrf24l01_pipe{
    pipe0 = 0,
    pipe1,
    pipe2,
    pipe3,
    pipe4,
    pipe5
}NRF24L01_PIPE;

typedef enum nrf24l01_errno{
   BUFFER_SIZE_LIMIT = 0xFFU,
   TIMEOUT = 0xFEU,
   SPI_ERROR = 0xFDU,
   REGISTER_ERROR = 0xFCU,
   REGISTER_LENGTH = 0xFBU,
   NULL_POINTER = 0xFAU,
   SPI_FAIL = 0xF9U,
}NRF24L01_ERROR;

typedef struct nrf24l01_config{
    GPIO_TypeDef *irq_port, *ce_port, *csn_port;
    uint16_t irq_pin, ce_pin, csn_pin;
    SPI_HandleTypeDef* spi;
    NRF24L01_RF_DATA_RATE rf_data_rate;
    uint8_t rf_freq; // 0 = 2.400Ghz, max = 125;
    NRF24L01_OUTPUT_POWER rf_output_power;
    uint8_t payload_length;
    NRF24L01_ADDRESS_WIDTH address_width;
    uint8_t *tx_addr;
    uint8_t *rx_addr_p0;
    uint8_t *rx_addr_p1;
    uint8_t *rx_addr_p2;
    uint8_t *rx_addr_p3;
    uint8_t *rx_addr_p4;
    uint8_t *rx_addr_p5;
}NRF24L01_CONFIG;

typedef struct nrf24l01_dev{
    NRF24L01_CONFIG *config;
}NRF24L01_DEV;

/**
 * @brief check if a number is error number or not
 * @param n number to be check
 * @return true if passed parameter is an error number
 */
uint8_t ErrorCheck(uint8_t n);


// chip selector

/**
 * @brief Pull down the csn pin on nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_Select(NRF24L01_DEV *dev);

/**
 * @brief Pull up the csn pin on nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_Unselect(NRF24L01_DEV *dev);

// chip enable/disable active mode

/**
 * @brief enable RX/TX mode on nrf24l01
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_Enable(NRF24L01_DEV *dev);

/**
 * @brief disable RX/TX mode on nrf24l01
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_Disable(NRF24L01_DEV *dev);

// Register access

/**
 * @brief Send command and data attached to nrf24l01
 * @param dev pointer points to nrf24l01 struct
 * @param command command to be sent to nrf24l01 module
 * @param pData pointer points to data segment which be sent with the command to nrf24l01 module
 * @param length total length of command and data in byte
 * @return value of STATUS register or nrf24l01 error number
 * @note command will always 1 byte, the minimum value of *length* is 1
 * @warning if parameter *length* is less than 1, it may catch undefined behavior
 */
uint8_t nrf24l01_SendCommand(NRF24L01_DEV *dev,uint8_t command, uint8_t *pData, uint8_t length);

/**
 * @brief Read a register's value from nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param RegToRead address of register to be read
 * @param pData pointer points to data segment which recieve the register's value
 * @param length length of register's value to be read
 * @return value of STATUS register of nrf24l01 error number
 */
uint8_t nrf24l01_ReadRegister(NRF24L01_DEV *dev,uint8_t RegToRead, uint8_t *pData, uint8_t length);

/**
 * @brief Write value to a register on nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param RegToWrite address of register to be written
 * @param pData pointer points to data segment which to be written to register
 * @param length length of data to be written
 * @return value of STATUS register or nrf24l01 error number
 */
uint8_t nrf24l01_WriteRegister(NRF24L01_DEV *dev, uint8_t RegToWrite, uint8_t *pData, uint8_t length);

// initialization and configuration functions

/**
 *  @brief Initialize nrf24l01 module
 *  @param dev pointer points to nrf24l01 struct
 *  @return None
 *  @note nrf24l01 module will initially be a PRX device,
 *  if you want to make it a PTX call MakePTX function after
 *  initializtion
 */
void nrf24l01_init(NRF24L01_DEV *dev);

/**
 * @brief Set RF Channel of nrf24l01 module
 * @param dev poiter points to nrf24l01 struct
 * @param channel channel to be set for nrf24l01 module
 * @return None
 * @note *channel* parameter has value between 0 and 125
 */
void nrf24l01_SetChannel(NRF24L01_DEV *dev, uint8_t channel);

/**
 * @brief Set RF transmission data rate of nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param rate data rate to be set
 * @return None
 */
void nrf24l01_SetDataRate(NRF24L01_DEV *dev, NRF24L01_RF_DATA_RATE rate);

/**
 * @brief Set RF output power of nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param outputPower output power to be set
 * @return None
 */
void nrf24l01_SetOutputPower(NRF24L01_DEV *dev, NRF24L01_OUTPUT_POWER outputPower);

/**
 * @brief Set width of address for nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param addressWidth width of address to be set
 * @return None
 */
void nrf24l01_SetAddressWidth(NRF24L01_DEV *dev, NRF24L01_ADDRESS_WIDTH addressWidth);

/**
 * @brief Set recieve address for a specific pipe of nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param pipe pipe to set address for
 * @param pAddr pointer points to address data segment
 * @param length length of address data segment
 * @return None
 */
void nrf24l01_SetRXAddress(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t *pAddr, uint8_t length);

/**
 * @brief Set transmit address of nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param length length of address data segment
 * @return None
 */
void nrf24l01_SetTXAddress(NRF24L01_DEV *dev, uint8_t *pAddr, uint8_t length);

/**
 * @brief Enable/Disable auto acknowledgement for a specific pipe of nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param pipe pipe to set auto acknowledgement for
 * @param enable boolean value to enable or diasble auto acknowledgement (0: diasble)
 * @return None
 */
void nrf24l01_SetAutoACK(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t enable);

/**
 * @brief Enable/Disable dynamic payload length for a specific pipe of nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param pipe pipe to set dynamic payload length for
 * @param enable boolean value to enable or diasble dynamic payload length (0: diasble)
 * @return None
 */
void nrf24l01_SetDynamicPayloadLength(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t enable);

/**
 * @brief Set number of retransmission of nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @param delay Time interval between retransmissions
 * @param count maximum number of retransmissions
 * @return None
 */
void nrf24l01_SetRetries(NRF24L01_DEV *dev, uint8_t delay, uint8_t count);

/**
 * @brief Set payload length for a specific rx pipe
 * @param dev pointer points to nrf24l01 struct
 * @param pipe pipe to set payload length
 * @param length expected rx payload length to be set in bytes
 * @return None
 */
void nrf24l01_SetRXPayloadLength(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t length);

/**
 * @brief Make nrf24l01 module become PTX device
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_MakePTX(NRF24L01_DEV* dev);

/**
 * @brief Make nrf24l01 module become PRX device
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_MakePRX(NRF24L01_DEV *dev);

// data transmission

/**
 * @brief Write a data payload to TX FIFO
 * @param dev pointer points to nrf24l01 struct
 * @param pData data payload to be written to TX FIFO
 * @param length length of data payload
 * @return STATUS register's value or nrf24l01 error code
 */
uint8_t nrf24l01_WritePayload(NRF24L01_DEV *dev, uint8_t *pData, uint8_t length);

/**
 * @brief attach a data payload to send along with ACK packet
 * @param dev pointer points to nrf24l01 struct
 * @param pipe pipe to be attached
 * @param pData pointer points to data segment
 * @param length length of data segment
 * @return STATUS register's value or nrf24l01 error code
 */
uint8_t nrf24l01_WriteACKPayload(NRF24L01_DEV *dev,  NRF24L01_PIPE pipe, uint8_t *pData, uint8_t length);

/**
 * @brief Flush TX FIFO
 * @param dev pointer points to nrf24l01 struct
 * @return STATUS register's value or nrf24l01 error code
 */
uint8_t nrf24l01_FlushTX(NRF24L01_DEV *dev);

/**
 * @brief Transmit a data payload
 * @param dev pointer points to nrf24l01 struct
 * @param pData pointer points to data segment
 * @param length length of data segment
 * @return STATUS register's value or nrf24l01 error code
 * @note the length of data segment must be equal to payload_length
 */
uint8_t nrf24l01_Transmit(NRF24L01_DEV *dev, uint8_t *pData, uint8_t length);

/**
 * @brief Put nrf24l01 module into RX mode for searching for packets
 * @brief dev poiter points to nrf24l01 struct
 * @return None
 */
uint8_t nrf24l01_Receive(NRF24L01_DEV *dev);

// data reception

/**
 * @brief Read a data payload from RX FIFO
 * @param dev pointer points to nrf24l01 struct
 * @param pData pointer points to data segment
 * @param length length of data segment
 * @return STATUS register's value or nrf24l01 error code
 */
uint8_t nrf24l01_ReadPayload(NRF24L01_DEV *dev, uint8_t *pData, uint8_t length);

/**
 * @brief Flush RX FIFO
 * @param dev pointer points to nrf24l01 struct
 * @return STATUS register's value or nrf24l01 error code
 */
uint8_t nrf24l01_FlushRX(NRF24L01_DEV *dev);

/**
 * @brief Get pipe number of the payload available for reading from RX FIFO
 * @param dev pointer poinst to nrf24l01 struct
 * @return pipe enum type
 * @note if error occurs function will return 0xff
 */
NRF24L01_PIPE nrf24l01_GetRXPipe(NRF24L01_DEV *dev);

/**
 * @brief Get RX/TX payload length
 * @param dev pointer points to nrf24l01 struct
 * @return payload length
 * @note if error occurs function will return 0xff
 */
uint8_t nrf24l01_GetPayloadLength(NRF24L01_DEV *dev);

// status and interrupt handling

/**
 * @brief Get STATUS register's value
 * @param dev pointer points to nrf24l01 struct
 * @return STATUS register's value or nrf24l01 error code
 */
uint8_t nrf24l01_GetStatus(NRF24L01_DEV *dev);

/**
 * @brief Check if a specific interrupt flag is set
 * @param dev pointer points to nrf24l01 struct
 * @param flag interrupt flag to be checked
 * @return true if interrupt flag is set
 */
uint8_t nrf24l01_CheckInterrupt(NRF24L01_DEV *dev, uint8_t flag);

/**
 * @brief Clear a specific interrupt flag
 * @param dev pointer points to nrf24l01 struct
 * @param flag interrupt flag to be cleared
 * @return STATUS register's value or nrf24l01 error code
 */
void nrf24l01_ClearInterrupt(NRF24L01_DEV* dev, uint8_t flag);

/**
 * @brief Clear all interrupt flags
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_ClearInterruptFlags(NRF24L01_DEV *dev);

// power management

/**
 * @brief Power up nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_PowerUp(NRF24L01_DEV *dev);

/**
 * @brief Power down nrf24l01 module
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_PowerDown(NRF24L01_DEV *dev);

/**
 * @brief Put nrf24l01 module to Standby-I mode
 * @param dev pointer points to nrf24l01 struct
 * @return None
 */
void nrf24l01_EnterStandby(NRF24L01_DEV *dev);

// FIFO and Pipe management

/**
 * @brief Check if TX FIFO is full
 * @param dev pointer points to nrf24l01 struct
 * @return boolean value indicates that TX FIFO is full or not
 */
uint8_t nrf24l01_IsTXFull(NRF24L01_DEV *dev);

/**
 * @brief Check if TX FIFO is empty
 * @param dev pointer points to nrf24l01 struct
 * @return boolean value indicates that TX FIFO is empty or not
 */
uint8_t nrf24l01_IsTXEmpty(NRF24L01_DEV *dev);

/**
 * @brief Check if RX FIFO is full
 * @param dev pointer points to nrf24l01 struct
 * @return boolean value indicates that RX FIFO is full or not
 */
uint8_t nrf24l01_IsRXFull(NRF24L01_DEV *dev);

/**
 * @brief Check if RX FIFO is empty
 * @param dev pointer points to nrf24l01 struct
 * @return boolean value indicates that RX FIFO is empty or not
 */
uint8_t nrf24l01_IsRXEmpty(NRF24L01_DEV *dev);

/**
 * @brief Enable specific data pipe
 * @param dev pointer points to nrf24l01 struct
 * @param pipe pipe to be enabled
 * @return None
 */
void nrf24l01_EnablePipe(NRF24L01_DEV *dev, NRF24L01_PIPE pipe);

#endif //RADIO_A_NRF24L01_H
