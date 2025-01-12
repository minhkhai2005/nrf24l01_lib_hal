//
// Created by Trần Minh Khải on 20/12/24.
//

#include "nrf24l01.h"
#include "malloc.h"
#include "string.h"

uint8_t ErrorCheck(uint8_t n){
  switch (n) {
    case BUFFER_SIZE_LIMIT:
    case TIMEOUT:
    case SPI_ERROR:
    case REGISTER_ERROR:
    case REGISTER_LENGTH:
    case NULL_POINTER:
    case SPI_FAIL:
      return 1;
    default:
      return 0;
  }
}
void nrf24l01_Select(NRF24L01_DEV *dev){
  HAL_GPIO_WritePin(dev->config->csn_port, dev->config->csn_pin, GPIO_PIN_RESET);
}

void nrf24l01_Unselect(NRF24L01_DEV *dev){
  HAL_GPIO_WritePin(dev->config->csn_port,dev->config->csn_pin, GPIO_PIN_SET);
}

void nrf24l01_Enable(NRF24L01_DEV *dev){
  HAL_GPIO_WritePin(dev->config->ce_port, dev->config->ce_pin, GPIO_PIN_SET)  ;
}

void nrf24l01_Disable(NRF24L01_DEV *dev){
  HAL_GPIO_WritePin(dev->config->ce_port, dev->config->ce_pin, GPIO_PIN_RESET)  ;
}

uint8_t nrf24l01_SendCommand(NRF24L01_DEV *dev,uint8_t command, uint8_t *pData, uint8_t length){
  uint8_t status = 0;
  uint8_t Tx_buffer[33];
  uint8_t Rx_buffer[33];

  // validate data length
  if (length - 1 > 32) return BUFFER_SIZE_LIMIT; // check if the length of data is over 32 bytes

  // prepare the transmit buffer
  Tx_buffer[0] = command;
  if  (command == W_TX_PAYLOAD || command == W_TX_PAYLOAD_NOACK || (command & 0xe0) == W_REGISTER || (command & 0xf8) == W_ACK_PAYLOAD){
    memcpy(Tx_buffer + 1, pData, length);
  }
  else{
    memset(Tx_buffer + 1, 0, sizeof (Tx_buffer) - 1);
  }

  // select nrf24l01 module
  nrf24l01_Select(dev);

  // wait for spi module to be ready
  uint16_t timeout = 10000;
  while (HAL_SPI_GetState(dev->config->spi) != HAL_SPI_STATE_READY){
    if (--timeout == 0){
      nrf24l01_Unselect(dev);
      return TIMEOUT;
    }
  }

  // perform spi transmit
  HAL_StatusTypeDef result = 0;
  result = HAL_SPI_TransmitReceive(dev->config->spi, Tx_buffer, Rx_buffer, length, 100);

  // unselect the nrf24l01 module
  nrf24l01_Unselect(dev);

  // check spi communication result
  if (result != HAL_OK){
    return SPI_FAIL;
  }

  // extract STATUS register value
  status = Rx_buffer[0];

  // Copy payload data for applicable commands
  if ((command >> 5) == R_REGISTER || command == R_RX_PAYLOAD || command == R_RX_PL_WID)
    memcpy(pData, Rx_buffer + 1, length - 1);

  return status; // return STATUS register value
}

uint8_t nrf24l01_ReadRegister(NRF24L01_DEV *dev, uint8_t RegToRead, uint8_t *pData, uint8_t length){
  uint8_t status = 0;

  // validate input
  if (RegToRead > 0x1FU)
    return REGISTER_ERROR;
  if (pData == NULL)
    return NULL_POINTER;
  if (length > 5)
    return REGISTER_LENGTH;

  // construct the read command
  uint8_t cmd = R_REGISTER | RegToRead;

  // send command and retrieve data
  status = nrf24l01_SendCommand(dev,cmd,pData,length + 1);
  return status; // return status of read operation (it may be STATUS register value or nrf24l01 error number).
}

uint8_t nrf24l01_WriteRegister(NRF24L01_DEV *dev, uint8_t RegToWrite, uint8_t *pData, uint8_t length){
  uint8_t status = 0;

  // validate input
  if (RegToWrite > 0x1FU)
    return REGISTER_ERROR;
  if (pData == NULL)
    return NULL_POINTER;
  if (length > 5)
    return REGISTER_LENGTH;

  // construct the write command;
  uint8_t cmd = W_REGISTER | RegToWrite;


  // send command and retrieve data
  status = nrf24l01_SendCommand(dev, cmd,pData,length + 1);
  return status;
}

void nrf24l01_init(NRF24L01_DEV *dev){
  nrf24l01_Disable(dev);
  HAL_Delay(100);

  // Power-up the module
  nrf24l01_PowerUp(dev);
  HAL_Delay(2);

  // Make PRX
  nrf24l01_MakePRX(dev);

  // Set RF Channel
  nrf24l01_SetChannel(dev, dev->config->rf_freq);

  // Set RF Power and Data rate
  nrf24l01_SetDataRate(dev, dev->config->rf_data_rate);
  nrf24l01_SetOutputPower(dev,dev->config->rf_output_power);

  // Config address width
  nrf24l01_SetAddressWidth(dev, dev->config->address_width);

  // Config RX/TX Address
  nrf24l01_SetTXAddress(dev, dev->config->tx_addr, dev->config->address_width);
  nrf24l01_SetRXAddress(dev,pipe0,dev->config->rx_addr_p0, dev->config->address_width);
  nrf24l01_SetRXAddress(dev, pipe1, dev->config->rx_addr_p1, dev->config->address_width);
  nrf24l01_SetRXAddress(dev, pipe2, dev->config->rx_addr_p2,1 );
  nrf24l01_SetRXAddress(dev, pipe3, dev->config->rx_addr_p3, 1);
  nrf24l01_SetRXAddress(dev, pipe4, dev->config->rx_addr_p4, 1);
  nrf24l01_SetRXAddress(dev, pipe5, dev->config->rx_addr_p5, 1);

  // Enable Auto ACK and Dynamic Payload
  nrf24l01_SetAutoACK(dev, pipe0, 1);
  nrf24l01_SetAutoACK(dev, pipe1, 1);
  nrf24l01_SetDynamicPayloadLength(dev, pipe0, 1);
  nrf24l01_SetDynamicPayloadLength(dev, pipe1, 1);

  // Flush FIFOs
  nrf24l01_FlushRX(dev);
  nrf24l01_FlushTX(dev);

  // Clear all interrupts
  nrf24l01_ClearInterruptFlags(dev);

  // Set payload length
  nrf24l01_SetRXPayloadLength(dev,pipe0,dev->config->payload_length);
  nrf24l01_SetRXPayloadLength(dev,pipe1,dev->config->payload_length);
  nrf24l01_SetRXPayloadLength(dev,pipe2,dev->config->payload_length);
  nrf24l01_SetRXPayloadLength(dev,pipe3,dev->config->payload_length);
  nrf24l01_SetRXPayloadLength(dev,pipe4,dev->config->payload_length);
  nrf24l01_SetRXPayloadLength(dev,pipe5,dev->config->payload_length);
}

void nrf24l01_SetChannel(NRF24L01_DEV *dev, uint8_t channel){
  nrf24l01_WriteRegister(dev, RF_CH, &channel, 1);
}

void nrf24l01_SetDataRate(NRF24L01_DEV *dev, NRF24L01_RF_DATA_RATE rate){
  uint8_t rf_setup;
  uint8_t status = nrf24l01_ReadRegister(dev,RF_SETUP,&rf_setup, 1);
  if (ErrorCheck(status)) return;
  switch (rate) {
    case _250Kbps:
      rf_setup |= (1 << RF_DR_LOW);
      rf_setup &= 0xf7;
      break;
    case _1Mbps:
      rf_setup &= 0xd7;
      break;
    case _2Mbps:
      rf_setup |= (1 << RF_DR_HIGH);
      rf_setup &= 0xdf;
      break;
    default:
      return;
  }
  nrf24l01_WriteRegister(dev, RF_SETUP, &rf_setup, 1);
}

void nrf24l01_SetOutputPower(NRF24L01_DEV *dev, NRF24L01_OUTPUT_POWER outputPower){
  uint8_t rf_setup;
  uint8_t status = nrf24l01_ReadRegister(dev,RF_SETUP,&rf_setup, 1);
  if (ErrorCheck(status)) return;
  switch (outputPower) {
    case _minus18dBm:
      rf_setup &= 0xf8;
      break;
    case _minus12dBm:
      rf_setup &= 0xfa;
      rf_setup |= (1 << 1);
      break;
    case _minus6dBm:
      rf_setup &= 0xfc;
      rf_setup |= (1 << 2);
      break;
    case _0dBm:
      rf_setup |= (1 << 1);
      rf_setup |= (1 << 2);
      break;
    default:
      return;
  }
  nrf24l01_WriteRegister(dev, RF_SETUP, &rf_setup, 1);
}

void nrf24l01_SetAddressWidth(NRF24L01_DEV *dev, NRF24L01_ADDRESS_WIDTH addressWidth){
  uint8_t setup_aw = 0;
  switch (addressWidth) {
    case addr_width_3B:
      setup_aw = 0b01;
      break;
    case addr_width_4B:
      setup_aw = 0b10;
      break;
    case addr_width_5B:
      setup_aw = 0b11;
      break;
    default:
      return;
  }
  nrf24l01_WriteRegister(dev, SETUP_AW, &setup_aw, 1);
}

void nrf24l01_SetRXAddress(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t *pAddr, uint8_t length){
  // validate address width
  if (pipe == pipe1 || pipe == pipe0){
    if (length < dev->config->address_width)
      return;
  }
  else if (length > 1) return;

  switch (pipe) {
    case pipe0:
      nrf24l01_WriteRegister(dev, RX_ADDR_P0, pAddr, length);
      break;
    case pipe1:
      nrf24l01_WriteRegister(dev, RX_ADDR_P1, pAddr, length);
      break;
    case pipe2:
      nrf24l01_WriteRegister(dev, RX_ADDR_P2, pAddr, length);
      break;
    case pipe3:
      nrf24l01_WriteRegister(dev, RX_ADDR_P3, pAddr, length);
      break;
    case pipe4:
      nrf24l01_WriteRegister(dev, RX_ADDR_P4, pAddr, length);
      break;
    case pipe5:
      nrf24l01_WriteRegister(dev, RX_ADDR_P5, pAddr, length);
      break;
    default:
      return;
  }
}

void nrf24l01_SetTXAddress(NRF24L01_DEV *dev, uint8_t *pAddr, uint8_t length){
  if (length < dev->config->address_width) return;
  nrf24l01_WriteRegister(dev, TX_ADDR, pAddr, length);
}

void nrf24l01_SetAutoACK(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t enable){
  uint8_t en_aa, status;
  // read out EN_AA register
  status = nrf24l01_ReadRegister(dev, EN_AA, &en_aa, 1);
  // check if any error occurs;
  if (ErrorCheck(status)) return;
  uint8_t mask = 1;
  switch (pipe) {
    case pipe0:
      mask <<= ENAA_P0;
      break;
    case pipe1:
      mask <<= ENAA_P1;
      break;
    case pipe2:
      mask <<= ENAA_P2;
      break;
    case pipe3:
      mask <<= ENAA_P3;
      break;
    case pipe4:
      mask <<= ENAA_P4;
      break;
    case pipe5:
      mask <<= ENAA_P5;
      break;
    default:
      return;
  }
  if (!enable){
    mask ^= 0xff;
    en_aa &= mask;
    nrf24l01_WriteRegister(dev, EN_AA, &en_aa, 1);
  }
  else {
    en_aa |= mask;
    nrf24l01_WriteRegister(dev, EN_AA, &en_aa, 1);
  }
}

void nrf24l01_SetDynamicPayloadLength(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t enable){
  uint8_t dynpd, status;
  // read out DYNPD register
  status = nrf24l01_ReadRegister(dev, DYNPD, &dynpd, 1);
  // check if any error occurs
  if (ErrorCheck(status)) return;

  uint8_t mask = 1;
  switch (pipe) {
    case pipe0:
      mask <<= ENAA_P0;
      break;
    case pipe1:
      mask <<= ENAA_P1;
      break;
    case pipe2:
      mask <<= ENAA_P2;
      break;
    case pipe3:
      mask <<= ENAA_P3;
      break;
    case pipe4:
      mask <<= ENAA_P4;
      break;
    case pipe5:
      mask <<= ENAA_P5;
      break;
    default:
      return;
  }
  if (!enable){
    mask ^= 0xff;
    dynpd &= mask;
    nrf24l01_WriteRegister(dev, EN_AA, &dynpd, 1);
  }
  else {
    dynpd |= mask;
    nrf24l01_WriteRegister(dev, EN_AA, &dynpd, 1);
  }
}

void nrf24l01_SetRetries(NRF24L01_DEV *dev, uint8_t delay, uint8_t count){
  if (count > 15) return;
  delay = delay << 4;
  count &= 0b00001111;
  count |= delay;
  nrf24l01_WriteRegister(dev, SETUP_RETR, &count, 1);
}

void nrf24l01_SetRXPayloadLength(NRF24L01_DEV *dev, NRF24L01_PIPE pipe, uint8_t length){
  if (length > 32) return;
  uint8_t reg = 0;
  switch (pipe) {
    case pipe0:
      reg = RX_PW_P0;
      break;
    case pipe1:
      reg = RX_PW_P1;
      break;
    case pipe2:
      reg = RX_PW_P2;
      break;
    case pipe3:
      reg = RX_PW_P3;
      break;
    case pipe4:
      reg = RX_PW_P4;
      break;
    case pipe5:
      reg = RX_PW_P5;
      break;
    default:
      return;
  }
  nrf24l01_WriteRegister(dev, reg, &length, 1);
}

uint8_t nrf24l01_WritePayload(NRF24L01_DEV *dev, uint8_t *pData, uint8_t length){
  uint8_t status;
  status = nrf24l01_SendCommand(dev, W_TX_PAYLOAD, pData, length + 1);
  return status;
}

uint8_t nrf24l01_WriteACKPayload(NRF24L01_DEV *dev,  NRF24L01_PIPE pipe, uint8_t *pData, uint8_t length){
  uint8_t status = 0;
  uint8_t cmd = W_ACK_PAYLOAD;
  switch (pipe) {
    case pipe0:
      cmd |= 0;
      break;
    case pipe1:
      cmd |= 1;
      break;
    case pipe2:
      cmd |= 2;
      break;
    case pipe3:
      cmd |= 3;
      break;
    case pipe4:
      cmd |= 4;
      break;
    case pipe5:
      cmd |= 5;
      break;
  }
  status = nrf24l01_SendCommand(dev, cmd, pData, length + 1);
  return status;
}

uint8_t nrf24l01_FlushTX(NRF24L01_DEV *dev){
  uint8_t status = 0;
  status = nrf24l01_SendCommand(dev, FLUSH_TX, EMPTY_DATA, 1);
  return status;
}

uint8_t nrf24l01_Transmit(NRF24L01_DEV *dev, uint8_t *pData, uint8_t length){
  uint8_t status = 0;
  // check if tx fifo is full
  // if there is no vacant slot in tx fifo, flush them
  if (nrf24l01_IsTXFull(dev)) nrf24l01_FlushTX(dev);

  // check if PRIM_RX is set high
  // if PRIM_RX is high, can't perform transmission
  // payload will be discarded
  uint8_t config = 0;
  status = nrf24l01_ReadRegister(dev, CONFIG, &config, 1);
  if (!(config ^ (1 << PRIM_RX))) return 0xff;

  // check if any error occurs
  if (ErrorCheck(status)) return status;

  // write tx payload to tx fifo
  status = nrf24l01_WritePayload(dev, pData, length);

  // check if any error occurs
  if (ErrorCheck(status)) return status;

  // perform transmission
  nrf24l01_Enable(dev);
  HAL_Delay(1);
  nrf24l01_Disable(dev);
  return nrf24l01_GetStatus(dev);
}

uint8_t nrf24l01_ReadPayload(NRF24L01_DEV *dev, uint8_t *pData, uint8_t length){
  uint8_t status;
  // check if there are any packets in rx fifo
  // if there is no packet in rx fifo, can't read out
  if (nrf24l01_IsRXEmpty(dev)) return 0xff;

  // read out data from rx fifo
  status = nrf24l01_SendCommand(dev, R_RX_PAYLOAD, pData, length + 1);
  return status;
}

uint8_t nrf24l01_FlushRX(NRF24L01_DEV *dev){
  uint8_t status = 0;
  status = nrf24l01_SendCommand(dev, FLUSH_RX, EMPTY_DATA, 1);
  return status;
}

NRF24L01_PIPE nrf24l01_GetRXPipe(NRF24L01_DEV *dev){
  uint8_t status = nrf24l01_GetStatus(dev);
  // Check error
  if (ErrorCheck(status)) return 0xff;
  uint8_t pipe = (status >> 1) & 0x7;
  switch (pipe) {
    case 0:
      return pipe0;
    case 1:
      return pipe1;
    case 2:
      return pipe2;
    case 3:
      return pipe3;
    case 4:
      return pipe4;
    case 5:
      return pipe5;
    default:
      return 0xff;
  }
}

// todo: complete this function
uint8_t nrf24l01_GetPayloadLength(NRF24L01_DEV *dev){

}

uint8_t nrf24l01_GetStatus(NRF24L01_DEV *dev){
  uint8_t status = 0;
  nrf24l01_ReadRegister(dev,STATUS,&status,1);
  return status;
}

uint8_t nrf24l01_CheckInterrupt(NRF24L01_DEV *dev, uint8_t flag){
  uint8_t status = nrf24l01_GetStatus(dev) ;
  if (ErrorCheck(status)) return 0xff;
  uint8_t check = 0;
  switch (flag) {
    case RX_DR:
      check = (status & 0x40) ^ (1 << RX_DR);
      break;
    case TX_DS:
      check = (status & 0x20) ^ (1 << TX_DS);
      break;
    case MAX_RT:
      check = (status & 0x10) ^ (1 << MAX_RT);
      break;
    default:
      return 0xff;
  }
  return check?1:0;
}

void nrf24l01_ClearInterrupt(NRF24L01_DEV* dev, uint8_t flag){
  uint8_t status = nrf24l01_GetStatus(dev) ;
  if ( ErrorCheck(status)) return;
  uint8_t mask = 0;
  switch (flag) {
    case RX_DR:
      mask = 1 << RX_DR;
      break;
    case TX_DS:
      mask = 1 << TX_DS;
      break;
    case MAX_RT:
      mask = 1 << MAX_RT;
      break;
    default:
      return;
  }
  status |= mask;
  nrf24l01_WriteRegister(dev, STATUS, &status, 1);
}

void nrf24l01_PowerUp(NRF24L01_DEV *dev){
  uint8_t config = 0, status = 0;
  status = nrf24l01_ReadRegister(dev, CONFIG, &config, 1);
  if (ErrorCheck(status)) return;
  config |= (1 << PWR_UP);
  nrf24l01_WriteRegister(dev, CONFIG, &config, 1);
}

void nrf24l01_PowerDown(NRF24L01_DEV *dev){
  uint8_t config = 0, status = 0;
  status = nrf24l01_ReadRegister(dev, CONFIG, &config, 1);
  if (ErrorCheck(status)) return;
  config &= 0xfd;
  nrf24l01_WriteRegister(dev, CONFIG, &config, 1);
}

void nrf24l01_EnterStandby(NRF24L01_DEV *dev){
  nrf24l01_Disable(dev);
  nrf24l01_PowerUp(dev);
  HAL_Delay(2);
}

uint8_t nrf24l01_IsTXFull(NRF24L01_DEV *dev){
  uint8_t fifo_status = 0;
  uint8_t status = 0;
  status = nrf24l01_ReadRegister(dev, FIFO_STATUS, &fifo_status, 1);
  if (ErrorCheck(status)) return 0xff;
  return (fifo_status & (1 << TX_FULL))?1:0;
}

uint8_t nrf24l01_IsTXEmpty(NRF24L01_DEV *dev){
  uint8_t fifo_status = 0;
  uint8_t status = 0;
  status = nrf24l01_ReadRegister(dev, FIFO_STATUS, &fifo_status, 1);
  if (ErrorCheck(status)) return 0xff;
  return (fifo_status & (1 << TX_EMPTY))?1:0;
}

uint8_t nrf24l01_IsRXFull(NRF24L01_DEV *dev){
  uint8_t fifo_status = 0;
  uint8_t status = 0;
  status = nrf24l01_ReadRegister(dev, FIFO_STATUS, &fifo_status, 1);
  if (ErrorCheck(status)) return 0xff;
  return (fifo_status & (1 << RX_FULL))?1:0;
}

uint8_t nrf24l01_IsRXEmpty(NRF24L01_DEV *dev){
  uint8_t fifo_status = 0;
  uint8_t status = 0;
  status = nrf24l01_ReadRegister(dev, FIFO_STATUS, &fifo_status, 1);
  if (ErrorCheck(status)) return 0xff;
  return (fifo_status & (1 << RX_EMPTY))?1:0;
}

void nrf24l01_EnablePipe(NRF24L01_DEV *dev, NRF24L01_PIPE pipe){
  uint8_t en_rxaddr = 0;
  uint8_t status = 0;
  status = nrf24l01_ReadRegister(dev, EN_RXADDR, &en_rxaddr, 1);
  if (ErrorCheck(status)) return;
  switch (pipe) {
    case pipe0:
      en_rxaddr |= (1 << ERX_P0);
      break;
    case pipe1:
      en_rxaddr |= (1 << ERX_P1);
      break;
    case pipe2:
      en_rxaddr |= (1 << ERX_P2);
      break;
    case pipe3:
      en_rxaddr |= (1 << ERX_P3);
      break;
    case pipe4:
      en_rxaddr |= (1 << ERX_P4);
      break;
    case pipe5:
      en_rxaddr |= (1 << ERX_P5);
      break;
    default:
      return;
  }
  nrf24l01_WriteRegister(dev, EN_RXADDR, &en_rxaddr, 1);
}

void nrf24l01_ClearInterruptFlags(NRF24L01_DEV *dev){
  uint8_t status = nrf24l01_GetStatus(dev);
  if (ErrorCheck(status)) return;
  status |= (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT);
  nrf24l01_WriteRegister(dev, STATUS, &status, 1);
}

void nrf24l01_MakePTX(NRF24L01_DEV* dev){
  uint8_t status, config;
  status = nrf24l01_ReadRegister(dev, CONFIG, &config, 1);
  if (ErrorCheck(status)) return;
  config &= (1 << PRIM_RX) ^ 0xff;
  nrf24l01_WriteRegister(dev, CONFIG, &config, 1);
}

void nrf24l01_MakePRX(NRF24L01_DEV *dev){
  uint8_t status, config;
  status = nrf24l01_ReadRegister(dev, CONFIG, &config, 1);
  if (ErrorCheck(status)) return;
  config |= (1 << PRIM_RX);
  nrf24l01_WriteRegister(dev, CONFIG, &config, 1);
}

uint8_t nrf24l01_Receive(NRF24L01_DEV *dev){
  uint8_t status, config;

  // disable device
  nrf24l01_Disable(dev);

  // read config register
  status = nrf24l01_ReadRegister(dev, CONFIG, &config, 1);

  // check for error
  if (ErrorCheck(status)) return 0xff;

  // ensure to set PRIM_RX and PWR_UP to high
  config |= (1 << PRIM_RX) | (1 << PWR_UP);
  status = nrf24l01_WriteRegister(dev, CONFIG, &config, 1);

  // check if RX FIFO is FULL
  // if true, then flush RX FIFO
  if (nrf24l01_IsRXFull(dev)) nrf24l01_FlushRX(dev);

  // enable device
  nrf24l01_Enable(dev);
  return status;
}
