#ifndef nrf24le01Module_h
#define nrf24le01Module_h

#include <nRF24L01.h>
#include <Arduino.h>
#include <SPI.h>

// Definições da rotina de interrupção
#define RX_DR               6
#define TX_DS               5
#define MAX_RT              4

#define PAYLOAD_WIDTH      32   // 30 bytes on TX payload
#define TX_ADR_WIDTH        5   // 5 bytes TX(RX) address width

//NOTE: Se der ruim trocar por constantes
/** Available data rates
 * The input argument of rf_init must be defined in this @c enum
 */
typedef enum
{
    RF_DATA_RATE_1Mbps,
    RF_DATA_RATE_2Mbps,
    RF_DATA_RATE_250kbps
} rf_data_rate_t;

/** Available tx power modes
 * The input argument of rf_init must be defined in this @c enum
 */
typedef enum
{
    RF_TX_POWER_NEGATIVE_18dBm,
    RF_TX_POWER_NEGATIVE_12dBm,
    RF_TX_POWER_NEGATIVE_6dBm,
    RF_TX_POWER_0dBm
} rf_tx_power_t;

class nrf24le01Module{
public:
  nrf24le01Module(uint8_t RFIRQ_pin, uint8_t RFCE_pin, uint8_t RFCSN_pin);
  uint8_t ADDR_HOST[TX_ADR_WIDTH] =  {0xE7,0xE7,0xE7,0xE7,0xE7};   // Define a static host adr
  uint8_t rx_buf[PAYLOAD_WIDTH];    // Define lenght of rx_buf and tx_buf
  uint8_t tx_buf[PAYLOAD_WIDTH];
  uint8_t payloadWidth = 0;
  bool newPayload = 0;    // Flag to indicate that there's a new payload sensor
  uint8_t sta;
  uint8_t TX_OK = 0;
  uint8_t RX_OK = 0;
  uint8_t RFIRQ;  //Pino do arduino conectado a IRQ
  uint8_t RFCE;  //Pino do arduino conectado a CE
  uint8_t RFCSN;  //Pino do arduino conectado a CSN
  void rf_init(uint8_t *rx_addr,uint8_t *tx_addr, uint8_t rf_channel, rf_data_rate_t rf_data_rate, rf_tx_power_t rf_pwr);
  void RX_Mode(void);
  void TX_Mode_NOACK(uint8_t payloadLength);
  void RF_IRQ(void);

private:
  unsigned long tempo, tempoAtual;
  uint8_t SPI_RW(uint8_t value);
  uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value);
  uint8_t SPI_Read_Status(void);
  uint8_t SPI_Read(uint8_t reg);
  uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
  uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
};



#endif
