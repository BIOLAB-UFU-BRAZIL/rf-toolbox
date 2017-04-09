#include "nrf24le01Module.h"


nrf24le01Module::nrf24le01Module(uint8_t RFIRQ_pin, uint8_t RFCE_pin, uint8_t RFCSN_pin){
  RFIRQ = RFIRQ_pin;
  RFCE = RFCE_pin;
  RFCSN = RFCSN_pin;
}

///////////
//PUBLIC //
///////////


/**
 * Inicia o estado recepcao, onde o nrf estara aguardando a interrupcao rf
 */
void nrf24le01Module::RX_Mode(void){
  newPayload = 0;
  sta = 0;
  RX_OK = 0;
  digitalWrite(RFCE,0); // Radio chip enable low -> Standby-1
  SPI_RW_Reg(W_REGISTER + CONFIG, 0x1F);  // Set PWR_UP bit, enable CRC(2 bytes) & Prim:RX. RX_DR enabled..
  digitalWrite(RFCE,1); // Set CE pin high to enable RX Mode
}

/**
 * Entra no estado de transmissao, com pacote sem ACK.
 * Transmitindo o atual valor em tx_buff
 * RF transceiver is never in TX mode longer than 4 ms.
 * @param payloadLength tamanho do pacote escrito em tx buff, maximo 32
 */
void nrf24le01Module::TX_Mode_NOACK(uint8_t payloadLength){
  digitalWrite(RFCE,0); // Radio chip enable low -> Standby-1
  SPI_RW_Reg(W_REGISTER + CONFIG, 0x1E);  // Set PWR_UP bit, enable CRC(2 bytes) & Prim:TX. RX_DR enabled.
  SPI_Write_Buf(W_TX_PAYLOAD_NOACK, tx_buf, payloadLength); // Writes data to TX payload
  // Endereço da porta P2, matrizes tx_buf (), o comprimento da matriz é enviada)
  TX_OK = 0;
  digitalWrite(RFCE,1); // Set CE pin high to enable TX Mode
  delayMicroseconds(12);
  digitalWrite(RFCE,0); // Radio chip enable low -> Standby-1
  // RX mode?
  tempo = micros() + 100000;
  while (1){
    tempoAtual = micros();
    if(tempoAtual>tempo){
      Serial.println("TX_MODE Polling Time Out!\n\0");
      break;
    }
    if(!digitalRead(RFIRQ)){
      RF_IRQ();
      //Debug
    //   if(TX_OK){
    //     Serial.println("Data send!");
    //   }
    //   if(RX_OK){
    //     Serial.println("Data Received.");
    //     Serial.println(payloadWidth);
    //     Serial.println(rx_buf[0],HEX);
    //   }
      break;
    }
  }
  RX_Mode();
}

/**
 * Inicia a comunicacao RF, apos configura-la ativa todas as interrupcoes e aguarda em RX Mode
 * @param rx_addr      Endereço RX de 5 bytes
 * @param tx_addr      Endereço RX de 5 bytes
 * @param rf_channel   Valor em MHz a ser somado a 2.4Hz como canal, entre 0 e 125.
 * @param rf_data_rate Velocidade de transmissao nor ar que deseja utilizar
 * @param rf_pwr       Power of the Transmission
 */
void nrf24le01Module::rf_init(uint8_t *rx_addr,uint8_t *tx_addr,
   uint8_t rf_channel, rf_data_rate_t rf_data_rate,
   rf_tx_power_t rf_pwr){
 uint8_t rf_setup_byte = 0x07; //0000 0111
  // Radio + SPI setup
  pinMode(RFIRQ, INPUT);  // Define RFIRQ as input to receive IRQ from nRF24L01+
  pinMode(RFCE, OUTPUT);  // Define RFCE as output to control nRF24L1+ Chip Enable
  pinMode(RFCSN, OUTPUT); // Define RFCSN as output to control nRF24L1+ SPI
  SPI.begin();            // start the SPI library:
  newPayload = 0;
  sta = 0;
  TX_OK = 0;
  RX_OK = 0;
  digitalWrite(RFCSN,1);                        // Set CSN low, init SPI tranaction
  digitalWrite(RFCE,0);                         // Radio chip enable low
  switch (rf_pwr) {
      case RF_TX_POWER_NEGATIVE_18dBm:
      rf_setup_byte &= 0xF9; //1111 1001
      break;
      case RF_TX_POWER_NEGATIVE_12dBm:
      rf_setup_byte &= 0xFB;//1111 1011
      rf_setup_byte |= 0x02;//0000 0010
      break;
      case RF_TX_POWER_NEGATIVE_6dBm:
      rf_setup_byte &= 0xFD;//1111 1101
      rf_setup_byte |= 0x04;//0000 0100
      break;
      case RF_TX_POWER_0dBm:
      rf_setup_byte |= 0x04;//0000 0110
      break;
    }
    switch (rf_data_rate) {
      case RF_DATA_RATE_250kbps:
      rf_setup_byte &= 0xF7;//1111 0111
      rf_setup_byte |= 0x20;//0010 0000
      break;
      case RF_DATA_RATE_1Mbps:
      rf_setup_byte &= 0xD7;//1101 0111
      break;
      case RF_DATA_RATE_2Mbps:
      rf_setup_byte &= 0xDF;//1101 1111
      rf_setup_byte |= 0x08;//0000 1000
      break;
    }

  //TODO: Add library nrf24le01
  //Transmit Address.
  SPI_Write_Buf(W_REGISTER + TX_ADDR, tx_addr, TX_ADR_WIDTH);
  //Receive Address
  SPI_Write_Buf(W_REGISTER + RX_ADDR_P0, rx_addr, TX_ADR_WIDTH);
  // Disable Auto.Ack
  SPI_RW_Reg(W_REGISTER + EN_AA, 0x00);        // Disable Auto.Ack:Pipe0
  // Enable Pipe0 (only pipe0)
  SPI_RW_Reg(W_REGISTER + EN_RXADDR, 0x01);    // Enable Pipe0 (only pipe0)

  SPI_RW_Reg(W_REGISTER + AW, 0x03);           // 5 bytes de endereço
  // Time to automatic retransmition selected: 250us, retransmition disabled
  SPI_RW_Reg(W_REGISTER + SETUP_RETR, 0x00);   // Tempo de retransmissão automática de 250us, retransmissão desabilitada
  // Select RF channel 40
  SPI_RW_Reg(W_REGISTER + RF_CH, rf_channel);          // Select RF channel 90. Fo = 2,490 GHz
  // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
  SPI_RW_Reg(W_REGISTER + RF_SETUP, rf_setup_byte);     // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
  // Ativa Payload din?mico em data pipe 0
  SPI_RW_Reg(W_REGISTER + DYNPD, 0x01);        // Ativa Payload dinâmico em data pipe 0
  // Ativa Payload din?mico, com ACK e comando W_TX_PAY
  SPI_RW_Reg(W_REGISTER + FEATURE, 0x07);      // Ativa Payload dinâmico, com ACK e comando W_TX_PAY
  //TODO: Stay in RX Mode waiting for command
  SPI_RW_Reg(FLUSH_TX,0);
  SPI_RW_Reg(FLUSH_RX,0);
  SPI_RW_Reg(W_REGISTER+NRF_STATUS,0x70);
  RX_Mode();
}

////////////
//Private //
////////////

/**
 * [SPI_RW description]
 * @param  value [description]
 * @return       [description]
 */
uint8_t nrf24le01Module::SPI_RW(uint8_t value){
  uint8_t SPIData;
  SPIData = SPI.transfer(value);
  return SPIData;                   // return SPI read value
}

/**
 * [SPI_RW_Reg description]
 * @param  reg   [description]
 * @param  value [description]
 * @return       [description]
 */
uint8_t nrf24le01Module::SPI_RW_Reg(uint8_t reg, uint8_t value){
  uint8_t status;
  digitalWrite(RFCSN,0);                      // CSN low, initiate SPI transaction£
  status = SPI_RW(reg);           // select register
  SPI_RW(value);                  // ..and write value to it..
  digitalWrite(RFCSN,1);                      // CSN high again  £¨rfcon^1
  return(status);                 // return nRF24L01 status byte
}

/**
 * [nrf24le01Module::SPI_Read_Status description]
 * @return  [description]
 */
uint8_t nrf24le01Module::SPI_Read_Status(void){
  uint8_t reg_val;

  digitalWrite(RFCSN,0);                          // CSN low, initialize SPI communication...
  reg_val = SPI_RW(NOP);                            // ..then read register value
  digitalWrite(RFCSN,1);                          // CSN high, terminate SPI communication RF
  return(reg_val);                                // return register value
}

/**
 * [nrf24le01Module::SPI_Read description]
 * @param  reg [description]
 * @return     [description]
 */
uint8_t nrf24le01Module::SPI_Read(uint8_t reg){
  uint8_t reg_val;

  digitalWrite(RFCSN,0);                          // CSN low, initialize SPI communication...
  SPI_RW(reg);                                    // Select register to read from..
  reg_val = SPI_RW(0);                            // ..then read register value
  digitalWrite(RFCSN,1);                          // CSN high, terminate SPI communication RF
  return(reg_val);                                // return register value
}

/**
 * [nrf24le01Module::SPI_Read_Buf description]
 * @param  reg   [description]
 * @param  pBuf  [description]
 * @param  bytes [description]
 * @return       [description]
 */
uint8_t nrf24le01Module::SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes){
  uint8_t status,byte_ctr;

  digitalWrite(RFCSN,0);                                    // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);                         // Select register to write to and read status byte

  for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
  pBuf[byte_ctr] = SPI_RW(0);                 // Perform SPI_RW to read byte from nRF24L01

  digitalWrite(RFCSN,1);                                    // Set CSN high again

  return(status);                               // return nRF24L01 status byte
}

/**
 * [nrf24le01Module::SPI_Write_Buf description]
 * @param  reg   [description]
 * @param  pBuf  [description]
 * @param  bytes [description]
 * @return       [description]
 */
uint8_t nrf24le01Module::SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes){
  uint8_t status,byte_ctr;

  digitalWrite(RFCSN,0);                        // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);                         // Select register to write to and read status byte
  for(byte_ctr=0; byte_ctr<bytes; byte_ctr++)   // then write all byte in buffer(*pBuf)
  SPI_RW(*pBuf++);

  digitalWrite(RFCSN,1);                        // Set CSN high again
  return(status);                               // return nRF24L01 status byte
}


//////////////
//Interrupt //
//////////////

/**
 * Interrupt handle para evento de recepcao de payload
 * ativa o sinalizador newPayload
 * o tamanho da payload é armazenado em payloadWidth
 */
void nrf24le01Module::RF_IRQ(void) {
  sta=SPI_Read(NRF_STATUS);
  if(bitRead(sta,RX_DR)){ // if receive data ready (RX_DR) interrupt
    RX_OK = 1;
    newPayload = 1;
    SPI_Read_Buf(R_RX_PAYLOAD,rx_buf,PAYLOAD_WIDTH);  // read receive payload from RX_FIFO buffer
    payloadWidth = SPI_Read(R_RX_PLD_WIDTH);  // Retorna o número de bytes no payload recebido
    if(payloadWidth > 32) {
      payloadWidth = 0;
      newPayload = 0;
    }
    SPI_RW_Reg(FLUSH_RX,0);
  }
  if(bitRead(sta,TX_DS)){
    TX_OK = 1;
    SPI_RW_Reg(FLUSH_TX,0);
  }
  SPI_RW_Reg(W_REGISTER+NRF_STATUS,0x70);
}
