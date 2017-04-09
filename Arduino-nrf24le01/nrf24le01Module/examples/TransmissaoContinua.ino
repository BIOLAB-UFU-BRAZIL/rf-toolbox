#include <nrf24le01Module.h>

#define POLLING_TIMEOUT 1000 //1 milliseconds of maximum wait time. tempo medio: 385

nrf24le01Module tx_nrf(2,3,4);

void setup(){
  Serial.begin(9600);
  tx_nrf.rf_init(tx_nrf.ADDR_HOST,tx_nrf.ADDR_HOST,10,RF_DATA_RATE_2Mbps,RF_TX_POWER_0dBm);
  Serial.print("Transmitter with polling iniciado...\n");
}
unsigned long timeout_init_time, timeout_actual_time;

void loop() {
  tx_nrf.tx_buf[0] = 0x42;
  tx_nrf.TX_Mode_NOACK(1);
  if(tx_nrf.newPayload){
    Serial.print(tx_nrf.rx_buf[0],HEX);
    if(tx_nrf.rx_buf[0] == 0x00){
      Serial.print(" - turn on signal received by sensor.\n");
    } else {
      Serial.print(" - Nao reconhecido.\n");
    }
    tx_nrf.sta = 0;
    tx_nrf.newPayload = 0;
  }
  wait_rf_response();
  delay(1000);

  tx_nrf.tx_buf[0] = 0x53;
  tx_nrf.TX_Mode_NOACK(1);
  wait_rf_response();
  if(tx_nrf.newPayload){
    Serial.print(tx_nrf.rx_buf[0],HEX);
    if(tx_nrf.rx_buf[0] == 0x01){
      Serial.print(" - turn off signal received by sensor.\n");
    } else {
      Serial.print(" - Nao reconhecido.\n");
    }
    tx_nrf.sta = 0;
    tx_nrf.newPayload = 0;
  }
  delay(1000);
}

void wait_rf_response(){
  timeout_init_time = micros() + POLLING_TIMEOUT;
  while(1){
    timeout_actual_time = micros();
    if(timeout_actual_time>timeout_init_time){
      Serial.println("Polling timeout!");
      break;
    }
    if(!digitalRead(tx_nrf.RFIRQ)){
      tx_nrf.RF_IRQ();
      DEBUG
      if(tx_nrf.TX_OK){
        Serial.println("Data send!");
      }
      if(tx_nrf.RX_OK){
        Serial.print("Data Received with ");
        Serial.print(timeout_actual_time-timeout_init_time+1000);
        Serial.print(" milliseconds.\n");
        Serial.println(tx_nrf.payloadWidth);
        Serial.println(tx_nrf.rx_buf[0],HEX);
      }
      break;
    }
  }
}
