#include "reg24le1.h" //definicoes basicas de pinos
#include "nRF-SPIComands.h" //Comunicacao RF
#include "hal_delay.h"

#define LED_GREEN P06
///////////////////
//Implementation //
///////////////////

/**
* Seta os pinos do nrf como saidas e entradas de acordo com as funcoes desejadas
*/
void iniciarIO(void){
    P0DIR = 0x00;   // Tudo output
    P1DIR = 0x00;   // Tudo output
    P0CON = 0x00; P1CON = 0x00; //Reseting PxCON registers
}

void setup(){
    iniciarIO();
    rf_init(ADDR_HOST,ADDR_HOST,10,RF_DATA_RATE_2Mbps,RF_TX_POWER_0dBm);
    LED_GREEN = 1;delay_ms(500);	LED_GREEN = 0;delay_ms(500);
    LED_GREEN = 1;delay_ms(500);	LED_GREEN = 0;delay_ms(500);
}

void main(){
    setup();
    while(1){ //Loop
        ///////////////////
        //Comunicacao RF //
        ///////////////////
        if(newPayload){
            switch (rx_buf[0]) {
              case 0x42:
              tx_buf[0]= 0x00;
              TX_Mode_NOACK(1);
              RX_Mode();
              LED_GREEN = 1;
              break;
              case 0x53:
              tx_buf[0]= 0x01;
              TX_Mode_NOACK(1);
              RX_Mode();
              LED_GREEN = 0;
              break;
            }
            sta = 0;
            newPayload = 0;
        }
    } /*END INFINITE LOOP*/
} /*END MAIN FUNCTION*/
