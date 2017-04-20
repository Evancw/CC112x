/*-----------------------------------------------------------------------------
'                     TX DEMO
'                     -------
'
'
'  - need PinChangeInt library
'  - *.eep file must flashed to Arduino first -> use eeprom Tool
'  - put cc1100_arduino.h in your sketch folder. dont't install it as library
'    for this example.
'
'-----------------------------------------------------------------------------*/
#include "cc1100_arduino.h"
#include <PinChangeInt.h>

//---------------------------=[Global variables]=----------------------------

//--------------------------[Global Task variables]--------------------------
uint32_t prev_millis_1s_timer = 0;

const uint16_t INTERVAL_1S_TIMER = 1000;        // interval at which to blink (milliseconds)

//--------------------------[Global CC1100 variables]------------------------
uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
int8_t rssi_dbm;

volatile uint8_t cc1101_packet_available;

//--------------------------[class constructors]-----------------------------
//init CC1100 constructor
CC1100 cc1100;

//---------------------------------[SETUP]-----------------------------------
void setup()
{
  // init serial Port for debugging
  Serial.begin(38400);Serial.println();

  // init CC1101 RF-module and get My_address from EEPROM
  cc1100.begin(My_addr);                   //inits RF module with main default settings

  cc1100.silde();                          //set to ILDE first
  cc1100.set_mode(0x04);                   //set modulation array mode
  cc1100.set_ISM(0x03);                    //set frequency
  cc1100.set_channel(0x01);                //set channel
  cc1100.set_output_power_level(0);        //set PA level in dbm
  cc1100.set_myaddr(0x01);                 //set my own address

  cc1100.spi_write_register(IOCFG2, 0x06); //set module in sync mode detection mode

  cc1100.show_main_settings();             //shows setting debug messages to UART
  cc1100.show_register_settings();         //shows current CC1101 register values
  cc1100.receive();                        //set to RECEIVE mode

  // init interrrupt function for available packet
  attachPinChangeInterrupt(GDO2, rf_available_int, HIGH);

  Serial.println(F("CC1101 TX Demo"));   //welcome message
}
//---------------------------------[LOOP]-----------------------------------
void loop()
{

  // one second update timer
  if (millis() - prev_millis_1s_timer >= INTERVAL_1S_TIMER)
  {
    Rx_addr = 0x03;                                              //receiver address

    uint32_t time_stamp = millis();                              //generate time stamp

    Tx_fifo[3] = (uint8_t)(time_stamp >> 24);                    //split 32-Bit timestamp to 4 byte array
    Tx_fifo[4] = (uint8_t)(time_stamp >> 16);
    Tx_fifo[5] = (uint8_t)(time_stamp >> 8);
    Tx_fifo[6] = (uint8_t)(time_stamp);

    Pktlen = 0x07;                                               //set packet len to 0x13

    detachPinChangeInterrupt(GDO2);                              //disable pin change interrupt
    cc1100.sent_packet(My_addr, Rx_addr, Tx_fifo, Pktlen, 1);    //sents package over air. ACK is received via GPIO polling
    attachPinChangeInterrupt(GDO2, rf_available_int, HIGH);      //enable pin change interrupt

    Serial.print(F("tx_time: "));Serial.print(millis()-time_stamp);Serial.println(F("ms"));
    prev_millis_1s_timer = millis();
  }

}
//--------------------------[end loop]----------------------------

//-----interrrupt function not needed for this demo
//---------------------[ check incomming RF packet ]-----------------------
void rf_available_int(void)
{
  detachPinChangeInterrupt(GDO2);

  if(cc1100.packet_available() == TRUE){
    cc1100.get_payload(Rx_fifo, pktlen, rx_addr, sender, rssi_dbm, lqi); //stores the payload data to Rx_fifo
    cc1101_packet_available = TRUE;                                      //set flag that an package is in RX buffer
  }

  attachPinChangeInterrupt(GDO2, rf_available_int, HIGH);
}
