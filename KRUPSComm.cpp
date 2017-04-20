/*------------------------------------------------------------------------------
'                     CC11XX Arduino Library
'-----------------------------------------------------------------------------*/
#include "KRUPSComm.h"
//#include "settings.h"

//-------------------------[CC112x reset function]------------------------------
void KRUPSComm::reset(void)			      // reset defined in cc112x datasheet
{
  digitalWrite(SS_PIN, LOW);
	delayMicroseconds(10);
	digitalWrite(SS_PIN, HIGH);
	delayMicroseconds(40);

	commandStrobe(SRES);
	delay(1);
}

//------------------------[set Power Down]--------------------------------------
void KRUPSComm::powerdown(void)
{
	setIDLE();
	commandStrobe(SPWD);           					// CC11XX Power Down
}

//---------------------------[WakeUp]-------------------------------------------
void KRUPSComm::wakeup(void)
{
	digitalWrite(SS_PIN, LOW);
	delayMicroseconds(10);
	digitalWrite(SS_PIN, HIGH);
	delayMicroseconds(10);
	receive();															// go to RX Mode
}

//-------------------[CC11XX functions "8.9ms"]----------------------------
bool KRUPSComm::begin(void)
{
    #ifdef CC1100_DEBUG
		Serial.println("Init CC112x...");
	#endif

	SPI.begin();
	reset();								//CC112x reset

	commandStrobe(SFTX); delayMicroseconds(100); //flush the TX_fifo content
	commandStrobe(SFRX); delayMicroseconds(100); //flush the RX_fifo content

	uint8_t partnum = spiRead(PARTNUM, EXTD_REGISTER); uint8_t version = spiRead(VERSION, EXTD_REGISTER);

	//checks if valid Chip ID is found. Usually 0x03 or 0x14. if not -> abort
	if(version == 0x00 || version == 0xFF) {
			#if CC1100_DEBUG == 1
				Serial.println("no CC11xx found!");
				end();												  //CC112x Powerdown and disable SPI bus
			#endif

			return false;
	}

	#if CC1100_DEBUG == 1
		Serial.print("Partnumber:");
		Serial.println(partnum);

		Serial.print("Version:");
		Serial.println(version);
	#endif

	// Write register configuration values ( there may be a better way )
	// spiWrite(IOCFG3, SET_IOCFG3);
	// spiWrite(IOCFG2, SET_IOCFG2);
	// spiWrite(IOCFG1, SET_IOCFG1);
	// spiWrite(IOCFG0, SET_IOCFG0);
	// spiWrite(SYNC_CFG1, SET_SYNC_CFG1);
	// spiWrite(DEVIATION_M, SET_DEVIATION_M);
	// spiWrite(MODCFG_DEV_E, SET_MODCFG_DEV_E);
	// spiWrite(DCFILT_CFG, SET_DCFILT_CFG);
	// spiWrite(FREQ_IF_CFG, SET_FREQ_IF_CFG);
	// spiWrite(IQIC, SET_IQIC);
	// spiWrite(CHAN_BW, SET_CHAN_BW);
	// spiWrite(MDMCFG0, SET_MDMCFG0);
	// spiWrite(SYMBOL_RATE2, SET_DRATE2);
	// spiWrite(SYMBOL_RATE1, SET_DRATE1);
	// spiWrite(SYMBOL_RATE0, SET_DRATE0);
	// spiWrite(AGC_REF, SET_AGC_REF);
	// spiWrite(AGC_CS_THR, SET_AGC_CS_THR);
	// spiWrite(AGC_CFG1, SET_AGC_CFG1);
	// spiWrite(AGC_CFG0, SET_AGC_CFG0);
	// spiWrite(FIFO_CFG, SET_FIFO_CFG);
	// spiWrite(SETTLING_CFG, SET_SETTLING_CFG);
	// spiWrite(FS_CFG, SET_FS_CFG);
	// spiWrite(PKT_CFG0, SET_PKT_CFG0);
	// spiWrite(PKT_LEN, SET_PKT_LEN);
	// spiWrite(IF_MIX_CFG, SET_IF_MIX_CFG, EXTD_REGISTER);
	// spiWrite(FREQOFF_CFG, SET_FREQOFF_CFG, EXTD_REGISTER);
	// spiWrite(FREQ2, SET_FREQ2, EXTD_REGISTER);
	// spiWrite(FREQ1, SET_FREQ1, EXTD_REGISTER);
	// spiWrite(FREQ0, SET_FREQ0, EXTD_REGISTER);
	// spiWrite(IF_ADC0, SET_IF_ADC0, EXTD_REGISTER);
	// spiWrite(FS_DIG1, SET_FS_DIG1, EXTD_REGISTER);
	// spiWrite(FS_DIG0, SET_FS_DIG0, EXTD_REGISTER);
	// spiWrite(FS_CAL0, SET_FS_CAL0, EXTD_REGISTER);
	// spiWrite(FS_DIVTWO, SET_FS_DIVTWO, EXTD_REGISTER);
	// spiWrite(FS_DSM0, SET_FS_DSM0, EXTD_REGISTER);
	// spiWrite(FS_DVC0, SET_FS_DVC0, EXTD_REGISTER);
	// spiWrite(FS_PFD, SET_FS_PFD, EXTD_REGISTER);
	// spiWrite(FS_PRE, SET_FS_PRE, EXTD_REGISTER);
	// spiWrite(FS_REG_DIV_CML, SET_FS_REG_DIV_CML, EXTD_REGISTER);
	// spiWrite(FS_SPARE, SET_FS_SPARE, EXTD_REGISTER);
	// spiWrite(XOSC5, SET_XOSC5, EXTD_REGISTER);
	// spiWrite(XOSC3, SET_XOSC3, EXTD_REGISTER);
	// spiWrite(XOSC1, SET_XOSC1, EXTD_REGISTER);

	#if CC1100_DEBUG == 1
		Serial.println("...done");
	#endif

	receive();								//set CC11XX in receive mode
	return true;
}

//-----------------[finish's the CC11XX operation]------------------------------
void KRUPSComm::end(void)
{
	powerdown();																	//power down CC112x
	SPI.end();																		//disable SPI Interface
}

//-----------------------[show all CC112x registers]----------------------------
// void KRUPSComm::show_register_settings(void)
// {
// 	#if CC1100_DEBUG == 1
// 		uint8_t config_reg_verify[CFG_REGISTER],Patable_verify[CFG_REGISTER];
//
// 		spi_read_burst(READ_BURST,config_reg_verify,CFG_REGISTER);	//reads all 47 config register
//
// 		Serial.println("Cfg_reg:");
//
// 		for(uint8_t i = 0 ; i < CFG_REGISTER; i++) {		//showes rx_buffer for debug
// 				Serial.print(config_reg_verify[i]); Serial.print(" ");
// 				if(i==9 || i==19 || i==29 || i==39)			//just for beautiful output style
// 						Serial.println();
// 			}
// 			Serial.println();
// 			Serial.println("PaTable:");
//
// 			for(uint8_t i = 0 ; i < 8; i++) { 						//showes rx_buffer for debug
// 					Serial.print(Patable_verify[i]); Serial.print(" ");
// 			}
// 		Serial.println();
// 	#endif
// }

//----------------------------[idle mode]---------------------------------------
void KRUPSComm::setIDLE(void)
{
	uint8_t marcstate = 0xFF;								//set unknown/dummy state value
	commandStrobe(SIDLE);								  	//sets to idle first. must be in

	while(marcstate != 0x01) {						    	//0x01 = SILDE
		marcstate = (spiRead(MARCSTATE, EXTD_REGISTER) & 0x1F);			//read out state of cc112x to be sure in RX
	}
	//Serial.println();
}

//---------------------------[transmit mode]------------------------------------
void KRUPSComm::transmit(void)
{
	uint8_t marcstate = 0xFF;										//set unknown/dummy state value
	commandStrobe(STX);									//sends the data over air

	while(marcstate != 0x01) {						       			//0x01 = ILDE after sending data
		marcstate = (spiRead(MARCSTATE, EXTD_REGISTER) & 0x1F);		//read out state of cc112x to be sure in IDLE and TX is finished
		delayMicroseconds(100);                         			//must be in for ever reason
	}

	commandStrobe(SFTX); delayMicroseconds(100);			//flush the Tx_fifo content -> a must for interrupt handling
}

//---------------------------[receive mode]-------------------------------------
void KRUPSComm::receive(void)
{
	uint8_t marcstate = 0xFF;							//set unknown/dummy state value
	setIDLE();			                            	//sets to idle first.
	commandStrobe(SRX);                     			//writes receive strobe (receive mode)

	while(marcstate != 0x0D) {                 		 	//0x0D = RX
		marcstate = (spiRead(MARCSTATE, EXTD_REGISTER) & 0x1F);		//read out state of cc112x to be sure in RX
	}
}

//-------------------------[tx_payload_burst]-----------------------------------
void KRUPSComm::tx_payload_burst(uint8_t *txbuffer, uint8_t length)
{
	txbuffer[0] = length-1;

	spi_write_burst(STANDARD_FIFO, txbuffer, length);	//writes TX_Buffer +1 because of pktlen must be also transfered

	#if CC1100_DEBUG == 1
		Serial.print(F("TX_FIFO:"));
		for(uint8_t i = 0 ; i < length; i++) {			   //TX_fifo debug out
			Serial.print(txbuffer[i]);
		}
		Serial.println();
	#endif
}

//------------------[rx_payload_burst - package received]-----------------------
void KRUPSComm::rx_payload_burst(uint8_t rxbuffer[], uint8_t &pktlen) // TODO: maybe done
{
	uint8_t bytes_in_RXFIFO = spiRead(NUM_RXBYTES, EXTD_REGISTER); //reads the number of bytes in RXFIFO

	if (bytes_in_RXFIFO) {
		pktlen = spiRead(STANDARD_FIFO | READ_SINGLE_BYTE);		//received pktlen +1 for complete TX buffer
		rxbuffer[0] = pktlen;
		for(uint8_t i = 1; i < pktlen + 1; i++) {			        // prepended packet length
			rxbuffer[i] = spiRead(STANDARD_FIFO | READ_SINGLE_BYTE);		//TODO
		}
	} else {
		//Serial.print(F("bad RX buffer!"));
	}
	setIDLE();
	commandStrobe(SFRX); delayMicroseconds(100);
	receive();
}

//---------------------------[sent packet]--------------------------------------
uint8_t KRUPSComm::sent_packet(uint8_t *txbuffer, uint8_t pktlen, uint8_t tx_retries)
{
//	uint8_t pktlen_ack;													//default package len for ACK
	// uint8_t rxbuffer[FIFOBUFFER];
	uint8_t tx_retries_count = 0;
//	uint16_t ackWaitCounter = 0;

	do {										                        //sent package out with retries
		tx_payload_burst(txbuffer, pktlen);			                    //loads the data in cc112x buffer
		transmit();												        //sents data over air
		receive();												        //receive mode

// 		if(rx_addr == BROADCAST_ADDRESS || tx_retries == 0) {  			//no wait acknowledge if sent to broadcast address or tx_retries = 0
// 			return true;												//successful sent to BROADCAST_ADDRESS
// 		}

// 		while (ackWaitCounter < ACK_TIMEOUT ) {						  	//Wait for an acknowge
// 			if (packet_available() == true)	{				      		//if RF package received check package acknowge
// 				uint8_t from_sender = rx_addr;                  		//the original message sender address
// 				rx_fifo_erase(rxbuffer);
// 				rx_payload_burst(rxbuffer, pktlen_ack);					//reads package in buffer
// 				check_acknowledge(rxbuffer, pktlen_ack, from_sender, my_addr);
// 				return true;											//package successfully sent
// 			} else {
// 				ackWaitCounter++;                   					//increment ACK wait counter
// 				delay(1);												//delay to give receiver time
// 			}
// 		}

// 		ackWaitCounter = 0;												//resets the ACK_Timeout
 		tx_retries_count++;												//increase tx retry counter

		#if CC1100_DEBUG == 1											//debug output messages
			Serial.print(F(" #:"));
			Serial.println(tx_retries_count-1);
		#endif
	}while(tx_retries_count <= tx_retries);                				//while count of retries is reaches

	return false;														//sent failed. too many retries
}

//--------------------------[sent ACKNOWLEDGE]------------------------------------
void KRUPSComm::sent_acknowledge(uint8_t my_addr, uint8_t tx_addr)
{
	uint8_t pktlen = 0x06;												//complete Pktlen for ACK packet
	uint8_t tx_buffer[0x06];											//tx buffer array init

	tx_buffer[3] = 'A';	tx_buffer[4] = 'c';	tx_buffer[5] = 'k';			//fill buffer with ACK Payload

	tx_payload_burst(tx_buffer, pktlen);				//load payload to CC1100
	transmit();															//sent package over the air
	receive();															//set CC112x in receive mode

	#if CC1100_DEBUG == 1												//debut output
		//Serial.println();
		Serial.println("Ack_sent!");
	#endif
}


//------------------[check Payload for ACK or Data]-----------------------------
uint8_t KRUPSComm::get_payload(uint8_t rxbuffer[], uint8_t &pktlen, int8_t &rssi_dbm, uint8_t &lqi)
{
	rx_fifo_erase(rxbuffer);										//delete rx_fifo bufffer
	rx_payload_burst(rxbuffer, pktlen);							    //read package in buffer

	if(pktlen == 0x00) {											//packet len not plausible?
	    #if CC1100_DEBUG == 1
			Serial.println("bad packet!");
		#endif
		return false;
	}

	if(check_acknowledge(rxbuffer, pktlen) == true) { 	//acknowledge received?
			return false;												//Ack received -> finished
	} else { 														//real data, and sent acknowledge
		rssi_dbm = rssi_convert();				//converts receiver strength to dBm
		lqi = lqi_convert(rxbuffer[pktlen + 2]);
		uint8_t crc = check_crc(lqi);								//get rf quality indicator

		#if CC1100_DEBUG == 1										//debug output messages
			if(rxbuffer[1] == BROADCAST_ADDRESS) {					//if my receiver address is BROADCAST_ADDRESS
				Serial.println("BROADCAST message");
			}
			Serial.print("RX_FIFO:");
			for(uint8_t i = 0 ; i < pktlen + 3; i++) {		    	//shows rx_buffer for debug
				Serial.print(rxbuffer[i]);
			}
			Serial.println();

			Serial.print("RSSI:"); Serial.print(rssi_dbm);Serial.print(" ");
			Serial.print("LQI:"); Serial.print(lqi);Serial.print(" ");
			Serial.print("CRC:"); Serial.println(crc);
			#endif

		// if(my_addr != BROADCAST_ADDRESS) {						//send only ack if no BROADCAST_ADDRESS
		// 	sent_acknowledge(my_addr, sender);					//sending acknowledge to sender!
		// }
		return true;
	}
}

//-------------------------[check ACKNOWLEDGE]----------------------------------
uint8_t KRUPSComm::check_acknowledge(uint8_t *rxbuffer, uint8_t pktlen)
{
	if(rxbuffer[3] == 'A' && rxbuffer[4] == 'c' && rxbuffer[5] == 'k') 		 //acknowledge received! TODO: check what should be in which bytes
	{
		int8_t rssi_dbm = rssi_convert();
		uint8_t lqi = lqi_convert(rxbuffer[pktlen + 2]);
		uint8_t crc = check_crc(lqi);
		#if CC1100_DEBUG == 1
			Serial.print("ACK! ");
			Serial.print("RSSI: "); Serial.print(rssi_dbm); Serial.print(" ");
			Serial.print("LQI: "); Serial.print(lqi); Serial.print(" ");
			Serial.print("CRC: "); Serial.println(crc);
		#endif
		return true;
	}
	return false;
}

//------------[check if Packet is received within defined time in ms]-----------
bool KRUPSComm::wait_for_packet(uint8_t milliseconds)
{
	for(uint8_t i = 0; i < milliseconds; i++) {
			delay(1);													//delay till system has data available
			if (packet_available()) {
				return true;
			} else if(i == milliseconds) {
				Serial.println(F("no packet received!"));
				return false;
			}
		}
}

//------------------------[set CC112x address]----------------------------------
// void KRUPSComm::set_myaddr(uint8_t addr)
// {
// 	spiWrite(ADDR, addr);														//stores MyAddr in the CC112x
// }

//---------------------------[set channel]--------------------------------------
// void KRUPSComm::set_channel(uint8_t channel)
// {
// 	spiWrite(CHANNR, channel);													//stores the new channel # in the CC112x
// }

//-----------------------[set modulation mode]----------------------------------
// void KRUPSComm::set_mode(uint8_t mode)
// {
// 	uint8_t Cfg_reg[CFG_REGISTER];
// 	switch (mode)
// 	{
// 		case 0x01:
// 					//eeprom_read_block(Cfg_reg,cc1100_GFSK_1_2_kb,CFG_REGISTER);			//sets up settings for GFSK 1,2 kbit mode/speed
// 					break;
// 		case 0x02:
// 					//eeprom_read_block(Cfg_reg,cc1100_GFSK_38_4_kb,CFG_REGISTER);			//sets up settings for GFSK 38,4 kbit mode/speed
// 					break;
// 		case 0x03:
// 					//eeprom_read_block(Cfg_reg,cc1100_GFSK_100_kb,CFG_REGISTER);			//sets up settings for GFSK 100 kbit mode/speed
// 					break;
// 		case 0x04:
// 					//eeprom_read_block(Cfg_reg,cc1100_MSK_250_kb,CFG_REGISTER);			//sets up settings for GFSK 38,4 kbit mode/speed
// 					break;
// 		case 0x05:
// 					//eeprom_read_block(Cfg_reg,cc1100_MSK_500_kb,CFG_REGISTER);			//sets up settings for GFSK 38,4 kbit mode/speed
// 					break;
// 		case 0x06:
// 					//eeprom_read_block(Cfg_reg,cc1100_OOK_4_8_kb,CFG_REGISTER);			//sets up settings for GFSK 38,4 kbit mode/speed
// 					break;
// 		default:
// 					//eeprom_read_block(Cfg_reg,cc1100_GFSK_38_4_kb,CFG_REGISTER);			//sets up settings for GFSK 38,4 kbit mode/speed
// 					mode = 0x02;
// 					break;
// 	}

// 	spi_write_burst(WRITE_BURST,Cfg_reg,CFG_REGISTER);										//writes all 47 config register
// }

//---------------------------[set ISM Band]-------------------------------------
// void KRUPSComm::set_ISM(uint8_t ism_freq)
// {
// 	uint8_t freq2, freq1, freq0;
//   	uint8_t Patable[8];

// 	switch (ism_freq) {												//loads the RF freq which is defined in cc1100_freq_select
// 		case 0x01:															//315MHz
// 					freq2=0x0C;
// 					freq1=0x1D;
// 					freq0=0x89;
// 					break;
// 		case 0x02:															//433.92MHz
// 					freq2=0x10;
// 					freq1=0xB0;
// 					freq0=0x71;
// 					break;
// 		case 0x03:															//868.3MHz
// 					freq2=0x21;
// 					freq1=0x65;
// 					freq0=0x6A;
// 					break;
// 		case 0x04:															//915MHz
// 					freq2=0x23;
// 					freq1=0x31;
// 					freq0=0x3B;
// 					break;
// 		/*
// 		case 0x05:															//2430MHz
// 					freq2=0x5D;
// 					freq1=0x76;
// 					freq0=0x27;
// 					break;
// 		*/
// 		default:															//default is 868.3MHz
// 					freq2=0x21;
// 					freq1=0x65;
// 					freq0=0x6A;
// 					ism_freq = 0x03;
// 					break;
// 	}

// 	spiWrite(FREQ2,freq2);													//stores the new freq setting for defined ISM band
// 	spiWrite(FREQ1,freq1);
// 	spiWrite(FREQ0,freq0);
// }

/*
void KRUPSComm::set_freq(uint32_t freq)
{
    uint32_t num;

    num = freq * (0x10000 / 1000000.0) / 26; //26MHz oscillator
    FREQ2 = num >> 16;
    FREQ1 = (num>>8) & 0xff;
    FREQ0 = num & 0xff;
}*/

//---------------------------[set PATABLE]--------------------------------------
// void KRUPSComm::set_patable(uint8_t *patable_arr)
// {
// 	spi_write_burst(PATABLE_BURST, patable_ arr, 8);				//writes output power settings to cc112x	"104us"
// }

//-------------------------[set output power]-----------------------------------
// void KRUPSComm::set_output_power_level(int8_t dBm)
// {
// 	uint8_t pa = 0xC0;

// 	if      (dBm <= -30) pa = 0x00;
// 	else if (dBm <= -20) pa = 0x01;
// 	else if (dBm <= -15) pa = 0x02;
// 	else if (dBm <= -10) pa = 0x03;
// 	else if (dBm <= 0)   pa = 0x04;
// 	else if (dBm <= 5)   pa = 0x05;
// 	else if (dBm <= 7)   pa = 0x06;
// 	else if (dBm <= 10)  pa = 0x07;

// 	spiWrite(FREND0, pa);
// }

//---------------[set data_whitening ON=true; OFF=false]-------------------------
void KRUPSComm::set_data_whitening(bool whitening)
{
  	uint8_t data = spiRead(PKT_CFG1);
  	if ( whitening )
		data = data | 0x40;
	else
		data = data & 0xBF;
  	spiWrite(PKT_CFG1, data);
}

//------------[set manchester encoding ON=true; OFF=false]----------------------
void KRUPSComm::set_manchester_encoding(bool manchester)
{
	uint8_t data = spiRead(MDMCFG1);

	if ( manchester )
		data = data | 0x20;
	else
		data = data & 0xDF;
	spiWrite(MDMCFG1, data);
}

//--------------------------[rssi_convert]--------------------------------------
float KRUPSComm::rssi_convert( void )
{
	int16_t rssi = spiRead(RSSI1, EXTD_REGISTER);
	int8_t RSSI0_val = spiRead(RSSI0, EXTD_REGISTER);

	if (0x01 & RSSI0_val)
		return ((rssi << 8) | (RSSI0_val & 0x70)) * .0625;
	else
		return 0;
}


//----------------------------[get temp]----------------------------------------
// uint8_t KRUPSComm::get_temp(uint8_t *ptemp_Arr)
// {
// 	const uint8_t num_samples = 8;
// 	uint16_t adc_result = 0;
// 	uint32_t temperature = 0;

// 	setIDLE();									            //sets CC112x into IDLE
// 	spiWrite(PTEST, 0xBF);									//enable temp sensor
// 	delay(50);												//wait a bit

// 	for(uint8_t i=0;i<num_samples;i++) {   //sampling analog temperature value
//     	adc_result += analogRead(GDO0);
// 		delay(1);
//   	}
// 	adc_result = adc_result / num_samples;
// 	//Serial.println(adc_result);

// 	temperature = (adc_result * CC1100_TEMP_ADC_MV) / CC1100_TEMP_CELS_CO;

// 	ptemp_Arr[0] = temperature / 10;      //cut last digit
// 	ptemp_Arr[1] = temperature % 10;      //isolate last digit

// 	#if CC1100_DEBUG == 1
// 		Serial.print(F("Temp:"));Serial.print(ptemp_Arr[0]);Serial.print(F("."));Serial.println(ptemp_Arr[1]);
// 	#endif

// 	spiWrite(PTEST, 0x7F);				//writes 0x7F back to PTest (app. note)

// 	receive();
// 	return (*ptemp_Arr);
// }


//|============================= SPI Transmission =============================|
void KRUPSComm::spiWrite(uint8_t reg, uint8_t data, uint8_t prefix)
{
	digitalWrite(SS_PIN, LOW);
	while(digitalRead(MOSI_PIN) ==0);
	SPI.beginTransaction(s);

	if ( prefix ) {
		SPI.transfer(prefix | WRITE_SINGLE_BYTE);
		SPI.transfer(reg);
	} else
		SPI.transfer(reg | WRITE_SINGLE_BYTE);

	SPI.transfer(data);
	SPI.endTransaction();

	digitalWrite(SS_PIN, HIGH);
}

uint8_t KRUPSComm::spiRead(uint8_t reg, uint8_t prefix)
{
	digitalWrite(SS_PIN, LOW);
	while(digitalRead(MOSI_PIN) == 0);
	SPI.beginTransaction(s);

	if ( prefix ) {
		SPI.transfer(prefix | READ_SINGLE_BYTE);
		SPI.transfer(reg);
	} else
		SPI.transfer(reg | READ_SINGLE_BYTE);

	uint8_t spi_instr = SPI.transfer(0xFF);
	SPI.endTransaction();

	digitalWrite(SS_PIN, HIGH);
	return spi_instr;
}

void KRUPSComm::spi_read_burst(uint8_t startReg, uint8_t *pArr, uint8_t length, uint8_t prefix)
{
	digitalWrite(SS_PIN, LOW);
	while(digitalRead(MOSI_PIN) == 0);
	SPI.beginTransaction(s);

	if ( prefix ) {
		SPI.transfer(prefix | READ_BURST);
		SPI.transfer(startReg);
	} else
		SPI.transfer(startReg | READ_BURST);

	for(uint8_t i=0; i<length ;i++) {
		pArr[i] = spiRead(0xFF);
	}
	SPI.endTransaction();
	digitalWrite(SS_PIN, HIGH);
}

void KRUPSComm::spi_write_burst(uint8_t startReg, uint8_t *pArr, uint8_t length, uint8_t prefix)
{
	digitalWrite(SS_PIN, LOW);
	while(digitalRead(MOSI_PIN) == 0);
	SPI.beginTransaction(s);

	if ( prefix ) {
		SPI.transfer(prefix | WRITE_BURST);
		SPI.transfer(startReg);
	} else
		SPI.transfer(startReg | WRITE_BURST);

	for(uint8_t i=0; i<length; i++) {
		SPI.transfer(pArr[i]);
  }
	SPI.endTransaction();
	digitalWrite(SS_PIN, HIGH);
}
