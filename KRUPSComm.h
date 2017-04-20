#ifndef KRUPSComm_H
#define KRUPSComm_H

#include <Arduino.h>
#include <SPI.h>

/*------------------------------[debug output 1/0]----------------------------*/
#define CC1100_DEBUG 1					//must defined in main project

//**************************** pins ******************************************//
#define SCK_PIN  					13
#define MISO_PIN 					12
#define MOSI_PIN 					11
#define SS_PIN   					10
#define GDO2	  					3       //2 main, 5 remote, 3 M16
#define GDO0	 						99

/*----------------------[CC11XX - misc]---------------------------------------*/
#define EXTD_REGISTER  			 			0x2F	//47 registers
#define FIFOBUFFER			  				0x42  	//size of Fifo Buffer
#define RSSI_OFFSET_868MHZ		  	0x4E	//dec = 74
#define TX_RETRIES_MAX			  		0x05	//tx_retries_max
#define ACK_TIMEOUT			   				100		//ACK timeout in ms
#define CC1100_COMPARE_REGISTER 	0x00	//register compare 0=no compare 1=compare
#define BROADCAST_ADDRESS 		  	0x00	//broadcast address
#define CC1100_FREQ_315MHZ		  	0x01
#define CC1100_FREQ_434MHZ		  	0x02
#define CC1100_FREQ_868MHZ		  	0x03
#define CC1100_FREQ_915MHZ		  	0x04
#define CC1100_TEMP_ADC_MV		  	3.225	//3.3V/1023 . mV pro digit
#define CC1100_TEMP_CELS_CO		  	2.47	//Temperature coeffiKcient 2.47mV per Grad Celsius

/*---------------------------[CC11XX - R/W offsets]---------------------------*/
#define WRITE_SINGLE_BYTE  			0x00
#define WRITE_BURST  	   			0x40
#define READ_SINGLE_BYTE   			0x80
#define READ_BURST  	   			0xC0

/*------------------------[CC11XX - FIFO commands]----------------------------*/
// #define TXFIFO_BURST        		0x7F    //write burst only
 #define STANDARD_FIFO  		    0x3F    //write single only
// #define RXFIFO_BURST  	    		0xFF    //read burst only
// #define RXFIFO_SINGLE_BYTE  		0xBF    //read single only

/*----------------------[CC11XX - config register]----------------------------*/
#define IOCFG3						0x00		// GDO3 output pin config
#define IOCFG2	 					0x01		// GDO2 output pin config
#define IOCFG1   					0x02		// GDO1 output pin config
#define IOCFG0   					0x03		// GDO0 output pin config
#define SYNC3  						0x04		// Sync word configuration
#define SYNC2    					0x05
#define SYNC1						0x06
#define SYNC0						0x07
#define SYNC_CFG1   			    0x08		// Sync word detection
#define SYNC_CFG0 				    0x09
#define DEVIATION_M 			    0x0A		// Frequency Deviation
#define MODCFG_DEV_E  	 	        0x0B		// Modulation format and frequency deviation
#define DCFILT_CFG   			    0x0C		// Digital DC Removal
#define PREAMBLE_CFG1			    0x0D    // Preamble length config
#define PREAMBLE_CFG0  		        0x0E
#define FREQ_IF_CFG  		    	0x0F		// RX Mixer frequency
#define IQIC  	 					0x10		// Digital image channel compensation
#define CHAN_BW  	 				0x11		// Frequency control word, low byte
#define MDMCFG1  					0x12 		// Modem configuration
#define MDMCFG0  					0x13		// Modem configuration
#define SYMBOL_RATE2			    0x14		// Modem configuration
#define SYMBOL_RATE1  		        0x15		// Modem configuration
#define SYMBOL_RATE0			    0x16		// Modem configuration
#define AGC_REF  				   	0x17    // Modem deviation setting
#define AGC_CS_THR  	 		    0x18		// Main Radio Cntrl State Machine config
#define AGC_GAIN_ADJUST  	        0x19		// Main Radio Cntrl State Machine config
#define AGC_CFG3  	 			    0x1A		// Main Radio Cntrl State Machine config
#define AGC_CFG2   				    0x1B		// Frequency Offset Compensation config
#define AGC_CFG1  	 			    0x1C		// Bit Synchronization configuration
#define AGC_CFG0 					0x1D		// AGC control
#define FIFO_CFG 					0x1E		// AGC control
#define DEV_ADDR 					0x1F		// AGC control
#define SETTLING_CFG  		        0x20		// High byte Event 0 timeout
#define FS_CFG  					0x21		// Low byte Event 0 timeout
#define WOR_CFG1 					0x22		// Wake On Radio control
#define WOR_CFG0 					0x23		// Front end RX configuration
#define WOR_EVENT0_MSB		        0x24		// Front end TX configuration
#define WOR_EVENT0_LSB	        	0x25		// Frequency synthesizer calibration
#define PKT_CFG2 					0x26		// Frequency synthesizer calibration
#define PKT_CFG1 					0x27		// Frequency synthesizer calibration
#define PKT_CFG0 					0x28		// Frequency synthesizer calibration
#define RFEND_CFG1		    		0x29		// RC oscillator configuration
#define RFEND_CFG0		    		0x2A		// RC oscillator configuration
#define PA_CFG2						0x2B		// Frequency synthesizer cal control
#define PA_CFG1	 					0x2C		// Production test
#define PA_CFG0  					0x2D		// AGC test
#define PKT_LEN  					0x2E		// Various test settings

/*------------------------[CC11XX Extended Config registers]------------------*/
#define IF_MIX_CFG		    		0x00		// IF Mix config
#define FREQOFF_CFG		    		0x01		// frequency offset correction
#define FREQ2						0x0C		// frequency config
#define FREQ1						0x0D
#define FREQ0						0x0E
#define IF_ADC0						0x11		// ADC config
#define FS_DIG1						0x12		// frequency synthesizer
#define FS_DIG0						0x13
#define FS_CAL0						0x17		// frequency synthesizer calibration
#define FS_DIVTWO					0x19		// freqency synthesizer divide by 2
#define FS_DSM0						0x1B		// freqency synthesizer module config
#define FS_DVC0						0x1D		// freqency synthesizer divider chain
#define FS_PFD						0x1F		// freqency synthesizer phase frequency detector
#define FS_PRE						0x20		// frequency synthesizer prescaler
#define FS_REG_DIV_CML		        0x21		// frequency synthesizer divider regulator
#define FS_SPARE					0x22		// frequency synthesizer spare
#define XOSC5						0x32		// crystal oscillator config
#define XOSC3						0x34
#define XOSC1						0x36


/*------------------------[CC11XX-command strobes]----------------------------*/
#define SRES    		            0x30				// Reset chip
#define SFSTXON 	            	0x31				// Enable/calibrate freq synthesizer
#define SXOFF   	            	0x32				// Turn off crystal oscillator.
#define SCAL    	            	0x33				// Calibrate freq synthesizer & disable
#define SRX  	        			0x34				// Enable RX.
#define STX  	        			0x35				// Enable TX.
#define SIDLE  	            		0x36				// Exit RX / TX
#define SAFC  	            		0x37				// AFC adjustment of freq synthesizer
#define SWOR  	            		0x38				// Start automatic RX polling sequence
#define SPWD  	            		0x39				// Enter pwr down mode when CSn goes hi
#define SFRX  	            		0x3A				// Flush the RX FIFO buffer.
#define SFTX  	            		0x3B				// Flush the TX FIFO buffer.
#define SWORRST 	            	0x3C				// Reset real time clock.
#define SNOP  		            	0x3D				// No operation.

/*----------------------[CC11XX - status register]----------------------------*/
#define PARTNUM                   	0x8F		// Part number
#define VERSION                   	0x90		// Current version number
#define FREQEST                   	0x01		// Frequency offset estimate
#define LQI                        	0x74		// Demodulator estimate for link quality
#define RSSI1 	                	0x71		// Received signal strength indication
#define RSSI0						0x72
#define MARCSTATE                  	0x73		// Control state machine state
//#define WORTIME1                 	0xF6		// High byte of WOR timer
//#define WORTIME0                 	0xF7		// Low byte of WOR timer
#define PKTSTATUS                	0xF8		// Current GDOx status and packet status
#define VCO_VC_DAC                	0xF9		// Current setting from PLL cal module
#define NUM_TXBYTES                	0xD6		// Underflow and # of bytes in TXFIFO
#define NUM_RXBYTES                	0xD7		// Overflow and # of bytes in RXFIFO
#define FIFO_NUM_TX	        		0xD8		// Number of available bytes in TXFIFO
#define FIFO_NUM_RX	        		0xD9		// Number of available bytes in RXFIFO
#define RCCAL_FINE	 		        0x07 		// RC Oscillator Fine Calibration
#define RCCAL_COARSE 	   	        0x08 		// RC Oscillator Coarse Calibration
#define RCCAL_OFFSET	        	0x09		// RC Oscillator Offset Calibration

class KRUPSComm
{
	private:
		void commandStrobe(uint8_t instr) {STATUS = spiRead(instr);};

		uint8_t STATUS;
		SPISettings s = SPISettings(4000000, MSBFIRST, SPI_MODE0);

	public:
		bool begin(void);
		void end(void);

		uint8_t spiRead(uint8_t reg, uint8_t prefix=0);
		void spiWrite(uint8_t reg, uint8_t data, uint8_t prefix=0);
		void spi_write_burst(uint8_t startReg, uint8_t *pArr, uint8_t length, uint8_t prefix=0);
		void spi_read_burst(uint8_t startReg, uint8_t *pArr, uint8_t length, uint8_t prefix=0);

		void reset(void);
		void wakeup(void);
		void powerdown(void);
		void setIDLE(void);
		void transmit(void);
		void receive(void);

		//void show_register_settings(void);

		int packet_available(void) {return spiRead(FIFO_NUM_RX, EXTD_REGISTER) & 0x0F;};
		bool wait_for_packet(uint8_t milliseconds);

		uint8_t get_payload(uint8_t rxbuffer[], uint8_t &pktlen_rx, int8_t &rssi_dbm, uint8_t &lqi);

		void tx_payload_burst(uint8_t *txbuffer, uint8_t length);
		void rx_payload_burst(uint8_t rxbuffer[], uint8_t &pktlen);

		void rx_fifo_erase(uint8_t *rxbuffer) {memset(rxbuffer, 0, sizeof(FIFOBUFFER));};
		void tx_fifo_erase(uint8_t *txbuffer) {memset(txbuffer, 0, sizeof(FIFOBUFFER));};

		uint8_t sent_packet(uint8_t *txbuffer, uint8_t pktlen, uint8_t tx_retries);
		void sent_acknowledge(uint8_t my_addr, uint8_t tx_addr);

		uint8_t check_acknowledge(uint8_t *rxbuffer, uint8_t pktlen);

		float rssi_convert(void);
		uint8_t check_crc(uint8_t lqi) {return (lqi & 0x80);};
		uint8_t lqi_convert(uint8_t lqi) {return (lqi & 0x7F);};
		//uint8_t get_temp(uint8_t *ptemp_Arr);

		//void set_myaddr(uint8_t addr);
		//void set_channel(uint8_t channel);
		//void set_ISM(uint8_t ism_freq);
		//void set_mode(uint8_t mode);
		//void set_output_power_level(int8_t dbm);
		//void set_patable(uint8_t *patable_arr);
		void set_data_whitening(bool whitening);
		void set_manchester_encoding(bool manchester);

};

#endif // KRUPSComm_H
