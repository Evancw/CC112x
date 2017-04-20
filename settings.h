// Rf settings for CC1125
RF_SETTINGS code rfSettings = {
    0xB0,  // IOCFG3              GPIO3 IO Pin Configuration
    0x06,  // IOCFG2              GPIO2 IO Pin Configuration
    0xB0,  // IOCFG1              GPIO1 IO Pin Configuration
    0x40,  // IOCFG0              GPIO0 IO Pin Configuration
    0x08,  // SYNC_CFG1           Sync Word Detection Configuration Reg. 1
    0xA3,  // DEVIATION_M         Frequency Deviation Configuration
    0x02,  // MODCFG_DEV_E        Modulation Format and Frequency Deviation Configur..
    0x1C,  // DCFILT_CFG          Digital DC Removal Configuration
    0x33,  // FREQ_IF_CFG         RX Mixer Frequency Configuration
    0xC6,  // IQIC                Digital Image Channel Compensation Configuration
    0x10,  // CHAN_BW             Channel Filter Configuration
    0x05,  // MDMCFG0             General Modem Parameter Configuration Reg. 0
    0x3F,  // SYMBOL_RATE2        Symbol Rate Configuration Exponent and Mantissa [1..
    0x75,  // SYMBOL_RATE1        Symbol Rate Configuration Mantissa [15:8]
    0x10,  // SYMBOL_RATE0        Symbol Rate Configuration Mantissa [7:0]
    0x20,  // AGC_REF             AGC Reference Level Configuration
    0x0C,  // AGC_CS_THR          Carrier Sense Threshold Configuration
    0xA9,  // AGC_CFG1            Automatic Gain Control Configuration Reg. 1
    0x00,  // FIFO_CFG            FIFO Configuration
    0x03,  // SETTLING_CFG        Frequency Synthesizer Calibration and Settling Con..
    0x12,  // FS_CFG              Frequency Synthesizer Configuration
    0x20,  // WOR_CFG0            eWOR Configuration Reg. 0
    0x02,  // WOR_EVENT0_MSB      Event 0 Configuration MSB
    0xE9,  // WOR_EVENT0_LSB      Event 0 Configuration LSB
    0x20,  // PKT_CFG0            Packet Configuration Reg. 0
    0x09,  // RFEND_CFG0          RFEND Configuration Reg. 0
    0x74,  // PA_CFG2             Power Amplifier Configuration Reg. 2
    0xFF,  // PKT_LEN             Packet Length Configuration
    0x00,  // IF_MIX_CFG          IF Mix Configuration
    0x22,  // FREQOFF_CFG         Frequency Offset Correction Configuration
    0x5B,  // FREQ2               Frequency Configuration [23:16]
    0x80,  // FREQ1               Frequency Configuration [15:8]
    0x05,  // IF_ADC0             Analog to Digital Converter Configuration Reg. 0
    0x00,  // FS_DIG1             Frequency Synthesizer Digital Reg. 1
    0x5F,  // FS_DIG0             Frequency Synthesizer Digital Reg. 0
    0x0E,  // FS_CAL0             Frequency Synthesizer Calibration Reg. 0
    0x03,  // FS_DIVTWO           Frequency Synthesizer Divide by 2
    0x33,  // FS_DSM0             FS Digital Synthesizer Module Configuration Reg. 0
    0x17,  // FS_DVC0             Frequency Synthesizer Divider Chain Configuration ..
    0x50,  // FS_PFD              Frequency Synthesizer Phase Frequency Detector Con..
    0x6E,  // FS_PRE              Frequency Synthesizer Prescaler Configuration
    0x14,  // FS_REG_DIV_CML      Frequency Synthesizer Divider Regulator Configurat..
    0xAC,  // FS_SPARE            Frequency Synthesizer Spare
    0x0E,  // XOSC5               Crystal Oscillator Configuration Reg. 5
    0xC7,  // XOSC3               Crystal Oscillator Configuration Reg. 3
    0x07,  // XOSC1               Crystal Oscillator Configuration Reg. 1
};
