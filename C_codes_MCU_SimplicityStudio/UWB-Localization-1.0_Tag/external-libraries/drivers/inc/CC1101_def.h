#ifndef DRIVERS_CC1101_DEF_H_
#define DRIVERS_CC1101_DEF_H_

#define CC1101_FIFO_SIZE                        64

#define CC1101_DATA_38kBaud_CONFIG 				1
#define CC1101_WAKEUP_CONFIG 					2
#define CC1101_WAKEUP_CONFIG_NARROW_BAND		3
#define CC1101_SPECTRAL_ANALYSIS				4

/* Configuration registers. */
#define CC1101_REG_IOCFG2                       0x00
#define CC1101_REG_IOCFG1                       0x01
#define CC1101_REG_IOCFG0                       0x02
#define CC1101_REG_FIFOTHR                      0x03
#define CC1101_REG_SYNC1                        0x04
#define CC1101_REG_SYNC0                        0x05
#define CC1101_REG_PKTLEN                       0x06
#define CC1101_REG_PKTCTRL1                     0x07
#define CC1101_REG_PKTCTRL0                     0x08
#define CC1101_REG_ADDR                         0x09
#define CC1101_REG_CHANNR                       0x0A
#define CC1101_REG_FSCTRL1                      0x0B
#define CC1101_REG_FSCTRL0                      0x0C
#define CC1101_REG_FREQ2                        0x0D
#define CC1101_REG_FREQ1                        0x0E
#define CC1101_REG_FREQ0                        0x0F
#define CC1101_REG_MDMCFG4                      0x10
#define CC1101_REG_MDMCFG3                      0x11
#define CC1101_REG_MDMCFG2                      0x12
#define CC1101_REG_MDMCFG1                      0x13
#define CC1101_REG_MDMCFG0                      0x14
#define CC1101_REG_DEVIATN                      0x15
#define CC1101_REG_MCSM2                        0x16
#define CC1101_REG_MCSM1                        0x17
#define CC1101_REG_MCSM0                        0x18
#define CC1101_REG_FOCCFG                       0x19
#define CC1101_REG_BSCFG                        0x1A
#define CC1101_REG_AGCCTRL2                     0x1B
#define CC1101_REG_AGCCTRL1                     0x1C
#define CC1101_REG_AGCCTRL0                     0x1D
#define CC1101_REG_WOREVT1                      0x1E
#define CC1101_REG_WOREVT0                      0x1F
#define CC1101_REG_WORCTRL                      0x20
#define CC1101_REG_FREND1                       0x21
#define CC1101_REG_FREND0                       0x22
#define CC1101_REG_FSCAL3                       0x23
#define CC1101_REG_FSCAL2                       0x24
#define CC1101_REG_FSCAL1                       0x25
#define CC1101_REG_FSCAL0                       0x26
#define CC1101_REG_RCCTRL1                      0x27
#define CC1101_REG_RCCTRL0                      0x28
#define CC1101_REG_FSTEST                       0x29
#define CC1101_REG_PTEST                        0x2A
#define CC1101_REG_AGCTEST                      0x2B
#define CC1101_REG_TEST2                        0x2C
#define CC1101_REG_TEST1                        0x2D
#define CC1101_REG_TEST0                        0x2E

/* Status registers. */
#define CC1101_REG_PARTNUM                      0x30
#define CC1101_REG_VERSION                      0x31
#define CC1101_REG_FREQEST                      0x32
#define CC1101_REG_LQI                          0x33
#define CC1101_REG_RSSI                         0x34
#define CC1101_REG_MARCSTATE                    0x35
#define CC1101_REG_WORTIME1                     0x36
#define CC1101_REG_WORTIME0                     0x37
#define CC1011_REG_PKTSTATUS                    0x38
#define CC1101_REG_VCO_VC_DAC                   0x39
#define CC1101_REG_TXBYTES                      0x3A
#define CC1101_REG_RXBYTES                      0x3B
#define CC1101_REG_RCCTRL1_STATUS               0x3C
#define CC1101_REG_RCCTRL0_STATUS               0x3D

/* Multi byte registers. */
#define CC1101_REG_PATABLE                      0x3E
#define CC1101_REG_TXFIFO                       0x3F
#define CC1101_REG_RXFIFO                       0x3F

/* Strobe commands. */
#define CC1101_SRES                             0x30
#define CC1101_SFSTXON                          0x31
#define CC1101_SXOFF                            0x32
#define CC1101_SCAL                             0x33
#define CC1101_SRX                              0x34
#define CC1101_STX                              0x35
#define CC1101_SIDLE                            0x36
#define CC1101_SWOR                             0x38
#define CC1101_SPWD                             0x39
#define CC1101_SFRX                             0x3A
#define CC1101_SFTX                             0x3B
#define CC1101_SWORRST                          0x3C
#define CC1101_SNOP                             0x3D


#define CC1101_STATUS_CHIP_RDYn_BM              0x80
#define CC1101_STATUS_STATE_BM                  0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM   0x0F

#define CC1101_STATE_IDLE                       0x00
#define CC1101_STATE_RX                         0x10
#define CC1101_STATE_TX                         0x20
#define CC1101_STATE_FSTXON                     0x30
#define CC1101_STATE_CALIBRATE                  0x40
#define CC1101_STATE_SETTLING                   0x50
#define CC1101_STATE_RXFIFO_OVERFLOW            0x60
#define CC1101_STATE_TXFIFO_OVERFLOW            0x70

#define CC1101_MARC_STATE_SLEEP                 0x00
#define CC1101_MARC_STATE_IDLE                  0x01
#define CC1101_MARC_STATE_XOFF                  0x02
#define CC1101_MARC_STATE_VCOON_MC              0x03
#define CC1101_MARC_STATE_REGON_MC              0x04
#define CC1101_MARC_STATE_MANCAL                0x05
#define CC1101_MARC_STATE_VCOON                 0x06
#define CC1101_MARC_STATE_REGON                 0x07
#define CC1101_MARC_STATE_STARTCAL              0x08
#define CC1101_MARC_STATE_BWBOOST               0x09
#define CC1101_MARC_STATE_FS_LOCK               0x0A
#define CC1101_MARC_STATE_IFADCON               0x0B
#define CC1101_MARC_STATE_ENDCAL                0x0C
#define CC1101_MARC_STATE_RX                    0x0D
#define CC1101_MARC_STATE_RX_END                0x0E
#define CC1101_MARC_STATE_RX_RST                0x0F
#define CC1101_MARC_STATE_TXRX_SWITCH           0x10
#define CC1101_MARC_STATE_RXFIFO_OVERFLOW       0x11
#define CC1101_MARC_STATE_FSTXON                0x12
#define CC1101_MARC_STATE_TX                    0x13
#define CC1101_MARC_STATE_TX_END                0x14
#define CC1101_MARC_STATE_RXTX_SWITCH           0x15
#define CC1101_MARC_STATE_TXFIFO_UNDERFLOW      0x16

#define CC1101_LQI_CRC_OK_BM                    0x80
#define CC1101_LQI_EST_BM                       0x7F

#define CC1101_CARRIER_SENSE	                0x0E


#endif /* DRIVERS_CC1101_DEF_H_ */
