/*
 * DWM1000.h
 *
 *  Created on: 03.05.2016
 *      Author: Patrick Fehrenbach
 */

#ifndef DWM1000_H_
#define DWM1000_H_

#include "config.h"
#include "revisionDef.h"

#include <stdbool.h>
#include <stdint.h>

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "TIMER.h"
#include "CRC.h"
#include "RTC.h"
#include "DWM1000_reg.h"
#include "DWM1000_device_param.h"

#define DWM1000_CS_PORT 	gpioPortF
#define DWM1000_CS_PIN 		2

#define DWM1000_MOSI_PORT	gpioPortE
#define DWM1000_MOSI_PIN 	10

#define DWM1000_MISO_PORT 	gpioPortE
#define DWM1000_MISO_PIN 	11

#define DWM1000_CLK_PORT 	gpioPortE
#define DWM1000_CLK_PIN		12

#define DWM1000_RSTn_PORT	gpioPortB
#define DWM1000_RSTn_PIN	11

#define DWM1000_IRQ_PORT	gpioPortB
#define DWM1000_IRQ_PIN		14

// Defines for enable_clocks function
#define FORCE_SYS_XTI  		0
#define ENABLE_ALL_SEQ 		1
#define FORCE_SYS_PLL  		2
#define READ_ACC_ON    		7
#define READ_ACC_OFF   		8
#define FORCE_OTP_ON   		11
#define FORCE_OTP_OFF  		12
#define FORCE_TX_PLL   		13

// OTP addresses definitions
#define LDOTUNE_ADDRESS 	0x04
#define PARTID_ADDRESS 		0x06
#define LOTID_ADDRESS  		0x07
#define VBAT_ADDRESS   		0x08
#define VTEMP_ADDRESS  		0x09
#define XTRIM_ADDRESS  		0x1E

typedef struct
{
    uint32_t      deviceID ;
    uint32_t      partID ;
    uint32_t      lotID ;
    uint32_t	  panID;
    uint8_t       chan;               	// Added channel here - used in the reading of accumulator
    uint8_t       longFrames ;        	// Flag in non-standard long frame mode
    uint8_t       otprev ;            	// OTP revision number (read during initialisation)
    uint32_t      txFCTRL ;           	// Keep TX_FCTRL register config
    uint8_t       xtrim;              	// XTAL trim value read from OTP
    uint8_t       dblbuffon;          	// Double RX buffer mode flag
    uint32_t      sysCFGreg ;        	// Local copy of system config register
    uint16_t      sleep_mode;         	// Used for automatic reloading of LDO tune and microcode at wake-up
    uint32_t	  sysctrl;				// System control register to enable TX or RX
    uint32_t      sysstatus;			// System Event Status Register
    uint32_t	  chancontrreg;			// Channel control register
    uint32_t	  sysmaskctrl;			// System Event Mask Register
    uint8_t		  sfdlength;			// SFD length
    uint8_t		  pacSize;				// PAC size
    uint8_t		  preaCode;				// TX/RX Preamble Code
    uint32_t	  pmscreg;				// PMSC reg for clock setting

    int 		  deviceMode;			// what mode of the Dw1000 we are in

    dwt_callback_data_t cdata;      	// Callback data structure

    uint8_t       wait4resp ;         	// wait4response was set with last TX start command
    int           prfIndex ;

    void (*dwt_txcallback)(const dwt_callback_data_t *txd);
    void (*dwt_rxcallback)(const dwt_callback_data_t *rxd);

} dwt_local_data_t ;


void DWM1000_GPIO_set_mode(uint8_t gpio_pin, uint32_t mode);

void DWM1000_set_GPIOs_sec_mode();

void DWM1000_set_GPIO_mode();

/* Init only the SPI_CS Pin to High because of low active CS */
void DWM1000_spi_default();

/*Init for the SPI of the DWM1000 Module */
void DWM1000_spi_init(uint32_t SPI_BAUDRATE);

uint8_t DWM1000_init();

void DWM1000_configuration(volatile dwt_config_t *config);

uint32_t DWM1000_get_device_ID(void);

void DWM1000_spi_read_burst_reg(uint16_t address, uint16_t index, uint8_t * const buffer, uint32_t count);

void DWM1000_spi_write_burst_reg(uint16_t address, uint16_t index, uint8_t * const txbuffer, uint32_t count);

void DWM1000_write16bitoffsetregister(int address, int index, uint16_t value);

void DWM1000_write32bitoffsetregister(int address, int index, uint32_t value);

uint16_t DWM1000_read16bitoffsetregister(int address, int index);

uint32_t DWM1000_read32bitoffsetregister(int address, int index);

void DWM1000_short_reset(void);

void DWM1000_enable_clocks(uint8_t clock);

uint16_t get_lde_replicaCoeff(uint16_t pcode);

uint32_t DWM1000_read_OTP(uint32_t address);

void DWM1000_XTALtrim(uint8_t trimvalue);

void DWM1000_loaducodefromrom(void);

void configure_lde(uint8_t prfindex);

void set_FS_PLL_config_tune(uint8_t chan);

void set_rx_ctrl(uint8_t chan);

void set_tx_ctrl(uint8_t chan);

void set_drx_tune0b(uint8_t datarate, uint8_t nssfd);

void set_drx_tune1a(uint8_t prfindex);

void set_drx_tune1b_4H(uint8_t datarate, uint8_t txpreamlength);

void set_drx_tune2(uint8_t prfindex, uint8_t rxpac);

void set_drx_tune3(uint16_t sfdTO);

void set_agc_tune1(uint8_t prfindex);

void set_sfd_improved(uint8_t datarate);

void DWM1000_set_rxantennadelay(uint16_t rxDelay);

void DWM1000_set_txantennadelay(uint16_t txDelay);

void DWM1000_set_rxaftertxdelay(uint32_t rxDelayTime);

void DWM1000_set_rxtimeout(uint16_t time);

void set_TC_PGDELAY (uint8_t chan);

void set_TX_Power_config(uint8_t prfindex, uint8_t chan, uint8_t smarttxpower);

void DWM1000_config_sleep_mode(uint16_t mode, uint8_t wakeup);

void DWM1000_enter_sleepmode(void);

void DWM1000_AON_arrayupload(void);

void DWM1000_SPI_Wake_Up(uint8_t *wakeupbuffer, uint16_t length);

void DWM1000_send_blink_message(uint8_t *tx_message, uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t message_length);

void DWM1000_send_poll_message(uint8_t *tx_message, uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t message_length);

int DWM1000_send_response_message(uint8_t *tx_message, uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t message_length);

int DWM1000_send_final_message(uint8_t *tx_message, uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t message_length, uint64_t poll_tx_timestamp, uint64_t resp_rx_timestamp);

void DWM1000_set_message_array_bytes(uint8_t *buffer, uint64_t writebytes, uint8_t numberofbytes);

void DWM1000_get_message_array_bytes(uint8_t *buffer, uint64_t *readbytes, uint8_t numberofbytes);

void DWM1000_prepare_tx_message(uint8_t *tx_packet, uint8_t packet_length, uint16_t txbufferoffset);

int DWM1000_start_transmit(uint8_t txmode, uint8_t rxmode);



void DWM1000_get_receive_message(uint8_t *tx_poll_msg);

void DWM1000_read_receive_data(uint8_t *tx_frame, uint16_t length, uint16_t bufferoffset);

int DWM1000_check_receive_message(uint8_t *rx_buffer, uint32_t *SOURCE_ID, uint32_t *DESTIN_ID, uint8_t *MSG_TYPE);


int DWM1000_enable_rx(int delayed);

void DWM1000_sync_rxbuffer_pointers(void);

void DWM1000_transceiver_off();



uint64_t DWM1000_get_rx_timestamp_40bit(void);

void DWM1000_read_rx_timestamp_40bit(uint8_t *rx_timestamp);

uint64_t DWM1000_get_tx_timestamp_40bit(void);

void DWM1000_read_tx_timestamp_40bit(uint8_t *tx_timestamp);

uint64_t DWM1000_set_delayed_txrx_time(uint64_t start_time);

uint32_t DWM1000_get_rx_timestamp_32bit(void);

uint32_t DWM1000_get_tx_timestamp_32bit(void);

void DWM1000_set_irq_mask_register(uint32_t irq_mask);

void DWM1000_clear_irq_mask_register(void);


enum states_tag DWM1000_receiver_on_state(void);

void DWM1000_idle_mode(void);

void DWM1000_new_receive();

void DWM1000_permanent_receive(bool val);

void DWM1000_new_transmit(void);

void DWM1000_start_Receive();

void DWM1000_correct_timestamp(uint64_t *timestamp);

int64_t DWM1000_check_time_overflow(int64_t timeoverflow);

void DWM1000_configuration2(volatile dwt_config_t *config);

void DWM1000_receiver();

void DWM1000_UWB_NODE(uint32_t ID_WHOWOKE_ME_UP, uint16_t measurement_numbers);

void DWM1000_UWB_TAG(float *range, uint16_t measurement_numbers, uint32_t NODE_ADD);

void DWM1000_Chip_INIT();

enum states_tag DWM1000_poll_receive_state(uint8_t *rx_buffer, uint32_t *SOURCE_ID, uint32_t *DESTIN_ID, uint8_t *MSG_TYPE, uint64_t *poll_rx_ts);

enum states_tag DWM1000_response_transmit_state(uint32_t DESTIN_ID, uint32_t SOURCE_ID, uint64_t *resp_tx_ts);

enum states_tag DWM1000_blink_transmit_state(uint32_t DESTIN_ID, uint32_t SOURCE_ID);

enum states_tag DWM1000_poll_transmit_state(uint32_t DESTIN_ID, uint32_t SOURCE_ID, uint64_t *poll_tx_ts);

enum states_tag DWM1000_response_receive_state(uint8_t *rx_buffer, uint32_t *SOURCE_ID, uint32_t *DESTIN_ID, uint8_t *MSG_TYPE, uint64_t *resp_rx_ts);


float DWM1000_compute_range_asymmetric(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts, uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts);

float DWM1000_compute_range_symmetric(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts, uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts);

#endif /* DWM1000_H_ */



