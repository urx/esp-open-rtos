/*!
\file
\brief MCP2515 helper functions
*/

#ifndef _MCP2515__H_
#define _MCP2515__H_

#include <stdint.h>
#include <stdbool.h>

#define CAN_DATA_LEN_MAX 	8 ///< maximal length of CAN frame data


/**
\brief Possible errors which mcp2515 library can return
*/
typedef enum CAN_ERRORS {
	CAN_NO_ERROR = 0,			///< on success
	CAN_MESSAGE_LEN_ERROR,		///< can frame has invalid lenght
	CAN_BUS_ERROR,				///< other can related errors
} can_error_t;


/**
\brief CAN frame representation
*/
typedef struct can_frame {
	uint32_t 	can_id;					///< ID
	uint8_t		can_dlc;				///< data length
	uint8_t		data[CAN_DATA_LEN_MAX]; ///< data itself
} can_frame_t;


/**
\brief MCP2515 connection info
*/
typedef struct mcp2515_device {
	uint8_t spi_bus_no; ///< number of ESP8266 SPI bus
	uint8_t cs_pin; ///< CS pin number
} mcp2515_device_t;

/**
\brief MCP2515 operation modes
*/
typedef enum OPERATION_MODE {
	NORMAL_OPERATION_MODE = 0,
	SLEEP_MODE,
	LOOPBACK_MODE,
	LISTEN_ONLY_MODE,
	CONFIGURATION_MODE,
} mcp2515_operation_mode_t;

/**
\brief CAN bus baudrates
*/
typedef enum CAN_SPEED {
 CAN_5KBPS,
 CAN_10KBPS,
 CAN_20KBPS,
 CAN_31K25BPS,
 CAN_33KBPS,
 CAN_40KBPS,
 CAN_50KBPS,
 CAN_80KBPS,
 CAN_100KBPS,
 CAN_125KBPS,
 CAN_200KBPS,
 CAN_250KBPS,
 CAN_500KBPS,
 CAN_1000KBPS,
 CAN_SPEED_MAX,
} mcp2515_can_speed_t;

/**
\brief RX buffer id
*/
typedef enum RX_BUFFER {
	RXB0,
	RXB1,
} mcp2515_rx_buffer_t;

/**
\brief MCP2515 interrupt sources
*/

#define	RX0IE BIT(0) ///< RX0 buffer full interrupt
#define RX1IE BIT(1) ///< RX1 buffer full interrupt
#define	TX0IE BIT(2) ///< TX0 buffer empty interrupt
#define	TX1IE BIT(3) ///< TX1 buffer empty interrupt
#define TX2IE BIT(4) ///< TX2 buffer empty interrupt
#define	ERRIE BIT(5) ///< Error interrupt
#define	WAKIE BIT(6) ///< Wakeup interrupt
#define MERRE BIT(7) ///< Message error interrupt

/* CANINTF bits definition */
#define CANINTF_MERRF   			BIT(7)
#define CANINTF_WAKIF				BIT(6)
#define CANINTF_ERRIF				BIT(5)
#define CANINTF_TX2IF				BIT(4)
#define CANINTF_TX1IF				BIT(3)
#define CANINTF_TX0IF				BIT(2)
#define CANINTF_RX1IF				BIT(1)
#define CANINTF_RX0IF				BIT(0)

/* EFLG bits definition */
#define EFLG_RX1OVR					BIT(7)
#define EFLG_RX0OVR					BIT(6)
#define EFLG_TXBO					BIT(5)
#define EFLG_TXEP					BIT(4)
#define EFLG_RXEP					BIT(3)
#define EFLG_TXWAR					BIT(2)
#define EFLG_RXWAR					BIT(1)
#define EFLG_EWARN					BIT(0)

/*!
\brief Setup MCP2515 connection parameters.
This function does nothing on MCP2515 itself, 
but setup SPI bus number and CS pin which is used for
communication to MCP2515 ic.
\param[in] dev -- MCP2515 connection settings
*/
void mcp2515_setup_device(mcp2515_device_t *dev);

/*!
\brief Perform MCP2515 soft reset procedure.
*/
void mcp2515_soft_reset(void);

/*!
\brief Read MCP2515 device status.
Exec READ STATUS command on MCP2515 device.
\return MCP2515 device satus
*/
uint8_t mcp2515_read_status(void);

/*!
\brief Configures MCP2515 device operation mode.
\param mode[in] -- desired operation mode
*/
void mcp2515_set_operation_mode(mcp2515_operation_mode_t mode);

/*!
\brief Read current MCP2515 device operation mode.
\return current MCP2515 operation mode.

*/
mcp2515_operation_mode_t mcp2515_get_operation_mode(void);

/*!
\brief Request of specific operation mode of MCP2515 device.
Sets specified operation mode of MCP2515 and read current operation
mode ather that. If current mode does not equal specified --
do vTaskDelay() for 200ms and repeat all routine from begining.

CAUTION: you can stuck in that function forever if some error occures.

\todo may be add some iteration counter and return error 
	  if counter value exceed some threshold value.
\return CAN_NO_ERROR on success, error code otherwise.
*/
uint8_t mcp2515_request_operation_mode(mcp2515_operation_mode_t mode);

/*!
\brief Configure MCP2515 device for specified CAN speed.
\param speed -- desired speed.
\return CAN_NO_ERROR on success, error code therwise.
*/
int mcp2515_set_can_speed(mcp2515_can_speed_t speed);

//uint8_t mcp2515_get_message_length(mcp2515_rx_buffer_t buf);

void mcp2515_set_filters(mcp2515_rx_buffer_t buf, bool on);

/*!
\brief Perform basic initialization over MCP2515 device
\param[in] speed -- CAN bus baudrate (\ref mcp2515_can_speed_t)
\return CAN_NO_ERROR on succes or error code
*/
int mcp2515_init(mcp2515_can_speed_t speed);

/*!
\brief Enables/disables specified interrupt source
\param[in] irq_mask -- interrupt source mask 
\param[in] enable -- new state of interrupt source
*/
void mcp2515_int_enable(uint8_t irq_mask, bool enable);

/*!
*	\brief Read one can frame from receive buffers
*
*	\param frm -- pointer to can_farme_t structure, used to save received frame
*	\return CAN_NO_ERROR on succes, error code otherwise
*/
uint8_t mcp2515_read_can_frame(can_frame_t *frm);

uint8_t mcp2515_read_eflg(void);

void mcp2515_clear_can_irq_flags(uint8_t mask);
uint8_t mcp2515_read_can_irq_flags();
uint8_t mcp2515_clear_eflg(uint8_t mask);


#endif // _MCP2515__H_
