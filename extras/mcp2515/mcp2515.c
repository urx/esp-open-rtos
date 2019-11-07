#include <esp/gpio.h>
#include <esp/spi.h>
#include <FreeRTOS.h>
#include <task.h>

#include "mcp2515_regs.h"
#include "mcp2515_defs.h"
#include "mcp2515.h"


static mcp2515_device_t mcp2515_dev = {
	.spi_bus_no = 0,
	.cs_pin = 0,
};


#undef MCP2515_DBG 
#ifdef MCP2515_DBG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif


/*static*/ uint8_t mcp2515_read_reg(uint8_t const reg)
{
	uint8_t ret=0x55;

    gpio_write(mcp2515_dev.cs_pin, false);
    spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_READ_REG);
    spi_transfer_8(mcp2515_dev.spi_bus_no, reg);
    ret = spi_transfer_8(mcp2515_dev.spi_bus_no, 0xFF);
    gpio_write(mcp2515_dev.cs_pin, true);

    return ret;
}

static void mcp2515_read_regs(uint8_t start, uint8_t count, uint8_t out[])
{
	gpio_write(mcp2515_dev.cs_pin, false);
	spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_READ_REG);

	for (int i = 0; i < count; i++) {
		out[ i ] = spi_transfer_8(mcp2515_dev.spi_bus_no, 0x00);
	}
	gpio_write(mcp2515_dev.cs_pin, true);
}


static void mcp2515_write_reg(uint8_t reg, uint8_t const data)
{
	gpio_write(mcp2515_dev.cs_pin, false);
	spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_WRITE_REG);
	spi_transfer_8(mcp2515_dev.spi_bus_no, reg);
	spi_transfer_8(mcp2515_dev.spi_bus_no, data);
	gpio_write(mcp2515_dev.cs_pin, true);

}

static void mcp2515_write_regs(uint8_t start, uint8_t count, uint8_t data[])
{
	gpio_write(mcp2515_dev.cs_pin, false);
	spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_WRITE_REG);
	spi_transfer_8(mcp2515_dev.spi_bus_no, start);

	for (int i = 0; i < count; i++) {
		spi_transfer_8(mcp2515_dev.spi_bus_no, data[i]);
	}

	gpio_write(mcp2515_dev.cs_pin, true);
}

static void mcp2515_modify_reg(uint8_t reg, uint8_t mask, uint8_t data)
{
	gpio_write(mcp2515_dev.cs_pin, false);
	spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_BIT_MODIFY);
	spi_transfer_8(mcp2515_dev.spi_bus_no, reg);
	spi_transfer_8(mcp2515_dev.spi_bus_no, mask);
	spi_transfer_8(mcp2515_dev.spi_bus_no, data);
	gpio_write(mcp2515_dev.cs_pin, true);
}


void mcp2515_setup_device(mcp2515_device_t *dev)
{
	mcp2515_dev.spi_bus_no = dev->spi_bus_no;
	mcp2515_dev.cs_pin = dev->cs_pin;
}

void mcp2515_soft_reset(void)
{
	gpio_write(mcp2515_dev.cs_pin, false);
	spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_RESET);
	sdk_os_delay_us(500);
	gpio_write(mcp2515_dev.cs_pin, true);
}

void mcp2515_enable_interrupts(uint8_t irq_mask)
{
	mcp2515_modify_reg(CANINTE_REG, irq_mask, irq_mask);
}

int mcp2515_init(mcp2515_can_speed_t speed)
{
	mcp2515_soft_reset();
	if (mcp2515_request_operation_mode(CONFIGURATION_MODE) < 0) {
		return 1;
	}

    /* set RX0BF and RX1BF into Z state */
	mcp2515_write_reg(BFPCTRL_REG, 0x00);

	/* set TXxRTS pins as inputs */
	mcp2515_write_reg(TXRTSCTRL_REG, 0x00);

	/* turn off filtering */
	mcp2515_set_filters(RXB0, false);
	mcp2515_set_filters(RXB1, false);

    if (mcp2515_set_can_speed(speed)) {
		DEBUG_PRINT(("mcp2515 init failed, ic unaccessible\n"));
		return 1;
	}

    /* enable RX0 and RX1 interrupts */
	mcp2515_enable_interrupts(CANINTE_RX0IE_MASK | CANINTE_RX1IE_MASK);

	DEBUG_PRINT(("mcp2515 initialization done\n"));
	return 0;
}


static void mcp2515_read_header(mcp2515_rx_buffer_t buf, uint8_t *out)
{
	gpio_write(mcp2515_dev.cs_pin, false);
	if (buf == RXB0) {
		spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_READ_RXB0SIDH);
	} else {
		spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_READ_RXB0SIDH);
	}
	for (int i = 0; i < 5; i ++) {
		*(out + i) = spi_transfer_8(mcp2515_dev.spi_bus_no, 0x0);
	}
	gpio_write(mcp2515_dev.cs_pin, true);
}

static void mcp2515_read_data(mcp2515_rx_buffer_t buf, uint8_t length, uint8_t *out)
{
	gpio_write(mcp2515_dev.cs_pin, false);
	if (buf == RXB0) {
    	spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_READ_RXB0D0);
	} else {
		spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_READ_RXB1D0);
	}
	
	for (int i = 0; i < length; i++) {
		*(out+i) = spi_transfer_8(mcp2515_dev.spi_bus_no, 0x0); 
	}
	gpio_write(mcp2515_dev.cs_pin, true);

}

uint8_t mcp2515_read_status(void)
{
	uint8_t reg = 0x0;
	gpio_write(mcp2515_dev.cs_pin, false);
	spi_transfer_8(mcp2515_dev.spi_bus_no, MCP2515_CMD_READ_STATUS);
	reg = spi_transfer_8(mcp2515_dev.spi_bus_no, 0x0);
	gpio_write(mcp2515_dev.cs_pin, true);
	return reg;
}

void mcp2515_set_operation_mode( mcp2515_operation_mode_t mode)
{
	mcp2515_modify_reg(CANCTRL_REG,
					   CANCTRL_REQOP0_MASK | CANCTRL_ABAT_MASK,
		               (mode << CANCTRL_REQOP0_OFF) | (1 << CANCTRL_ABAT_OFF));
}

mcp2515_operation_mode_t mcp2515_get_operation_mode(void)
{
	return (mcp2515_read_reg(CANSTAT_REG) & CANSTAT_OPMOD_MASK) >> CANSTAT_OPMOD_OFF;
}


uint8_t mcp2515_request_operation_mode(mcp2515_operation_mode_t mode)
{
	mcp2515_operation_mode_t m;
again:
	mcp2515_set_operation_mode(mode);
	vTaskDelay(200 / portTICK_PERIOD_MS);
	m = mcp2515_get_operation_mode();
    if (mode == m)
			return 0;

	DEBUG_PRINT(("%s: request 0x%01X, got 0x%01X\n", __FUNCTION__, mode, m));
	goto again;
	//return -1;
}

int mcp2515_set_can_speed(mcp2515_can_speed_t speed)
{
	uint8_t cnf1, cnf2, cnf3;
	switch (speed) {
        case (CAN_5KBPS) :
          cnf1 = MCP2515_5kBPS_CFG1;
          cnf2 = MCP2515_5kBPS_CFG2;
          cnf3 = MCP2515_5kBPS_CFG3;
          break;

        case (CAN_10KBPS) :
          cnf1 = MCP2515_10kBPS_CFG1;
          cnf2 = MCP2515_10kBPS_CFG2;
          cnf3 = MCP2515_10kBPS_CFG3;
          break;

        case (CAN_20KBPS) :
          cnf1 = MCP2515_20kBPS_CFG1;
          cnf2 = MCP2515_20kBPS_CFG2;
          cnf3 = MCP2515_20kBPS_CFG3;
          break;

        case (CAN_31K25BPS) :
          cnf1 = MCP2515_31k25BPS_CFG1;
          cnf2 = MCP2515_31k25BPS_CFG2;
          cnf3 = MCP2515_31k25BPS_CFG3;
          break;

        case (CAN_33KBPS) :
          cnf1 = MCP2515_33kBPS_CFG1;
          cnf2 = MCP2515_33kBPS_CFG2;
          cnf3 = MCP2515_33kBPS_CFG3;
          break;

        case (CAN_40KBPS) :
          cnf1 = MCP2515_40kBPS_CFG1;
          cnf2 = MCP2515_40kBPS_CFG2;
          cnf3 = MCP2515_40kBPS_CFG3;
          break;

        case (CAN_50KBPS) :
          cnf1 = MCP2515_50kBPS_CFG1;
          cnf2 = MCP2515_50kBPS_CFG2;
          cnf3 = MCP2515_50kBPS_CFG3;
          break;

        case (CAN_80KBPS) :
          cnf1 = MCP2515_80kBPS_CFG1;
          cnf2 = MCP2515_80kBPS_CFG2;
          cnf3 = MCP2515_80kBPS_CFG3;
          break;

        case (CAN_100KBPS) :
          cnf1 = MCP2515_100kBPS_CFG1;
          cnf2 = MCP2515_100kBPS_CFG2;
          cnf3 = MCP2515_100kBPS_CFG3;
          break;

        case (CAN_125KBPS) :
          cnf1 = MCP2515_125kBPS_CFG1;
          cnf2 = MCP2515_125kBPS_CFG2;
          cnf3 = MCP2515_125kBPS_CFG3;
          break;

        case (CAN_200KBPS) :
          cnf1 = MCP2515_200kBPS_CFG1;
          cnf2 = MCP2515_200kBPS_CFG2;
          cnf3 = MCP2515_200kBPS_CFG3;
          break;

        case (CAN_250KBPS) :
          cnf1 = MCP2515_250kBPS_CFG1;
          cnf2 = MCP2515_250kBPS_CFG2;
          cnf3 = MCP2515_250kBPS_CFG3;
          break;

        case (CAN_500KBPS) :
          cnf1 = MCP2515_500kBPS_CFG1;
          cnf2 = MCP2515_500kBPS_CFG2;
          cnf3 = MCP2515_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS) :
          cnf1 = MCP2515_1000kBPS_CFG1;
          cnf2 = MCP2515_1000kBPS_CFG2;
          cnf3 = MCP2515_1000kBPS_CFG3;
          break;

        default:
			return -1;
	}

	mcp2515_write_reg(CNF1_REG, cnf1);
	mcp2515_write_reg(CNF2_REG, cnf2);
	mcp2515_write_reg(CNF3_REG, cnf3);

	if (mcp2515_read_reg(CNF1_REG) != cnf1) {
		DEBUG_PRINT(("setting CNF1_REG failed."));
		return -1;
	}
	if (mcp2515_read_reg(CNF2_REG) != cnf2) {
		DEBUG_PRINT(("setting CNF1_REG failed."));
		return -1;
	}
	if (mcp2515_read_reg(CNF3_REG) != cnf3) {
		DEBUG_PRINT(("setting CNF1_REG failed."));
		return -1;
	}

	return 0;
}

uint8_t mcp2515_get_message_length(mcp2515_rx_buffer_t buf)
{
	if (buf == RXB0) {
		return (mcp2515_read_reg(RXB0DLC_REG) & RXBxDLC_DLC_MASK) >> RXBxDLC_DLC_OFF;
	}

	return (mcp2515_read_reg(RXB1DLC_REG) & RXBxDLC_DLC_MASK) >> RXBxDLC_DLC_OFF;
}

void mcp2515_set_filters(mcp2515_rx_buffer_t buf, bool on)
{
	if (buf == RXB0) {
		mcp2515_modify_reg(RXB0CTRL_REG, RXB0CTRL_RXM_MASK,
						   (on) ? ~RXB0CTRL_RXM_MASK : RXB0CTRL_RXM_MASK);
		return;
	}

	mcp2515_modify_reg(RXB1CTRL_REG, RXB1CTRL_RXM_MASK,
					   (on) ?  ~RXB1CTRL_RXM_MASK : RXB1CTRL_RXM_MASK);
}

void mcp2515_int_enable(uint8_t mask, bool enable)
{
	uint8_t reg = 0x0;
	if (mask & RX0IE) reg |= CANINTE_RX0IE_MASK;
	if (mask & RX1IE) reg |= CANINTE_RX1IE_MASK;
	if (mask & TX0IE) reg |= CANINTE_TX0IE_MASK;
    if (mask & TX1IE) reg |= CANINTE_TX1IE_MASK;
    if (mask & TX2IE) reg |= CANINTE_TX2IE_MASK;
    if (mask & ERRIE) reg |= CANINTE_ERRIE_MASK;
    if (mask & WAKIE) reg |= CANINTE_WAKIE_MASK;
    if (mask & MERRE) reg |= CANINTE_MERRE_MASK;
	/* we can use reg itself as mask here, because we always modify one bit at once */
	mcp2515_modify_reg(CANINTE_REG, reg, (enable) ? reg : ~reg);
}

uint8_t mcp2515_read_eflg(void)
{
	return mcp2515_read_reg(EFLG_REG);
}

/*! \brief Read single can frame

	This function was intended to read single can frame from RX buffers.
	For now it does not handle rollover (at least it is not tested yet)

	@param frm -- pointer to can_frame_t for saving received frame
*/
uint8_t mcp2515_read_can_frame(can_frame_t *frm)
{
	uint8_t hdr[5] = {0};
	uint8_t status = mcp2515_read_status();
	uint8_t rxbctrl = 0;
	mcp2515_rx_buffer_t buf = RXB0;

	if (status & MCP2515_STATUS_RX0IF) {
		buf = RXB0;
	} else if (status & MCP2515_STATUS_RX1IF) {
		buf = RXB1;
	}

	mcp2515_read_header(buf, hdr);
	if (buf == RXB0) {
		rxbctrl = mcp2515_read_reg(RXB0CTRL_REG);
	} else {
		rxbctrl = mcp2515_read_reg(RXB1CTRL_REG);
	}

	/* parse frame header */
	frm->can_id = (hdr[SIDH] << 3) | (hdr[SIDL] >> 5); // 11 bit total

	/* check type of frame: extended or standard */
	if (hdr[SIDL] & RXBxSIDL_IDE_MASK) {
		/* extended frame */
		frm->can_id = (frm->can_id << 2) | hdr[SIDL] & 0x3; // EID 17..16
		frm->can_id = (frm->can_id << 8) | hdr[EID8]; 		// EID 15..8
		frm->can_id = (frm->can_id << 8) | hdr[EID0]; 		// EID 7..0
	}

	uint8_t frm_len = (hdr[DLC] & RXBxDLC_DLC_MASK) >> RXBxDLC_DLC_OFF;
	if (frm_len > CAN_DATA_LEN_MAX) {
		return CAN_MESSAGE_LEN_ERROR;
	}

	frm->can_dlc = frm_len;

	/* Check for remote transfer request */
	if (buf == RXB0) {
		frm->can_id = (rxbctrl & RXB0CTRL_RXRTR_MASK) ? frm->can_id |= (1 << 32) : frm->can_id;
	} else {
		frm->can_id = (rxbctrl & RXB1CTRL_RXRTR_MASK) ? frm->can_id |= (1 << 32) : frm->can_id;
	}

	/* read data bytes */
	mcp2515_read_data(buf, frm->can_dlc, frm->data);
	return 0;
}

void mcp2515_clear_can_irq_flags(uint8_t mask)
{
	mcp2515_modify_reg(CANINTF_REG, mask, ~mask);
}

uint8_t mcp2515_read_can_irq_flags()
{
	mcp2515_read_reg(CANINTF_REG);
}

uint8_t mcp2515_clear_eflg(uint8_t mask)
{
	mcp2515_modify_reg(EFLG_REG, mask, ~mask);
}
