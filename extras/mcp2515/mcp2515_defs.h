#ifndef _MCP2515_DEFS__H_
#define _MCP2515_DEFS__H_


/* MCP2515 commands */
#define MCP2515_CMD_RESET			(0xC0)
#define MCP2515_CMD_BIT_MODIFY		(0x05)
#define MCP2515_CMD_WRITE_REG		(0x02)
#define MCP2515_CMD_READ_REG		(0x03)
#define MCP2515_CMD_READ_STATUS 	(0xA0)
#define MCP2515_CMD_READ_RX_STATUS	(0xB0)
#define MCP2515_CMD_READ_RXB0SIDH	(0x90)
#define MCP2515_CMD_READ_RXB0D0		(0x92)
#define MCP2515_CMD_READ_RXB1SIDH	(0x94)
#define MCP2515_CMD_READ_RXB1D0		(0x96)



/* can frame header stuff */
#define SIDH (0)
#define SIDL (1)
#define EID8 (2)
#define EID0 (3)
#define DLC  (4)
#define DATA (5)


#define _BV(x)		(1 << x)
#define MCP2515_STATUS_RX0IF	_BV(0)
#define MCP2515_STATUS_RX1IF	_BV(1)


#define MCP2515_1000kBPS_CFG1 (0x00)
#define MCP2515_1000kBPS_CFG2 (0x80)
#define MCP2515_1000kBPS_CFG3 (0x00)

#define MCP2515_500kBPS_CFG1 (0x00)
#define MCP2515_500kBPS_CFG2 (0x90)
#define MCP2515_500kBPS_CFG3 (0x02)

#define MCP2515_250kBPS_CFG1 (0x00)
#define MCP2515_250kBPS_CFG2 (0xb1)
#define MCP2515_250kBPS_CFG3 (0x05)

#define MCP2515_200kBPS_CFG1 (0x00)
#define MCP2515_200kBPS_CFG2 (0xb4)
#define MCP2515_200kBPS_CFG3 (0x06)

#define MCP2515_125kBPS_CFG1 (0x01)
#define MCP2515_125kBPS_CFG2 (0xb1)
#define MCP2515_125kBPS_CFG3 (0x05)

#define MCP2515_100kBPS_CFG1 (0x01)
#define MCP2515_100kBPS_CFG2 (0xb4)
#define MCP2515_100kBPS_CFG3 (0x06)

#define MCP2515_80kBPS_CFG1 (0x01)
#define MCP2515_80kBPS_CFG2 (0xbf)
#define MCP2515_80kBPS_CFG3 (0x07) // check this

#define MCP2515_50kBPS_CFG1 (0x03)
#define MCP2515_50kBPS_CFG2 (0xb4)
#define MCP2515_50kBPS_CFG3 (0x06)

#define MCP2515_40kBPS_CFG1 (0x03)
#define MCP2515_40kBPS_CFG2 (0xbf)
#define MCP2515_40kBPS_CFG3 (0x07)

#define MCP2515_33kBPS_CFG1 (0x47) //
#define MCP2515_33kBPS_CFG2 (0xe2) //
#define MCP2515_33kBPS_CFG3 (0x85) // check this all


#define MCP2515_31k25BPS_CFG1 (0x07)
#define MCP2515_31k25BPS_CFG2 (0xa4)
#define MCP2515_31k25BPS_CFG3 (0x04)

#define MCP2515_20kBPS_CFG1 (0x07)
#define MCP2515_20kBPS_CFG2 (0xbf)
#define MCP2515_20kBPS_CFG3 (0x07)

#define MCP2515_10kBPS_CFG1 (0x0f)
#define MCP2515_10kBPS_CFG2 (0xbf)
#define MCP2515_10kBPS_CFG3 (0x07)

#define MCP2515_5kBPS_CFG1 (0x1f)
#define MCP2515_5kBPS_CFG2 (0xbf)
#define MCP2515_5kBPS_CFG3 (0x07)

#endif // _MCP2515_DEFS__H_