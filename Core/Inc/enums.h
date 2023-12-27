/*
 * enums.h
 *
 *  Created on: Dec 27, 2023
 *      Author: SP5WWP
 */

#ifndef INC_ENUMS_H_
#define INC_ENUMS_H_

enum trx_t
{
	CHIP_RX,
	CHIP_TX
};

enum tp_t
{
	TP1,
	TP2
};

enum transf_t
{
	TX_SINGLE,
	TX_BURST,
	RX_SINGLE,
	RX_BURST
};

enum strobe_t
{
	STR_SRES = 0x30,		//reset
	STR_SCAL = 0x33,		//calibrate
	STR_SRX,				//set state to "receive"
	STR_STX,				//set state to "transmit"
	STR_IDLE,				//set state to "idle"
	STR_SNOP = 0x3D			//no operation
};

enum trx_state_t
{
	TX_IDLE,
	TX_ACTIVE,
	RX_IDLE,
	RX_ACTIVE
};

enum interface_comm_t
{
	COMM_IDLE,
	COMM_RDY,
	COMM_TOT,
	COMM_OVF
};

#endif /* INC_ENUMS_H_ */
