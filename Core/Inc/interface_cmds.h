/*
 * interface_cmds.h
 *
 *  Created on: Dec 27, 2023
 *  Revised on: Mar 29, 2024
 *
 *      Author: Wojciech Kaczmarski, SP5WWP, M17 Project
 *
 *   Reference: TBF
 */
#pragma once

enum cari_cmd_t
{
	CMD_PING,

	//SET
	CMD_SET_RX_FREQ,
	CMD_SET_TX_FREQ,
	CMD_SET_TX_POWER,
	CMD_SET_RESERVED_1,
	CMD_SET_RX_FREQ_CORR,
	CMD_SET_TX_FREQ_CORR,
	CMD_SET_AFC,
	CMD_SET_RESERVED_2,
	CMD_SET_RX,
	CMD_SUB_CONNECT,
	CMD_STREAM_DATA,

	//GET
	CMD_GET_IDENT = 0x80,
	CMD_GET_CAPS,
	CMD_GET_RX_FREQ,
	CMD_GET_TX_FREQ,
	CMD_GET_TX_POWER,
	CMD_GET_FREQ_CORR,
	CMD_GET_MEAS
};

enum cari_err_t
{
	ERR_OK,					//all good
	ERR_MALFORMED,			//malformed frame
	ERR_CMD_UNSUP,			//command unsupported
	ERR_ZMQ_BIND,			//ZMQ port bind failed
	ERR_ZMQ_CONN,			//ZMQ connection failed
	ERR_RANGE				//out of range
};

enum cari_cpbl_t
{
	CAP_AM = 1,
	CAP_FM,
	CAP_SSB,
	CAP_PSK,
	CAP_IQ,
	CAP_DUPLEX = 7
};
