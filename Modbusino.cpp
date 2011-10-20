/*
 * Copyright © 2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 */

#include <inttypes.h>

#include "WProgram.h"
#include "Modbusino.h"

#define _MODBUS_RTU_SLAVE              0
#define _MODBUS_RTU_FUNCTION           1
#define _MODBUS_RTU_PRESET_REQ_LENGTH  6
#define _MODBUS_RTU_PRESET_RSP_LENGTH  2

#define _MODBUS_RTU_CHECKSUM_LENGTH    2

#define _MODBUSINO_RTU_MAX_ADU_LENGTH  128

/* Supported function codes */
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10

enum {
    _STEP_FUNCTION = 0x01,
    _STEP_META,
    _STEP_DATA
};

extern HardwareSerial modbus_uart;

static uint16_t crc16(uint8_t *req, uint8_t req_length)
{
    uint8_t j;
    uint16_t crc;

    crc = 0xFFFF;
    while (req_length--) {
	crc = crc ^ *req++;
	for (j=0; j < 8; j++) {
	    if (crc & 0x0001)
		crc = (crc >> 1) ^ 0xA001;
	    else
		crc = crc >> 1;
	}
    }

    return (crc << 8  | crc >> 8);
}


void ModbusinoSlave::setup(int slave, long baud) {
    setup(slave, baud, 0);
}

void ModbusinoSlave::setup(int slave, long baud, uint16_t base_address) {
    setup(slave, baud, base_address, 0);
}

void ModbusinoSlave::setup(int slave, long baud, uint16_t base_address, uint8_t pin_txe) {
    _base_addr = base_address;
    if ((slave >= 0) && (slave <= 247)) {
	_slave = slave;
    }
    if (pin_txe > 0) {
        _pin_txe = pin_txe;
        pinMode(_pin_txe, OUTPUT);
        digitalWrite(_pin_txe, LOW);
    }
    modbus_uart.begin(baud);
}

static int check_integrity(uint8_t *msg, uint8_t msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;

    if (msg_length < 2)
	return -1;

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calculated == crc_received) {
        return msg_length;
    } else {
        return -1;
    }
}

static int build_response_basis(uint8_t slave, uint8_t function,
				uint8_t* rsp)
{
    rsp[0] = slave;
    rsp[1] = function;

    return _MODBUS_RTU_PRESET_RSP_LENGTH;
}

static void send_msg(uint8_t pin_txe, uint8_t *msg, uint8_t msg_length)
{
    uint16_t crc = crc16(msg, msg_length);

    msg[msg_length++] = crc >> 8;
    msg[msg_length++] = crc & 0x00FF;

    // These delays are pseudo-science. You _may_ need to tweak them.
    if (pin_txe > 0) {
        digitalWrite(pin_txe, HIGH);
        delay(1);
    }
    modbus_uart.write(msg, msg_length);
    if (pin_txe > 0) {
        delay(4);
        digitalWrite(pin_txe, LOW);
    }
}

static uint8_t response_exception(uint8_t slave, uint8_t function,
				  uint8_t exception_code,
				  uint8_t *rsp)
{
    uint8_t rsp_length;

    rsp_length = build_response_basis(slave, function + 0x80, rsp);

    /* Positive exception code */
    rsp[rsp_length++] = exception_code;

    return rsp_length;
}

/**
 * Wait for the interbyte timeout to expire, flushing as we go
 */
static void flush(void)
{
    uint8_t i = 0;
    while (++i < MODBUS_TIMEOUT_INTERBYTE) {
        modbus_uart.flush();
        delay(1);
        if (modbus_uart.available()) {
            i = 0;
        }
    }
}

int ModbusinoSlave::mb_slave_receive(uint8_t *req)
{
    uint8_t i;
    uint8_t length_to_read;
    uint8_t req_index;
    uint8_t step;
    uint8_t function = 0;

    /* We need to analyse the message step by step.  At the first step, we want
     * to reach the function code because all packets contain this
     * information. */
    step = _STEP_FUNCTION;
    length_to_read = _MODBUS_RTU_FUNCTION + 1;

    req_index = 0;
    while (length_to_read != 0) {

	/* The timeout is defined to ~10 ms between each bytes.  Precision is
	   not that important so I rather to avoid millis() to apply the KISS
	   principle (millis overflows after 50 days, etc) */
        if (!modbus_uart.available()) {
	    i = 0;
	    while (!modbus_uart.available()) {
		delay(1);
		if (++i == MODBUS_TIMEOUT_INTERBYTE) {
		    /* Too late, bye */
		    return -1 - MODBUS_INFORMATIVE_RX_TIMEOUT;
		}
	    }
        }

	req[req_index] = modbus_uart.read();

        /* Moves the pointer to receive other data */
	req_index++;

        /* Computes remaining bytes */
        length_to_read--;

        if (length_to_read == 0) {
            if (req[_MODBUS_RTU_SLAVE] != _slave &&
                req[_MODBUS_RTU_SLAVE != MODBUS_BROADCAST_ADDRESS]) {
                flush();
                return - 1 - MODBUS_INFORMATIVE_NOT_FOR_US;
            }

            switch (step) {
            case _STEP_FUNCTION:
                /* Function code position */
		function = req[_MODBUS_RTU_FUNCTION];
		if (function == _FC_READ_HOLDING_REGISTERS) {
		    length_to_read = 4;
		} else if (function == _FC_WRITE_MULTIPLE_REGISTERS) {
		    length_to_read = 5;
		} else {
		    /* Wait a moment to receive the remaining garbage */
		    flush();
		    /* It's for me so send an exception (reuse req) */
                    // TODO - no response if broadcast
		    uint8_t rsp_length = response_exception(_slave, function,
			    MODBUS_EXCEPTION_ILLEGAL_FUNCTION, req);
		    send_msg(_pin_txe, req, rsp_length);
		    return - 1 - MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
		}
		step = _STEP_META;
		break;
            case _STEP_META:
		length_to_read = _MODBUS_RTU_CHECKSUM_LENGTH;

		if (function == _FC_WRITE_MULTIPLE_REGISTERS)
		    length_to_read += req[_MODBUS_RTU_FUNCTION + 5];

                if ((req_index + length_to_read) > _MODBUSINO_RTU_MAX_ADU_LENGTH) {
		    flush();
		    /* It's for me so send an exception (reuse req) */
                    // TODO - no response if broadcast
		    uint8_t rsp_length = response_exception(_slave, function,
			    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, req);
		    send_msg(_pin_txe, req, rsp_length);
		    return - 1 - MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
                }
                step = _STEP_DATA;
                break;
            default:
                break;
            }
        }
    }
    return check_integrity(req, req_index);
}

/**
 * Returns 0 if request is partially, or completely, out of range
 */
static int request_in_range(uint16_t request_base, uint8_t request_count,
                            uint16_t our_base, uint8_t our_count) {
    if (request_base + request_count > our_base + our_count) {
        // they want to read beyond our last register
        return 0;
    }
    if (request_base < our_base) {
        // no matter how many they want, at least one is before our base
        return 0;
    }
    return 1;
}


void ModbusinoSlave::mb_slave_reply(uint16_t *tab_reg, uint8_t nb_reg,
		  uint8_t *req, uint8_t req_length)
{
    uint8_t slave = req[_MODBUS_RTU_SLAVE];
    uint8_t function = req[_MODBUS_RTU_FUNCTION];
    uint16_t req_address = (req[_MODBUS_RTU_FUNCTION + 1] << 8) +
	req[_MODBUS_RTU_FUNCTION + 2];
    uint16_t nb = (req[_MODBUS_RTU_FUNCTION + 3] << 8) +
	req[_MODBUS_RTU_FUNCTION + 4];
    uint8_t rsp[_MODBUSINO_RTU_MAX_ADU_LENGTH];
    uint8_t rsp_length = 0;

    if (slave != _slave &&
	slave != MODBUS_BROADCAST_ADDRESS) {
	return;
    }

    if (!request_in_range(req_address, nb, _base_addr, nb_reg)) {
	rsp_length = response_exception(
	    slave, function,
	    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
        send_msg(_pin_txe, rsp, rsp_length);
        return;
    }

    req_length -= _MODBUS_RTU_CHECKSUM_LENGTH;

    if (function == _FC_READ_HOLDING_REGISTERS) {
	uint8_t i;
	rsp_length = build_response_basis(slave, function, rsp);
	rsp[rsp_length++] = nb << 1;
        int li = req_address - _base_addr;  // start at the beginning relative to zero
	for (i = 0; i < nb; i++, li++) {
	    rsp[rsp_length++] = tab_reg[li] >> 8;
	    rsp[rsp_length++] = tab_reg[li] & 0xFF;
	}
    } else if (function == _FC_WRITE_MULTIPLE_REGISTERS) {
        unsigned int i = 0; // counts off their requests
        int j; // offset in their message
        int li = req_address - _base_addr;  // logical addressing
        j = 6;  // offset to first actual data word
	for (; i < nb; i++, j += 2, li++) {
	    /* 6 and 7 = first value */
	    tab_reg[li] = (req[_MODBUS_RTU_FUNCTION + j] << 8) +
		req[_MODBUS_RTU_FUNCTION + j + 1];
	}

	rsp_length = build_response_basis(slave, function, rsp);
	/* 4 to copy the req_address (2) and the no. of registers */
	memcpy(rsp + rsp_length, req + rsp_length, 4);
	rsp_length += 4;
    } else {
        // OH CRAP!
        // Receive should have already sent an exception here...
    }

    send_msg(_pin_txe, rsp, rsp_length);
}

int ModbusinoSlave::loop(uint16_t* tab_reg, uint8_t nb_reg)
{
    int rc = 0;
    uint8_t req[_MODBUSINO_RTU_MAX_ADU_LENGTH];

    if (modbus_uart.available()) {
	rc = mb_slave_receive(req);
	if (rc > 0) {
	    mb_slave_reply(tab_reg, nb_reg, req, rc);
	}
    }

    /* Returns a positive value if successful,
       0 if a slave filtering has occured, or no data present
       -1 if an undefined error has occured,
       -2 for MODBUS_EXCEPTION_ILLEGAL_FUNCTION
       etc */
    return rc;
}
