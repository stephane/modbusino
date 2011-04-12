/* TODO
   Slow CRC code
*/
#include <inttypes.h>

#include "WProgram.h"
#include "Modbusino.h"

/* Request message on the server side */
#define MSG_INDICATION 1
/* Request message on the client side */
#define MSG_CONFIRMATION 2

#define _MODBUS_RTU_HEADER_LENGTH      1
#define _MODBUS_RTU_PRESET_REQ_LENGTH  6
#define _MODBUS_RTU_PRESET_RSP_LENGTH  2

#define _MODBUS_RTU_CHECKSUM_LENGTH    2

/* Function codes */
#define _FC_READ_COILS                0x01
#define _FC_READ_DISCRETE_INPUTS      0x02
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_READ_INPUT_REGISTERS      0x04
#define _FC_WRITE_SINGLE_COIL         0x05
#define _FC_WRITE_SINGLE_REGISTER     0x06
#define _FC_READ_EXCEPTION_STATUS     0x07
#define _FC_WRITE_MULTIPLE_COILS      0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10
#define _FC_REPORT_SLAVE_ID           0x11
#define _FC_READ_AND_WRITE_REGISTERS  0x17

enum {
    _STEP_FUNCTION = 0x01,
    _STEP_META,
    _STEP_DATA
};

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

Modbusino::Modbusino(void) {
    _slave = -1;
    _nb_register = 0;
    tab_reg = NULL;
}

void Modbusino::set_slave(int slave) {
    _slave = slave;
}

void Modbusino::mapping_new(int nb_register) {
    _nb_register = nb_register;
    tab_reg = (uint16_t *)malloc(nb_register * sizeof(uint16_t));
}

void Modbusino::mapping_free(void) {
    _nb_register = 0;
    free(tab_reg);
}

uint8_t Modbusino::_compute_meta_length_after_function(int function,
						       int msg_type)
{
    int length;

    if (msg_type == MSG_INDICATION) {
	if (function <= _FC_WRITE_SINGLE_REGISTER) {
            length = 4;
        } else if (function == _FC_WRITE_MULTIPLE_COILS ||
                   function == _FC_WRITE_MULTIPLE_REGISTERS) {
            length = 5;
        } else if (function == _FC_READ_AND_WRITE_REGISTERS) {
            length = 9;
        } else {
            /* _FC_READ_EXCEPTION_STATUS, _FC_REPORT_SLAVE_ID */
            length = 0;
        }
    } else {
        /* MSG_CONFIRMATION */
        switch (function) {
        case _FC_WRITE_SINGLE_COIL:
        case _FC_WRITE_SINGLE_REGISTER:
        case _FC_WRITE_MULTIPLE_COILS:
        case _FC_WRITE_MULTIPLE_REGISTERS:
            length = 4;
            break;
        default:
            length = 1;
        }
    }

    return length;
}

int Modbusino::_compute_data_length_after_meta(uint8_t *msg,
						      int msg_type)
{
    int function = msg[_MODBUS_RTU_HEADER_LENGTH];
    int length;

    if (msg_type == MSG_INDICATION) {
        switch (function) {
        case _FC_WRITE_MULTIPLE_COILS:
        case _FC_WRITE_MULTIPLE_REGISTERS:
            length = msg[_MODBUS_RTU_HEADER_LENGTH + 5];
            break;
        case _FC_READ_AND_WRITE_REGISTERS:
            length = msg[_MODBUS_RTU_HEADER_LENGTH + 9];
            break;
        default:
            length = 0;
        }
    } else {
        /* MSG_CONFIRMATION */
        if (function <= _FC_READ_INPUT_REGISTERS ||
            function == _FC_REPORT_SLAVE_ID ||
            function == _FC_READ_AND_WRITE_REGISTERS) {
            length = msg[_MODBUS_RTU_HEADER_LENGTH + 1];
        } else {
            length = 0;
        }
    }
    length += _MODBUS_RTU_CHECKSUM_LENGTH;

    return length;
}

int Modbusino::_check_integrity(uint8_t *msg, const int msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calculated == crc_received) {
        return msg_length;
    } else {
        return -1;
    }
}

int Modbusino::_receive_msg(uint8_t *msg, int msg_type)
{
    int rc;
    int length_to_read;
    int msg_index;
    int step;

    /* We need to analyse the message step by step.  At the first step, we want
     * to reach the function code because all packets contain this
     * information. */
    step = _STEP_FUNCTION;
    length_to_read = _MODBUS_RTU_HEADER_LENGTH + 1;

    msg_index = 0;
    while (length_to_read != 0) {

	/* FIXME To replace by a timeout of 3 ms */
	delay(1);

        if (!Serial.available()) {
            rc = -1;
        }

	msg[msg_index] = Serial.read();

        /* Moves the pointer to receive other data */
	msg_index++;

        /* Computes remaining bytes */
        length_to_read--;

        if (length_to_read == 0) {
            switch (step) {
            case _STEP_FUNCTION:
                /* Function code position */
                length_to_read = _compute_meta_length_after_function(
                    msg[_MODBUS_RTU_HEADER_LENGTH],
                    msg_type);
                if (length_to_read != 0) {
                    step = _STEP_META;
                    break;
                } /* else switches straight to the next step */
            case _STEP_META:
                length_to_read = _compute_data_length_after_meta(
                    msg, msg_type);
                if ((msg_index + length_to_read) > 256) {
                    /*
		      errno = EMBBADDATA;
                    _error_print(ctx, "too many data");
		    */
                    return -1;
                }
                step = _STEP_DATA;
                break;
            default:
                break;
            }
        }
    }

    return _check_integrity(msg, msg_index);
}

int Modbusino::receive(uint8_t *req) {
    return _receive_msg(req, MSG_INDICATION);
}

int Modbusino::_build_response_basis(int slave, int function, uint8_t* rsp)
{
    rsp[0] = slave;
    rsp[1] = function;

    return _MODBUS_RTU_PRESET_RSP_LENGTH;
}

int Modbusino::_send_msg(uint8_t *req, int req_length)
{
    uint16_t crc = crc16(req, req_length);

    req[req_length++] = crc >> 8;
    req[req_length++] = crc & 0x00FF;

    Serial.write(req, req_length);
}

int Modbusino::_response_exception(int slave, int function, int exception_code,
				   uint8_t *rsp)
{
    int rsp_length;

    function = function + 0x80;
    rsp_length = _build_response_basis(slave, function, rsp);

    /* Positive exception code */
    rsp[rsp_length++] = exception_code;

    return rsp_length;
}

int Modbusino::reply(uint8_t *req, int req_length) {
    int offset = _MODBUS_RTU_HEADER_LENGTH;
    int slave = req[offset - 1];
    int function = req[offset];
    uint16_t address = (req[offset + 1] << 8) + req[offset + 2];
    uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];
    int rsp_length = 0;

    /* FIXME No filtering */
    req_length -= _MODBUS_RTU_CHECKSUM_LENGTH;

    switch (function) {
    case _FC_READ_HOLDING_REGISTERS: {
	int nb = (req[offset + 3] << 8) + req[offset + 4];

	if (address + nb > _nb_register) {
	    rsp_length = _response_exception(
		slave, function,
		MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
	} else {
	    int i;
	    rsp_length = _build_response_basis(slave, function, rsp);
	    rsp[rsp_length++] = nb << 1;
	    for (i = address; i < address + nb; i++) {
		rsp[rsp_length++] = tab_reg[i] >> 8;
		rsp[rsp_length++] = tab_reg[i] & 0xFF;
	    }
	}
    }
        break;
    case _FC_WRITE_MULTIPLE_REGISTERS: {
	int nb = (req[offset + 3] << 8) + req[offset + 4];
	if ((address + nb) > _nb_register) {
	    rsp_length = _response_exception(
		slave, function,
		MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
	} else {
	    int i, j;

	    for (i = address, j = 6; i < address + nb; i++, j += 2) {
		/* 6 and 7 = first value */
		tab_reg[i] = (req[offset + j] << 8) + req[offset + j + 1];
	    }

	    rsp_length = _build_response_basis(slave, function, rsp);
	    /* 4 to copy the address (2) and the no. of registers */
	    memcpy(rsp + rsp_length, req + rsp_length, 4);
	    rsp_length += 4;
	}
    }
	break;
    }

    return _send_msg(rsp, rsp_length);
}
