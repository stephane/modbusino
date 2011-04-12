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

static uint16_t crc16(uint8_t *req, int req_length)
{
    int j;
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

	/* The timeout is defined to ~10 ms between each bytes.  Precision is
	   not that important so I rather to avoid millis() to apply the KISS
	   principle (millis overflows after 50 days, etc) */
        if (!Serial.available()) {
	    int i=0;
	    while (!Serial.available()) {
		delay(1);
		if (++i == 10) {
		    /* Too late, bye */
		    return -1;
		}
	    }
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
