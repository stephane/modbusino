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

#ifndef Modbusino_h
#define Modbusino_h

#include <inttypes.h>
#include "WProgram.h"

#define MODBUS_BROADCAST_ADDRESS 0

/* Protocol exceptions */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION     1
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 2
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE   3
#define MODBUS_INFORMATIVE_NOT_FOR_US   4
#define MODBUS_INFORMATIVE_RX_TIMEOUT   5

class ModbusinoSlave {
public:
    /**
     * use the given slaveid and baud rate for the serial port
     * @param slave
     * @param baud
     */
    void setup(int slave, long baud);

    /**
     * Use a given base address as logical 0
     * @param slave
     * @param baud
     * @param base_address
     */
    void setup(int slave, long baud, uint16_t base_address);

    /**
     * As per setup(), but also use the given pin as a tx enable pin
     * 
     * This is for use on multi drop RS485 networks.
     * @param slave
     * @param pin_txe 0 will disable the pin
     * @param baud
     */
    void setup(int slave, long baud, uint16_t base_address, uint8_t pin_txe);

    
    /**
     * Call this often!
     * @param tab_reg logical table of registers.  This will be offset to base_address, if given
     * @param nb_reg number of registers
     * @return 
     */
    int loop(uint16_t *tab_reg, uint8_t nb_reg);
private:
    int mb_slave_receive(uint8_t *req);
    void mb_slave_reply(uint16_t *tab_reg, uint8_t nb_reg, uint8_t *req, uint8_t req_length);
    uint16_t _base_addr;
    uint8_t _slave;
    int _pin_txe;
};

#endif
