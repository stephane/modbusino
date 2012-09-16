 /*
 * Copyright © 2011-2012 Stéphane Raimbault <stephane.raimbault@gmail.com>
 * License ISC, see LICENSE for more details.
 */

#include <Modbusino.h>

/* Initialize the slave with the ID 1 */
ModbusinoSlave modbusino_slave(1);
/* Allocate a mapping of 10 values */
uint16_t tab_reg[10];

void setup() {
    /* The transfer speed is set to 115200 bauds */
    modbusino_slave.setup(115200);
}

void loop() {
    /* Initialize the first register to have a value to read */
    tab_reg[0] = 0x1234;
    /* Launch Modbus slave loop with:
       - pointer to the mapping
       - max values of mapping */
    modbusino_slave.loop(tab_reg, 10);
}
