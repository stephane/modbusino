Modbusino
=========

Introduction
------------
This project fork from https://github.com/stephane/modbusino.

Modbusino is a ISC licensed library to handle Modbus requests on Arduino
(slave).


Features
--------

To keep it simple and to reduce memory consumption, only the two following
Modbus functions are supported:

* read holding registers (0x03)
* write multiple registers (0x10)

Example
-------

```c
#include <Modbusino.h>

/* Initialize the slave */
ModbusinoSlave modbusino_slave;
/* Allocate a mapping of 10 values */
uint16_t tab_reg[10];

void setup() {
    /* Set the slave address with the ID 1 */
    /* The transfer speed is set to 115200 bauds */
    /* Set the serial port is Serial */
    modbusino_slave.setup(1, 115200, &Serial);
}

void loop() {
    /* Initialize the first register to have a value to read */
    tab_reg[0] = 0x1234;
    /* Launch Modbus slave loop with:
       - pointer to the mapping
       - max values of mapping */
    uint32_t t = 0;

    t = modbusino_slave.loop(tab_reg, 10);
    
    uint16_t dlen = temp;
    uint16_t addr = temp >> 16;

    /* If the Modbus commend is 16(0x10):
       - dlen: length of the regs that want write
       - addr: index of the start address 
       Else, t always is 0*/

}
```
