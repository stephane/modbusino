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
    modbusino_slave.loop(tab_reg, 10);
}
```

Contribute
----------

I want to keep this library very basic and small so if you want to contribute:

1. Check for open issues or open a fresh issue to start a discussion around a feature idea or a bug.
2. Fork the [repository](https://github.com/stephane/modbusino/) on Github to start making your changes on another
   branch.
3. Send a pull request (of your small and atomic changes).
4. Bug the maintainer if he's too busy to answer :)
