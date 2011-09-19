#include <Modbusino.h>

// This is where you can specify which serial port to use
HardwareSerial modbus_uart = Serial;
ModbusinoSlave modbusino_slave;
uint16_t tab_reg[10];

void setup() {
    // slave id = 1, serial baud rate = 115200, 8n1
    modbusino_slave.setup(1, 115200);
}

void loop() {
    tab_reg[0] = 0x1234;
    modbusino_slave.loop(tab_reg, 10);
}
