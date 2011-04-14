#include <Modbusino.h>

ModbusinoSlave modbusino_slave(1);
uint16_t tab_reg[10];

void setup() {
    modbusino_slave.setup(115200);
}

void loop() {
    tab_reg[0] = 0x1234;
    modbusino_slave.loop(tab_reg, 10);
}
