// A more involved example of a modbus slave on a RS495 network.
// Considered to be released into the public domain.

#include <Modbusino.h>

// Use the second serial port on a arduino mega
HardwareSerial modbus_uart = Serial1;
ModbusinoSlave slave;

#define MB_BASE_ADDRESS  0x2000
const int pin_tx_enable = 6;  // rs485 transceiver
const int pin_led = 11;

enum my_registers {
  mb_ticks = 0,
  mb_ticks_lo,
  mb_my_16bit_value,
  mb_delta_somevalue,
  // Add more register names here....
  mb_register_count
};

uint16_t tab_reg[mb_register_count];


/**
 * Portably write a 32 bit number into 2 16 bit modbus registers, bigendian.
 * DO NOT use casting tricks, they rely on the endianess of the platform
 * for instance, for writing 0x12345678 to register 4,
 * register 4 gets 0x1234 and register 5 gets 0x5678
 * @param table_base pointer to the register table
 * @param reg_number which register number is being used. (this will be the "top")
 * @param value the value to write
 */
void modbus_set_32(uint16_t *table_base, uint16_t reg_number, uint32_t value) {
  table_base[reg_number] = value >> 16 & 0xffff;
  table_base[reg_number + 1] = value & 0xffff;
}

uint8_t determine_slave_id() {
  // perhaps read in a byte from a 1-wire serial number chip, 
  // so that we can automatically address ourself.
  // but for now, just show that the slaveid can be determined from code
  return 0x69;
}

uint16_t read_some_hardware() {
  // just something to show what's going on....
  return 0x12;
}

void setup() {
  uint8_t slaveid = determine_slave_id();
  // This will only respond to requests for register requests from 0x2000->0x2003
  // but only 4 registers in memory are required.
  // Using various base addresses can help you distinguish device types on your network
  slave.setup(slaveid, 38400, MB_BASE_ADDRESS, pin_tx_enable);
}

void loop() {

  modbus_set_32(tab_reg, mb_ticks, millis());
  tab_reg[mb_delta_somevalue] += read_some_hardware();
  
  int rc = slave.loop(tab_reg, mb_register_count);
  if (rc > 0) {
    Serial.print("Received some sort of command...");
    // We perpetually accumulate from the hardware, 
    // but only reset when it is read out from us.
    tab_reg[mb_delta_somevalue] = 0;
  }

}

