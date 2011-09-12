#include <Modbusino.h>

// Example of modbusino compatible with the jpmozeta arduino modbus slave example.

ModbusinoSlave modbusino_slave(0x69);
uint16_t tab_reg[10];


// Yes, this is important...
HardwareSerial modbus_uart = HardwareSerial();
int ledPin = 11;
int tx_enable_pin = 6;  // connected to rs485 transceiver
unsigned long wdog = 0;         /* watchdog */
unsigned long tprev = 0;         /* previous time*/

/* slave registers */
enum {        
        MB_CTRL,        /* Led control on, off or blink */
        MB_TIME,        /* blink time in milliseconds */
        MB_CNT,        /* count the number of blinks */
        MB_REGS	 	/* total number of registers on slave */
};

void setup() {
    modbusino_slave.setup(tx_enable_pin, 38400);
    pinMode(ledPin, OUTPUT);
    Serial.begin(57600);
}

long ktime = 0;
void loop() {
    if (ktime + 2000 < millis()) {
      ktime = millis();
      Serial.println("still alive..."); 
    }
    
    int rc = modbusino_slave.loop(tab_reg, 10);
    if ((rc > 0) && (rc < 240)) {
      Serial.print("loop got positive rc=");Serial.println(rc, DEC);
      wdog = millis();
    }
    
    // no comms in 5 seconds, turn off the led
    if ((wdog + 5000) < millis()) {
      tab_reg[MB_CTRL] = 0;
    }
    
    switch (tab_reg[MB_CTRL]) {
    case 0: digitalWrite(ledPin, LOW);
            break;
    case 1: digitalWrite(ledPin, HIGH);
            break;
    default:
            if (millis() - tprev > tab_reg[MB_TIME]) {
                if (LOW == digitalRead(ledPin)) {
                      digitalWrite(ledPin, HIGH);
                      // return some values to the master..
                      tab_reg[MB_CNT]++;
                } else {
                      digitalWrite(ledPin, LOW);
                }
                tprev = millis();
            }
    }
}
