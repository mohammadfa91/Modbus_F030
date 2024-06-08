#include "ArduinoRS485.h"
#include "ArduinoModbus.h"
#define RS485_RX  PA3
#define RS485_TX  PA2
#define RS485_RE  PA0
#define RS485_TE  PA1

HardwareSerial MBSerial(RS485_RX, RS485_TX);
RS485Class RS485BUS(MBSerial, RS485_TX, RS485_TE, RS485_RE);
ModbusRTUServerClass ModbusRTU(RS485BUS);
int counter = 0;
void setup() {
  ModbusRTU.begin(100, 115200);
  ModbusRTU.configureCoils(0x00, 1);
  ModbusRTU.configureHoldingRegisters(0, 4);
}

void loop() {
  int packetReceived = ModbusRTU.poll();
  if(packetReceived) {
    int coilValue = ModbusRTU.coilRead(0x00);
  
    if (coilValue) {
      ModbusRTU.holdingRegisterWrite(0, counter);
      counter ++;
    } else {
      counter --;
    }
      }
}