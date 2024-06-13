#include "ArduinoModbusRtuServerLite.h"
// #include "ArduinoModbus.h"
#define RS485_RX  PA3
#define RS485_TX  PA2
#define RS485_RE  PA0
#define RS485_TE  PA1

RS485Class RS485BUS(Serial1, RS485_TE, RS485_RE);
ModbusRTUServerLite ModbusRTU(RS485BUS);

void OutputPins(){
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
}

int counter = 0;
void setup() {
  OutputPins();
  ModbusRTU.begin(100, 115200);
  ModbusRTU.configureCoils(0x00, 64);
  ModbusRTU.configureHoldingRegisters(0, 20);
}

void loop() {
  int packetReceived = ModbusRTU.poll();
  if(packetReceived){
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_1);
    uint16_t val = ModbusRTU.holdingRegisterRead(0);
    ModbusRTU.holdingRegisterWrite(0, val + 1);
    ModbusRTU.holdingRegisterWrite(1, val + 2);
    ModbusRTU.holdingRegisterWrite(2, val + 3);
    ModbusRTU.holdingRegisterWrite(3, val + 4);
  }
}
