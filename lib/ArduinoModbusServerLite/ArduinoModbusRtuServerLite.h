#ifndef __ARDUINOMODBUSSERVERLITE__
#define __ARDUINOMODBUSSERVERLITE__
#include "ArduinoRS485Lite.h"
class ModbusRTUServerLite {
public:
  ModbusRTUServerLite();
  ModbusRTUServerLite(RS485Class& rs485);
  virtual ~ModbusRTUServerLite();
  void begin(int id, unsigned long baudrate, uint16_t config = SERIAL_8N1);
  void begin(RS485Class& rs485, int id, unsigned long baudrate, uint16_t config = SERIAL_8N1);
  void configureCoils(int startAddress, int nb);
  void configureHoldingRegisters(int startAddress, int nb);
  void configureInputRegisters(int startAddress, int nb);
  void configureDiscreteInputs(int startAddress, int nb);
  long inputRegisterRead(int address);
  int inputRegisterWrite(int address, uint16_t value);
  long holdingRegisterRead(int address);
  int holdingRegisterWrite(int address, uint16_t value);
  long discreteInputsRead(int address);
  int discreteInputsWrite(int address, uint8_t value);
  long coilRead(int address);
  int coilWrite(int address, uint8_t value);
  void analyzeInput(uint8_t* buffer, uint8_t size);
  int poll();

private:
  RS485Class* _rs485;
  int _id;
  uint32_t lastrec = 0;
  uint8_t buffer[128];
  uint8_t tx_buffer[128];
  uint8_t index = 0;
  unsigned long previousMillis = 0;
  uint8_t coils_address;
  uint8_t coils_quantity;
  uint8_t *coil_buffer;
  uint8_t holding_registers_address;
  uint8_t holding_registers_quantity;
  uint16_t* holding_registers_buffer;
  uint8_t discrete_input_address;
  uint8_t discrete_input_quantity;
  uint8_t *discrete_input_buffer;
  uint8_t input_registers_address;
  uint8_t input_registers_quantity;
  uint16_t *input_registers_buffer;
};

#endif