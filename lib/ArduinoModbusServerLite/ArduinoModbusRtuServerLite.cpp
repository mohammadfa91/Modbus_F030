#include "ArduinoModbusRtuServerLite.h"
#define READ_COILS              1
#define READ_DISCRETE_INPUT     2
#define READ_HOLDING_REGISTER   3
#define READ_INPUT_REGISTER     4
#define WRITE_SINGLE_COIL       5
#define WRITE_SINGLE_REGISTER   6
#define WRITE_MULTI_COIL        15
#define WRITE_MULTIR_EGISTER    16

typedef enum{
  NoError,
  IllegalFunction,
  IllegalDataAddress,
  IllegalDataValue,
  Acknowledge,
  SlaveDeviceFailure,
  SlaveDeviceBusy,
  NegativeAcknowledge,
  MemoryParityError
}ExceptionCode;

uint8_t ErrorFunction(uint8_t Func){
  switch (Func)
  {
    case READ_COILS:
      return 0x81;
    case READ_DISCRETE_INPUT:
      return 0x82;
    case READ_HOLDING_REGISTER:
      return 0x83;
    case READ_INPUT_REGISTER:
      return 0x84;
    case WRITE_SINGLE_COIL:
      return 0x85;
    case WRITE_SINGLE_REGISTER:
      return 0x86;
    case WRITE_MULTI_COIL:
      return 0x8F;
    case WRITE_MULTIR_EGISTER:
      return 0x90;
    default:
      return 0;
  }
}
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */
    // uint16_t length = buffer_length;
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }
    // buffer[length] = crc_hi;
    // buffer[length + 1] = crc_lo;
    return (crc_hi << 8 | crc_lo);
}
ModbusRTUServerLite::ModbusRTUServerLite()
{
}

ModbusRTUServerLite::ModbusRTUServerLite(RS485Class& rs485) : _rs485(&rs485)
{
}

ModbusRTUServerLite::~ModbusRTUServerLite()
{
}


void ModbusRTUServerLite::begin(int id, unsigned long baudrate, uint16_t config)
{
  _id = id;
  _rs485->setTimeout(10);
  _rs485->begin(baudrate, config);
}

void ModbusRTUServerLite::begin(RS485Class& rs485, int id, unsigned long baudrate, uint16_t config)
{
  _rs485 = &rs485;
  begin(id, baudrate, config);
}

long ModbusRTUServerLite::holdingRegisterRead(int address)
{
  if (holding_registers_address > address || (holding_registers_address + holding_registers_quantity) < (address + 1))
    return -1;
  return holding_registers_buffer[address - holding_registers_address];
}
int ModbusRTUServerLite::holdingRegisterWrite(int address, uint16_t value){
  if (holding_registers_address > address || (holding_registers_address + holding_registers_quantity) < (address + 1))
    return 0;
  holding_registers_buffer[address - holding_registers_address] = value;
  return 1;
}
long ModbusRTUServerLite::inputRegisterRead(int address)
{
  if (input_registers_address > address || (input_registers_address + input_registers_quantity) < (address + 1))
    return -1;
  return input_registers_buffer[address - input_registers_address];
}
int ModbusRTUServerLite::inputRegisterWrite(int address, uint16_t value){
  if (input_registers_address > address || (input_registers_address + input_registers_quantity) < (address + 1))
    return 0;
  input_registers_buffer[address - input_registers_address] = value;
  return 1;
}
long ModbusRTUServerLite::discreteInputsRead(int address)
{
  if (discrete_input_address > address || (discrete_input_address + discrete_input_quantity) < (address + 1))
    return -1;
  return discrete_input_buffer[address - discrete_input_address];
}
int ModbusRTUServerLite::discreteInputsWrite(int address, uint8_t value){
  if (discrete_input_address > address || (discrete_input_address + discrete_input_quantity) < (address + 1))
    return 0;
  discrete_input_buffer[address - discrete_input_address] = value;
  return 1;
}
long ModbusRTUServerLite::coilRead(int address)
{
  if (coils_address > address || (coils_address + coils_quantity) < (address + 1))
    return -1;
  return coil_buffer[address - coils_address];
}
int ModbusRTUServerLite::coilWrite(int address, uint8_t value){
  if (coils_address > address || (coils_address + coils_quantity) < (address + 1))
    return 0;
  coil_buffer[address - coils_address] = value;
  return 1;
}
void ModbusRTUServerLite::analyzeInput(uint8_t* buffer, uint8_t sizeOfinput){
  if(sizeOfinput < 8) return;
  if(buffer[0] != _id) return;
  uint8_t Function = buffer[1];
  uint16_t StartAddress = buffer[2] << 8 | buffer[3];
  uint16_t NubmerOfRequested = buffer[4] << 8 | buffer[5];
  tx_buffer[0] = buffer[0];
  ExceptionCode exception = NoError;
  uint8_t size = 3;

  if(Function == READ_COILS){
    if(StartAddress < coils_address)
      exception = IllegalDataAddress;
    else if(coils_address + coils_quantity < StartAddress + NubmerOfRequested)
      exception = IllegalDataAddress;
    else{
      uint8_t bytenum = 0;
      for(int i=0; i<NubmerOfRequested; i++){
        if(i % 8 == 0){
          size++;
          tx_buffer[2]++;
        }
        uint8_t val = coil_buffer[(StartAddress - coils_address) + i];
        if(val > 0) val = 1;
        tx_buffer[size-1] |= val << (i % 8);
      }
    }
  }
  else if(Function == READ_DISCRETE_INPUT){
    if(StartAddress < discrete_input_address)
      exception = IllegalDataAddress;
    else if(discrete_input_address + discrete_input_quantity < StartAddress + NubmerOfRequested)
      exception = IllegalDataAddress;
    else{
      uint8_t bytenum = 0;
      for(int i=0; i<NubmerOfRequested; i++){
        if(i % 8 == 0){
          size++;
          tx_buffer[2]++;
        }
        uint8_t val = discrete_input_buffer[(StartAddress - discrete_input_address) + i];
        if(val > 0) val = 1;
        tx_buffer[size-1] |= val << (i % 8);
      }
    }
  }
  else if(Function == READ_HOLDING_REGISTER){
    if(StartAddress < holding_registers_address)
      exception = IllegalDataAddress;
    else if(holding_registers_address + holding_registers_quantity < StartAddress + NubmerOfRequested)
      exception = IllegalDataAddress;
    else{
      tx_buffer[2] = NubmerOfRequested * 2;
      for(int i=0; i<NubmerOfRequested; i++){
        uint16_t reg  = holding_registers_buffer[(StartAddress - holding_registers_address) + i];
        tx_buffer[size] = reg >> 8;
        size++;
        tx_buffer[size] = reg & 0xFF;
        size++;
      }
    }
  }
  else if(Function == READ_INPUT_REGISTER){
    if(StartAddress < input_registers_address)
      exception = IllegalDataAddress;
    else if(input_registers_address + input_registers_quantity < StartAddress + NubmerOfRequested)
      exception = IllegalDataAddress;
    else{
      tx_buffer[2] = NubmerOfRequested * 2;
      for(int i=0; i<NubmerOfRequested; i++){
        uint16_t reg  = input_registers_buffer[(StartAddress - input_registers_address) + i];
        tx_buffer[size] = reg >> 8;
        size++;
        tx_buffer[size] = reg & 0xFF;
        size++;
      }
    }
  }
  else if(Function == WRITE_SINGLE_COIL){
    if(StartAddress < coils_address)
      exception = IllegalDataAddress;
    else if(coils_address + coils_quantity < StartAddress)
      exception = IllegalDataAddress;
    else{
      uint16_t Command = NubmerOfRequested;
      if(Command == 0xFF00)
        coil_buffer[StartAddress - coils_address] = 1;
      else if(Command == 0x0000)
        coil_buffer[StartAddress - coils_address] = 0;
      tx_buffer[1] = buffer[1];
      tx_buffer[2] = buffer[2];
      tx_buffer[3] = buffer[3];
      tx_buffer[4] = buffer[4];
      tx_buffer[5] = buffer[5];
      size = 6;
    }
  }
  else if(Function == WRITE_SINGLE_REGISTER){
    if(StartAddress < holding_registers_address)
      exception = IllegalDataAddress;
    else if(holding_registers_address + holding_registers_quantity < StartAddress)
      exception = IllegalDataAddress;
    else{
      uint16_t Value = NubmerOfRequested;
      holding_registers_buffer[StartAddress - holding_registers_address] = Value;
      tx_buffer[1] = buffer[1];
      tx_buffer[2] = buffer[2];
      tx_buffer[3] = buffer[3];
      tx_buffer[4] = buffer[4];
      tx_buffer[5] = buffer[5];
      size = 6;
    }
  }
  else if(Function == WRITE_MULTI_COIL){
    if(StartAddress < coils_address)
      exception = IllegalDataAddress;
    else if(coils_address + coils_quantity < StartAddress + NubmerOfRequested)
      exception = IllegalDataAddress;
    else if(sizeOfinput < 10)
      exception = IllegalDataValue;
    else{
      uint8_t byteCount = buffer[6];
      uint32_t Data = 0;
      tx_buffer[1] = buffer[1];
      tx_buffer[2] = buffer[2];
      tx_buffer[3] = buffer[3];
      tx_buffer[4] = buffer[4];
      tx_buffer[5] = buffer[5];
      size = 6;
      for (int i = byteCount - 1; i >= 0; i--) {
          Data <<= 8;
          Data |= buffer[7 + i];
      }
      for(int i=0; i<NubmerOfRequested; i++){
        coil_buffer[StartAddress - coils_address + i] = (Data & (1 << i)) != 0;
      }
    }
  }
  else if(Function == WRITE_MULTIR_EGISTER){
    if(StartAddress < holding_registers_address)
      exception = IllegalDataAddress;
    else if(holding_registers_address + holding_registers_quantity < StartAddress + NubmerOfRequested)
      exception = IllegalDataAddress;
    else if(sizeOfinput < 10)
      exception = IllegalDataValue;
    else{
      tx_buffer[1] = buffer[1];
      tx_buffer[2] = buffer[2];
      tx_buffer[3] = buffer[3];
      tx_buffer[4] = buffer[4];
      tx_buffer[5] = buffer[5];
      size = 6;
      for(int i=0; i<NubmerOfRequested; i++){
        holding_registers_buffer[StartAddress - holding_registers_address + i] = buffer[7+2*i]<<8 | buffer[8+2*i];
      }
    }
  }
  else{
    exception = IllegalFunction;
  }
  //////////////// Conclusion ////////////////
  if(exception != NoError){
    tx_buffer[1] = ErrorFunction(Function);
    tx_buffer[2] = exception;
  }
  else{
    tx_buffer[1] = buffer[1];
  }

  uint16_t crc = crc16(tx_buffer, size);
  tx_buffer[size] = crc >>8;
  tx_buffer[size + 1] = crc & 0xFF;
  delay(1);
  _rs485->beginTransmission();
  for(int i=0; i< size + 2; i++)
    _rs485->write(tx_buffer[i]);
  delay(1);
  _rs485->endTransmission();
  memset(tx_buffer, 0, sizeof(tx_buffer));
}

int ModbusRTUServerLite::poll()
{
  unsigned long currentMillis = millis();
  if (_rs485->available() > 0) {
    char incomingChar = _rs485->read();
    buffer[index] = incomingChar;
    index++;
    previousMillis = currentMillis;
  }
  if (currentMillis - previousMillis >= 10 && index > 0) {
    if (index > 0) {
      buffer[index] = '\0';
      analyzeInput(buffer, index);
    }
    memset(buffer, 0, sizeof(buffer));
    index = 0;
    return 1;
  }
  return 0;
}

void ModbusRTUServerLite::configureDiscreteInputs(int startAddress, int nb){
  discrete_input_address = startAddress;
  discrete_input_quantity = nb;
  discrete_input_buffer = (uint8_t *)malloc(nb * sizeof(uint8_t));
  for(int i = 0; i < nb; i++)
    discrete_input_buffer[i] = 0;
}
void ModbusRTUServerLite::configureCoils(int startAddress, int nb){
  coils_address = startAddress;
  coils_quantity = nb;
  coil_buffer = (uint8_t *)malloc(nb * sizeof(uint8_t));
  for(int i = 0; i < nb; i++)
    coil_buffer[i] = 0;
}
void ModbusRTUServerLite::configureHoldingRegisters(int startAddress, int nb){
  holding_registers_address = startAddress;
  holding_registers_quantity = nb;
  holding_registers_buffer = (uint16_t *)malloc(nb * sizeof(uint16_t));
  for(int i = 0; i < nb; i++)
    holding_registers_buffer[i] = 0;
  // memset(holding_registers_buffer, 0, nb);
}
void ModbusRTUServerLite::configureInputRegisters(int startAddress, int nb){
  input_registers_address = startAddress;
  input_registers_quantity = nb;
  input_registers_buffer = (uint16_t *)malloc(nb * sizeof(uint16_t));
  for(int i = 0; i < nb; i++)
    input_registers_buffer[i] = 0;
}