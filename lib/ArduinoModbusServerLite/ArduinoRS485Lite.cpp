/*
  This file is part of the ArduinoRS485 library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "ArduinoRS485Lite.h"

RS485Class::RS485Class(HardwareSerial& hwSerial, uint32_t dePin, uint32_t rePin) :
  _serial(&hwSerial),
  _dePin(dePin),
  _rePin(rePin),
  _transmisionBegun(false)
{
}

void RS485Class::begin(unsigned long baudrate)
{
  begin(baudrate, SERIAL_8N1, 50, 50);
}

void RS485Class::begin(unsigned long baudrate, int predelay, int postdelay)
{
  begin(baudrate, SERIAL_8N1, predelay, postdelay);
}

void RS485Class::begin(unsigned long baudrate, uint16_t config)
{
  begin(baudrate, config, 50, 50);
}

void RS485Class::begin(unsigned long baudrate, uint16_t config, int predelay, int postdelay)
{
  _baudrate = baudrate;
  _config = config;

  // Set only if not already initialized with ::setDelays
  _predelay = _predelay == 0 ? predelay : _predelay;
  _postdelay = _postdelay == 0 ? postdelay : _postdelay;

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER &= ~GPIO_MODER_MODER0_1;
  GPIOA->MODER |= GPIO_MODER_MODER0_0;
  GPIOA->MODER &= ~GPIO_MODER_MODER1_1;
  GPIOA->MODER |= GPIO_MODER_MODER1_0;
  GPIOA->BRR = GPIO_PIN_1;
  GPIOA->BRR = GPIO_PIN_0;

  _transmisionBegun = false;

  _serial->begin(baudrate, config);
}
int RS485Class::available()
{
  return _serial->available();
}

int RS485Class::peek()
{
  return _serial->peek();
}

int RS485Class::read(void)
{
  return _serial->read();
}

void RS485Class::flush()
{
  return _serial->flush();
}

size_t RS485Class::write(uint8_t b)
{
  USART1->TDR = b;   // LOad the Data
	while (!(USART1->ISR & (1<<7)));  // Wait for TC to SET.. This indicates that the data has been transmitted
  return 1;
}

RS485Class::operator bool()
{
  return true;
}

void RS485Class::beginTransmission()
{
    GPIOA->BSRR = GPIO_PIN_0;
    GPIOA->BSRR = GPIO_PIN_1;
    if (_predelay) delayMicroseconds(_predelay);

  _transmisionBegun = true;
}

void RS485Class::endTransmission()
{
  _serial->flush();

    if (_postdelay) delayMicroseconds(_postdelay);
    GPIOA->BRR = GPIO_PIN_0;
    GPIOA->BRR = GPIO_PIN_1;

  _transmisionBegun = false;
#ifdef ESP32
  // there is a bug in ESP32 for Serial2
  while(_serial->available())
    _serial->read();
#endif
}


void RS485Class::setPins(uint32_t dePin, uint32_t rePin)
{
  _dePin = dePin;
  _rePin = rePin;
}

void RS485Class::setDelays(int predelay, int postdelay)
{
  _predelay = predelay;
  _postdelay = postdelay;
}
