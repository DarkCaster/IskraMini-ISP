// Minimal software SPI implementation for use with ISP programming protocol.
// Much faster than BitBangedSPI SPI from ArduinoISP sketch.
// Only essential features for use with ISP protocol was implemented:
//  - Only Mode 0 is implemented (CPOL = 0, CPHA = 0);
//  - Slave Select signal is not defined and used;

// The 2-Clause BSD License

// Copyright (c) 2018 DarkCaster

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "MicroSPI.h"

MicroSPI::MicroSPI(const uint8_t _pinMOSI, const uint8_t _pinMISO, const uint8_t _pinSCLK, const uint8_t _HSCLK_uSec, const uint8_t _LSCLK_uSec) :
  //set SCLK low and high pulse widths
  HSCLK_uSec(_HSCLK_uSec),
  LSCLK_uSec(_LSCLK_uSec),
  //set pins' designations
  pinMOSI(_pinMOSI),
  pinMISO(_pinMISO),
  pinSCLK(_pinSCLK),
  //set port-registers' pointers
  portMOSI(portOutputRegister(digitalPinToPort(_pinMOSI))),
  portMISO(portInputRegister(digitalPinToPort(_pinMISO))),
  portSCLK(portOutputRegister(digitalPinToPort(_pinSCLK))),
  //get masks for setting\reading pins' signals
  maskMOSI_H(digitalPinToBitMask(_pinMOSI)),
  maskMISO(digitalPinToBitMask(_pinMISO)),
  maskSCLK_H(digitalPinToBitMask(_pinSCLK)),
  maskMOSI_L(~maskMOSI_H),
  maskSCLK_L(~maskSCLK_H)
{ }

void MicroSPI::ResetPins() const
{
  //reset pin modes
  pinMode(pinMOSI, OUTPUT);
  pinMode(pinMISO, INPUT);
  pinMode(pinSCLK, OUTPUT);
  //set SCLK and MOSI lines to low
  *portSCLK &= maskSCLK_L;
  *portMOSI &= maskMOSI_L;
  delayMicroseconds(LSCLK_uSec);
}

uint8_t MicroSPI::TransferByte(uint8_t value) const
{
  noInterrupts();
  uint8_t incomingByte = 0;
  uint8_t bitOffset = 8;
  do
  {
    --bitOffset;
    //set MOSI line accordingly to value's bit and set SCLK line to high
    *portMOSI = value&0x80 ? *portMOSI|maskMOSI_H : *portMOSI&maskMOSI_L;
    value <<= 1;
    *portSCLK |= maskSCLK_H;
    delayMicroseconds(HSCLK_uSec);
    //read incomingByte's bit and set SCLK line to low
    incomingByte <<= 1;
    incomingByte |= *portMISO&maskMISO ? 1 : 0;
    *portSCLK &= maskSCLK_L;
    delayMicroseconds(LSCLK_uSec);
  } while(bitOffset>0);
  interrupts();
  return incomingByte;
}

