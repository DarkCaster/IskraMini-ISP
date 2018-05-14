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

#ifndef MICROSPI_H_INCLUDED
#define MICROSPI_H_INCLUDED

#include <Arduino.h>

class MicroSPI
{
  private:
    const uint8_t pinMOSI;
    const uint8_t pinMISO;
    const uint8_t pinSCLK;
    volatile uint8_t* const portMOSI;
    volatile uint8_t* const portMISO;
    volatile uint8_t* const portSCLK;
    const uint8_t maskMOSI_H;
    const uint8_t maskMISO;
    const uint8_t maskSCLK_H;
    const uint8_t maskMOSI_L;
    const uint8_t maskSCLK_L;
  public:
    MicroSPI(const uint8_t pinMOSI, const uint8_t pinMISO, const uint8_t pinSCLK);
    void ResetPins() const;
    uint8_t TransferByte(uint8_t value) const;
};

#endif
