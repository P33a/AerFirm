/* Copyright (c) 2020  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include "Arduino.h"
#include "TM1638simple.h"

TM1638simple_t::TM1638simple_t(uint8_t new_strobe_pin, uint8_t new_clock_pin, uint8_t new_data_pin) 
{
  strobe_pin = new_strobe_pin;
  clock_pin = new_clock_pin;
  data_pin = new_data_pin;
}

void TM1638simple_t::init(void)
 {
  pinMode(strobe_pin, OUTPUT);
  pinMode(clock_pin, OUTPUT);
  pinMode(data_pin, OUTPUT);
  
  send(TM1638_ACTIVATE_CMD);
  setBrightness(3);
  clear();
}

void TM1638simple_t::send(uint8_t value)
{
  digitalWrite(strobe_pin, 0);
  shiftOut(data_pin, clock_pin, LSBFIRST, value);
  digitalWrite(strobe_pin, 1);
}

void TM1638simple_t::clear(void) 
{
  send(TM1638_WRITE_INC_CMD); 
  
  digitalWrite(strobe_pin, 0);
  shiftOut(data_pin, clock_pin, LSBFIRST, TM1638_BASE_ADR);   
  for (uint8_t i = 0; i < 16; i++) {
    shiftOut(data_pin, clock_pin, LSBFIRST, 0x00);
  }
  digitalWrite(strobe_pin, 1);
}


void TM1638simple_t::setBrightness(uint8_t brightness)
{
  uint8_t value = TM1638_BRIGHT_ADR + (0x0F & brightness);
  digitalWrite(strobe_pin, 0); 
  shiftOut(data_pin, clock_pin, LSBFIRST, value);
  digitalWrite(strobe_pin, 1); 
}

void TM1638simple_t::writeSegment(uint8_t pos, uint8_t value) 
{
  send(TM1638_WRITE_CMD);
  digitalWrite(strobe_pin, 0);
  shiftOut(data_pin, clock_pin, LSBFIRST, TM1638_BASE_ADR + 2 * pos);
  shiftOut(data_pin, clock_pin, LSBFIRST, value);
  digitalWrite(strobe_pin, 1); 
}

void TM1638simple_t::writeChar(uint8_t pos, uint8_t c) 
{
  writeSegment(pos, pgm_read_byte(&TM1638_char_set[c - ' ']));
}

void TM1638simple_t::writeCharDot(uint8_t pos, uint8_t c) 
{
  writeSegment(pos, 0x80 | pgm_read_byte(&TM1638_char_set[c - ' ']));
}


void TM1638simple_t::writeString(const char *s) 
{
  uint8_t pos = 0;
  
  while(*s) {
    if (*(s + 1) == '.') {
      writeCharDot(pos, *s);  // Write the dot
      s += 2; // advance 2 chars
    } else {
      writeChar(pos, *s);
      s++;   // Advance 1 char
    }
    pos++; 
  }
}
 
 
 void TM1638simple_t::writeLED(uint8_t pos, uint8_t value)
{
  send(TM1638_WRITE_CMD);
  digitalWrite(strobe_pin, 0);
  shiftOut(data_pin, clock_pin, LSBFIRST, TM1638_BASE_ADR + TM1638_LEDS_OFFSET + 2 * pos);
  shiftOut(data_pin, clock_pin, LSBFIRST, value);
  digitalWrite(strobe_pin, 1);
}

uint8_t TM1638simple_t::readButtons(void)
{
  uint8_t buttons = 0;
  digitalWrite(strobe_pin, 0);
  shiftOut(data_pin, clock_pin, LSBFIRST, TM1638_BUTTONS_CMD);
  pinMode(data_pin, INPUT_PULLUP);  

  for (uint8_t i = 0; i < 4; i++) {
    buttons |= shiftIn(data_pin, clock_pin, LSBFIRST) << i;
  }

  pinMode(data_pin, OUTPUT);
  digitalWrite(strobe_pin, 1); 
  return buttons;
}



