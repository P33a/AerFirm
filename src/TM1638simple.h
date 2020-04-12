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

#ifndef TM1638SIMPLE_H
#define TM1638SIMPLE_H

#include "Arduino.h"

#define TM1638_ACTIVATE_CMD 0x88  // Activate board (bit a) and set brightness (bits b) (1000abbb)
#define TM1638_BUTTONS_CMD 0x42   // read buttons
#define TM1638_WRITE_CMD 0x44     // Write value 0xYZ at location 0xcW  ARGS: 0xcW 0xYZ
#define TM1638_WRITE_INC_CMD 0x40 // Write value 0xYZ at location 0xcW and increment ARGS: 0xcW 0xYZ ...
#define TM1638_BASE_ADR 0xC0 
#define TM1638_BRIGHT_ADR 0x88    // Brightness address
#define TM1638_LEDS_OFFSET 1

// Address	Description
// 0x00 display #1
// 0x01 LED #1 00000001 – red, 00000010 – green
// 0x02 display #2
// 0x03 LED #2 00000001 – red, 00000010 – green
// 0x04 display #3
// 0x05 LED #3 00000001 – red, 00000010 – green
// 0x06 display #4
// 0x07 LED #4 00000001 – red, 00000010 – green
// 0x08 display #5
// 0x09 LED #5 00000001 – red, 00000010 – green
// 0x0a display #6
// 0x0b LED #6 00000001 – red, 00000010 – green
// 0x0c display #7
// 0x0d LED #7 00000001 – red, 00000010 – green
// 0x0e display #8
// 0x0f LED #8 00000001 – red, 00000010 – green

class TM1638simple_t  {
  private:
	uint8_t strobe_pin;
	uint8_t clock_pin;
	uint8_t data_pin;

  public:
	TM1638simple_t(uint8_t set_strobe_pin, uint8_t set_clock_pin, uint8_t set_data_pin);
	
	void init(void);
    void send(uint8_t value);
	void clear(void);

	void setBrightness(uint8_t brightness);

	void writeSegment(uint8_t pos, uint8_t value);
	void writeChar(uint8_t pos, uint8_t c);
	void writeCharDot(uint8_t pos, uint8_t c);
	void writeString(const char *s);

	void writeLED(uint8_t pos, uint8_t value);
	uint8_t readButtons(void);

};

// Bits for segments
//   0
// 5   1
//   6
// 4   2
//   3
//      7

const PROGMEM uint8_t TM1638_char_set[] = {
  0b00000000, // space
  0b10000110, // ! 
  0b00100010, // " 
  0b01111110, // # 
  0b01101101, // 0b 
  0b11010010, // % 
  0b01000110, // & 
  0b00100000, // ' 
  0b00101001, // ( 
  0b00001011, // ) 
  0b00100001, // * 
  0b01110000, // + 
  0b00010000, // , 
  0b01000000, // - 
  0b10000000, // . 
  0b01010010, // / 
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b00001001, // : 
  0b00001101, // ; 
  0b01100001, // < 
  0b01001000, // = 
  0b01000011, // > 
  0b11010011, // ? 
  0b01011111, // @ 
  0b01110111, // A
  0b01111100, // B
  0b00111001, // C
  0b01011110, // D
  0b01111001, // E
  0b01110001, // F
  0b00111101, // G
  0b01110110, // H
  0b00110000, // I
  0b00011110, // J
  0b01110101, // K
  0b00111000, // L
  0b00010101, // M
  0b00110111, // N
  0b00111111, // O
  0b01110011, // P
  0b01101011, // Q
  0b00110011, // R
  0b01101101, // S
  0b01111000, // T
  0b00111110, // U 
  0b00111110, // V 
  0b01111110, // W 
  0b01110110, // X 
  0b01101110, // Y 
  0b01011011, // Z 
  0b00111001, // [ 
  0b01100100, // Backslash 
  0b00001111, // ] 
  0b00100011, // ^ 
  0b00001000, // _ 
  0b00000010, // ` 
  0b01011111, // a 
  0b01111100, // b 
  0b01011000, // c 
  0b01011110, // d 
  0b01111011, // e 
  0b01110001, // f 
  0b01101111, // g 
  0b01110100, // h 
  0b00010000, // i 
  0b00001100, // j 
  0b01110101, // k 
  0b00110000, // l 
  0b00010100, // m 
  0b01010100, // n 
  0b01011100, // o 
  0b01110011, // p 
  0b01100111, // q 
  0b01010000, // r 
  0b01101101, // s 
  0b01111000, // t 
  0b00011100, // u 
  0b00011100, // v 
  0b00010100, // w 
  0b01110110, // x 
  0b01101110, // y 
  0b01011011, // z 
  0b01000110, // { 
  0b00110000, // | 
  0b01110000, // } 
  0b00000001, // ~ 
};

#endif // TM1638SIMPLE_H
