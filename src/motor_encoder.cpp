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
#include "motor_encoder.h"

/*
const int M1_ENABLE_pin = 2; // disables both outputs of motor when LOW

const int M2_ENABLE_pin = 4; // disables both outputs of motor when LOW

const int M1_DIAG_pin = 6;   // driven low when certain faults have occurred or when the driver is disabled by the EN or ENB inputs
const int M1_DIR_pin = 7;    // Motor 1 direction input
const int M2_DIR_pin = 8;    // Motor 2 direction input
const int M1_PWM_pin = 9;    // Motor 1 PWM input
const int M2_PWM_pin = 10;   // Motor 2 PWM input
const int M2_DIAG_pin = 12;  // driven low when certain faults have occurred or when the driver is disabled by the EN or ENB inputs

const int M1_FB_pin = A0;  // Motor 1 current sense output (approx. 525 mV/A)
const int M2_FB_pin = A1;  // Motor 2 current sense output (approx. 525 mV/A)

const int dir_pin = M1_DIR_pin;
const int PWM_pin = M1_PWM_pin;

static void set_M1_voltage(int new_voltage)
{
  if (new_voltage >= 0) {
    M1_PWM_value = new_voltage;
    digitalWrite(M1_DIR_pin, 0);
  } else {
    M1_PWM_value = -new_voltage;
    digitalWrite(M1_DIR_pin, 1);
  }
  Timer1.setPwmDuty(TIMER1_A_PIN, M1_PWM_value);
}

static void set_M2_voltage(int new_voltage)
{
  if (new_voltage >= 0) {
    M2_PWM_value = new_voltage;
    digitalWrite(M2_DIR_pin, 0);
  } else {
    M2_PWM_value = -new_voltage;
    digitalWrite(M2_DIR_pin, 1);
  }
  Timer1.setPwmDuty(TIMER1_B_PIN, M2_PWM_value);
}

static void set_motor_voltage(uint8_t motor_num, int new_voltage)
{
  if (motor_num == 1) set_M1_voltage(new_voltage);
  else if (motor_num == 2) set_M2_voltage(new_voltage);
}
*/

static byte encoder_state;
static volatile int16_t encoder_odo;

void timer_interrupt(void)
{
  byte new_state;
  static int8_t encoder_table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

  // Put encoder 1 channels in the lowest bits
  //new_state = (digitalRead(encoder1A_pin) ? (1 << 1) : 0) | (digitalRead(encoder1B_pin) ? 1 : 0);
  new_state = ((digitalRead(encoder1A_pin) << 1)  | digitalRead(encoder1B_pin));
  encoder_odo += encoder_table[encoder_state | new_state];
  encoder_state = new_state << 2;
}


motorTB9051_t::motorTB9051_t()
{
  v_max = 12;
  PWM_max = 1024;
}

void motorTB9051_t::init(uint32_t usecs)
{
  pinMode(dir_pin, OUTPUT);
  pinMode(PWM_pin, OUTPUT);
  pinMode(M1_ENABLE_pin, OUTPUT);

  pinMode(encoder1A_pin, INPUT_PULLUP);
  pinMode(encoder1B_pin, INPUT_PULLUP);

  Timer1.attachInterrupt(timer_interrupt); 
  Timer1.initialize(usecs); // Calls every usecs (microseconds)
  Timer1.pwm(TIMER1_A_PIN, 0); 
  Timer1.pwm(TIMER1_B_PIN, 0); 
  digitalWrite(M1_DIR_pin, 0);
  digitalWrite(enable_pin, 1);
}

void motorTB9051_t::update_encoder(void)
{
  cli();
    odo = encoder_odo;
    encoder_odo = 0;
  sei();
  encoder_pos += odo; 
}


void motorTB9051_t::set_PWM(int new_PWM, byte mot_enable)
{
  //new_PWM = 4 * new_PWM; // adaptation to pwm values from -255 to 255
  
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  PWM_value = new_PWM;

  
  if (mot_enable) {
    digitalWrite(enable_pin, 1);
    if (new_PWM >= 0) {
      digitalWrite(dir_pin, 0);
      Timer1.setPwmDuty(TIMER1_A_PIN, new_PWM);
      Timer1.setPwmDuty(TIMER1_B_PIN, new_PWM);
    } else {
      digitalWrite(dir_pin, 1);
      Timer1.setPwmDuty(TIMER1_A_PIN, -new_PWM);
      Timer1.setPwmDuty(TIMER1_B_PIN, PWM_max + new_PWM);
    }
  } else {
    digitalWrite(dir_pin, 0);
    digitalWrite(enable_pin, 0);
  }
}

void motorTB9051_t::set_voltage(float new_voltage)
{
  v = new_voltage;
  set_PWM((v / v_max) * PWM_max, 1);
}

void motorTB9051_t::add_voltage(float add_voltage)
{
  v += add_voltage;
  set_PWM((v / v_max) * PWM_max, 1);
}



motorMC33926_t::motorMC33926_t()
{
  v_max = 12;
  PWM_max = 1024;
}

void motorMC33926_t::init(uint32_t usecs)
{
  pinMode(dir_pin, OUTPUT);
  pinMode(PWM_pin, OUTPUT);
  pinMode(nD2_pin, OUTPUT);

  pinMode(encoder1A_pin, INPUT_PULLUP);
  pinMode(encoder1B_pin, INPUT_PULLUP);

  Timer1.attachInterrupt(timer_interrupt); 
  Timer1.initialize(usecs); // Calls every usecs (microseconds)
  Timer1.pwm(TIMER1_A_PIN, 0); 
  digitalWrite(nD2_pin, 1);
}

void motorMC33926_t::update_encoder(void)
{
  cli();
    odo = encoder_odo;
    encoder_odo = 0;
  sei();
  encoder_pos += odo; 
}


void motorMC33926_t::set_PWM(int new_PWM, byte mot_enable)
{
  //new_PWM = 4 * new_PWM; // adaptation to pwm values from -255 to 255
  
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  PWM_value = new_PWM;

  
  if (mot_enable) {
    digitalWrite(nD2_pin, 1);
    if (new_PWM >= 0) {
      digitalWrite(M1_DIR_pin, 0);
      Timer1.setPwmDuty(TIMER1_A_PIN, new_PWM);
      Timer1.setPwmDuty(TIMER1_B_PIN, new_PWM);
    } else {
      digitalWrite(M1_DIR_pin, 1);
      Timer1.setPwmDuty(TIMER1_A_PIN, -new_PWM);
      Timer1.setPwmDuty(TIMER1_B_PIN, PWM_max + new_PWM);
    }
  } else {
    digitalWrite(dir_pin, 0);
    digitalWrite(nD2_pin, 0);
  }
}

void motorMC33926_t::set_voltage(float new_voltage)
{
  v = new_voltage;
  set_PWM((v / v_max) * 255, 1);
}

void motorMC33926_t::add_voltage(float add_voltage)
{
  v += add_voltage;
  set_PWM((v / v_max) * 255, 1);
}




motorL298_t::motorL298_t()
{
  v_max = 12;
}

void motorL298_t::init(uint32_t usecs)
{

  Timer1.attachInterrupt(timer_interrupt); 
  Timer1.initialize(usecs); 
  
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, 0);
  pinMode(PWM_pin, OUTPUT);
  analogWrite(PWM_pin, 0);   
}

void motorL298_t::update_encoder(void)
{
  cli();
  odo = encoder_odo;
  encoder_odo = 0;
  sei();
  encoder_pos += odo; 
}


void motorL298_t::set_PWM(int new_PWM, byte mot_enable)
{
  if (new_PWM >  255) new_PWM =  255;
  if (new_PWM < -255) new_PWM = -255;
  PWM_value = new_PWM;

  if (mot_enable) {
    if (new_PWM > 0) {
      digitalWrite(dir_pin, 0);
      analogWrite(PWM_pin, new_PWM); 
    } else {
      digitalWrite(dir_pin, 1);
      analogWrite(PWM_pin, abs(new_PWM)); 
    }
  } else {
    digitalWrite(dir_pin, 0);
    analogWrite(PWM_pin, 0); 
  }
}

void motorL298_t::set_voltage(float new_voltage)
{
  v = new_voltage;
  set_PWM((v / v_max) * 255, 1);
}

void motorL298_t::add_voltage(float add_voltage)
{
  v += add_voltage;
  set_PWM((v / v_max) * 255, 1);
}
