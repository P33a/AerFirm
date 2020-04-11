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

#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include "Arduino.h"
#include "TimerOne.h"

const int encoder1A_pin = A3;  // Motor encoder A input
const int encoder1B_pin = A2;  // Motor encoder B input


class motor_DC_t
{
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
  const int enable_pin = 4;

  public:
    int odo;
    int32_t encoder_pos;

    int PWM_value, PWM_max;
    uint16_t timeout_count, timeout_top;

    float speed, pos;
    float speed_ref, pos_ref;
  
    float v, v_max;

    motor_DC_t();
    
    void init(uint32_t usecs);

    void update_encoder(void);

    
    void set_PWM(int new_PWM, byte mot_enable);
    void set_voltage(float new_voltage);
    void add_voltage(float add_voltage);
};



#endif // MOTOR_ENCODER
