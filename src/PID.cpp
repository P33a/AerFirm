/* Copyright (c) 2019  Paulo Costa
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
#include "PID.h"

PID_t::PID_t()
{
  Se = 0;
  stress = 0;
  lambda = 0.9;
  // Some typical values
  dt = 0.1;
  Kp = 1;
  Ki = 0;
  Kd = 0;
  Kf = 0;
  
  m_max =  12;
  m_min = -12;
}


float PID_t::calc(float new_y_ref, float new_y)
{
  float de, tmpSe;
  
  y_ref = new_y_ref;
  y = new_y;

  e = y_ref - y;
  
  // derivative of the error
  de = (e - last_e) / dt;
  last_e = e;
  
  // Integral of the error
  tmpSe = Se + e * dt;

  // Remove integration for zero reference
  if (y_ref == 0) {
    Se = 0;
  }
  
  // Calc PID output
  m = Kp * e + Ki * tmpSe + Kd * de + Kf * y_ref;

  // Anti windup
  if (((m > m_max) && tmpSe > 0) || ((m < m_min) && (tmpSe < 0))) {
    // remove extra integral component
    m = m + Ki * (Se - tmpSe);  
  } else {
    // OK: integrate the error
    Se = tmpSe;
  }

  // The stress is the integral of the absolute error
  // with exponential forgetting
  stress = stress * lambda + fabs(e) * dt;

  // Saturate the output
  if (m > m_max) {
    m = m_max;
  } else if (m < m_min) {
    m = m_min;
  }

  return m;
}
