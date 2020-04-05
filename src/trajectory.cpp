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
#include "trajectory.h"

trajectory_t::trajectory_t()
{
  // Some typical values
  m_i = 0;
  m_f = 0;

}

void trajectory_t::set(float initial_pos, float initial_speed, float final_pos, float final_speed, float final_time)
{
  t = 0;
  pos_i = initial_pos;
  m_i = initial_speed;
  pos_f = final_pos;
  m_f = final_speed;
  if (final_time > 0) t_f = final_time;
  else t_f = -1;
}

float trajectory_t::get_position(float t_act)
{
  if (t_f <= 0) return 0;

  float tc = t_act / t_f;
  
  if (tc < 1) {
    float tc2 = tc * tc;
    float tc3 = tc2 * tc;
    return (2 * tc3 - 3 * tc2 + 1) * pos_i + 
           (tc3 - 2 * tc2 + tc) * t_f * m_i +
           (-2 * tc3 + 3 * tc2) * pos_f +
           (tc3 - tc2) * t_f * m_f;
    //return h00(tc) * pos_i + h01(tc) * pos_f;
  } else {
    return pos_f;
  }
}


float trajectory_t::get_speed(float t_act)
{
  if (t_f <= 0) return 0;

  float tc = t_act / t_f;

  if (tc < 1) {
    //return (6 * tc * (t - 1) * pos_i + 6 * tc * (1 - tc) * pos_f) / t_f;
    //return (6 * tc * (tc - 1) * (pos_i - pos_f)) / t_f;
    float tc2 = tc * tc;
    return ((6 * tc2 - 6 * tc) * pos_i + 
            (3 * tc2 - 4 * tc + 1) * t_f * m_i + 
            (-6 * tc2 + 6 * tc) * pos_f + 
            (3 * tc2 - 2 * tc) * t_f * m_f) / t_f;
    
  } else return 0;   
}


void trajectory_t::show_values(float dt, float final_t, byte digits)
{
  float t = 0;
  while(t <= final_t) {
    Serial.print(t, digits);
    Serial.print(" ");
    Serial.print(get_speed(t), digits);
    Serial.print(" ");
    Serial.print(get_position(t), digits);
    Serial.println();
    t = t + dt;
  }
}
