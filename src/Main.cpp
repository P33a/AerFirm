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

#include <Arduino.h>

#include "dchannels.h"
#include "proj_types.h"
#include "motor_encoder.h"
#include "PID.h"
#include "trajectory.h"
#include "button.h"

#include <TM1638plus.h>

// GPIO I/O pins on the Arduino connected to strobe, clock, data,
#define  STROBE_TM 2
#define  CLOCK_TM 3
#define  DIO_TM 5

TM1638plus tm(STROBE_TM, CLOCK_TM , DIO_TM);

#define KEY_BUFFER_SIZE 8
uint8_t key_buffer[KEY_BUFFER_SIZE];
uint8_t key_index;

view_mode_t view_mode;
uint32_t last_view_mode_change;

//LowBPM BMPRes

// A0 <- (Motor Current)
// A1 <- (MotorB Current)
// A2 <- ENC1_B
// A3 <- ENC1_A
// A4 <- 
// A5 <- 

//  2 ->  TM1638 Strobe 
//  3 ->  TM1638 Clock
//  4 ->  Motor Enable (Tri-state disables both outputs of both motor channels when LOW)
//  5 <-> TM1638 DIO 
//  6 ->  Buzzer
//  7 ->  Motor Dir
//  8 <- 
//  9 ->  Motor PWM
// 10 <-  Motor PWM_alt
// 11 <-  End switch
// 12 ->  (Status flag indicator (LOW indicates fault))
// 13 ->  LED

// Metal Gearmotor 12VDC with 64 CPR Encoder xxx rpm
// Color  - Function

// Black  - motor power
// Red    - motor power
// Blue   - Hall sensor Vcc (5 V)
// Green  - Hall sensor GND
// Yellow - Hall sensor A output
// White  - Hall sensor B output

const int end_switch_A_pin = 11;  //End switch (Normaly Open) input
const int pressure_pin = A5;

const int buzzer_pin = 6;

button_t button_bmp_up, 
         button_bmp_down, 
         button_start, 
         button_stop, 
         button_volume_up, 
         button_volume_down,
         button_ie_up, 
         button_ie_down;

byte mode_button;

dchannels_t serial_channels;

byte end_switch_A, prev_end_switch_A; 

motor_DC_t motor;

PID_t PID_speed, PID_pos;

#define NUM_PARS 16

float Pars[NUM_PARS];
uint8_t send_par_index;

#define SAVE_BUFFER_SIZE (4 + NUM_PARS * sizeof(float))

uint8_t save_buffer[SAVE_BUFFER_SIZE];
int save_par_byte;
uint8_t save_requested;

#define bmp_par Pars[7]
#define volume_par Pars[8]
#define ie_ratio_par Pars[9]
#define ticks_to_m Pars[10]

float dt;

//float ticks_to_m;

trajectory_t trajectory;

// Pumping State machine
pump_fsm_t pump;
pump_cycle_t pump_cycle;

const int led_pin =  13;
uint8_t led_state = 0;         

unsigned long last_micros = 0;        // last control time
unsigned long interval_us;            // Control period in microseconds

float manual_motor_voltage;

int buzzer_beep;

void pump_set_state(byte new_state);
void save_prepare(void);
void save_pars_to_eeprom(void);
uint8_t read_pars_from_eeprom(void);
void fill_sane_pars(void);
void motor_manual_action(void);
void normal_key_actions(void);
void alt1_key_actions(void);

void set_interval(uint32_t new_interval_us)
{
  interval_us = new_interval_us;
  dt = interval_us * 1e-6;
  PID_speed.dt = dt;
  PID_pos.dt = dt;
}

uint32_t crc32c(uint8_t* data, uint8_t num_bytes)
{
  uint32_t b, mask;
  uint32_t crc = 0xFFFFFFFF;
  uint8_t i;
  int8_t shift;

  for(i = 0 ; i < num_bytes ; i++ ) {
    b = data[i];
    crc = crc ^ b;

    for(shift = 7; shift >= 0; shift--) {
      mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}


void process_serial_packet(char channel, char sub_channel, uint32_t value, dchannels_t& obj)
{
 byte i;

 if (channel == 'S')  {
   if (sub_channel == 'p') {
     int16_t pwm = value & 0xFFFF;
     motor.set_PWM(pwm, 1);
     motor.timeout_count = 0;
    
     PID_pos.active = 0;
     PID_speed.active = 0;
     trajectory.active = 0;

   } else if (sub_channel == 't') {
     pump_set_state(value);
     if(value == ps_init) buzzer_beep = 1;
     
   } else if (sub_channel == 'v') {
     save_prepare();
   }

 } else if (channel == 'L')  {
   PID_speed.active = !!(value & (1 << 0));
   PID_pos.active = !!(value & (1 << 1));
   trajectory.active = !!(value & (1 << 2));
   trajectory.set(motor.pos, 0, motor.pos, 0, 1);
   //serial_channels.send('V', 'x', PID_speed.active || PID_pos.active || trajectory.active, 0);

 } else if (channel == 'P')  {
    i = sub_channel - 'i';
    if (i < NUM_PARS && i >= 0) {
      Pars[i] = *((float *) &value);
    }  
    
 } else if (channel == 'T')  {
   motor.encoder_pos = value;

 } else if (channel == 'R')  {
   motor.speed_ref = *((float *) &value);
   motor.pos_ref = *((float *) &value);
   //if (trajectory.active) {
     trajectory.set(motor.pos, 0, motor.pos_ref, 0, 2);
   //}
 
 } else if (channel == 'I')  {
   set_interval(value);

 } else if (channel == 'G')  { // Ping
   obj.send(channel, sub_channel, value + 1);
   Serial.println(value + 1);
 }
}


void serial_write(uint8_t b)
{
  Serial.write(b);
}


float speed_control(float speed_ref, float speed_odo)
{
  PID_speed.Kp = Pars[0];
  PID_speed.Ki = Pars[1];
  PID_speed.Kd = Pars[2];
  PID_speed.Kf = Pars[3];
  
  float u = PID_speed.calc(speed_ref,  speed_odo);

  return u;
}


float pos_control(float pos_ref, float pos_odo)
{
  PID_pos.Kp = Pars[4];
  PID_pos.Ki = Pars[5];
  PID_pos.Kd = Pars[6];

  float u = PID_pos.calc(pos_ref,  pos_odo);

  return u;
}

void pump_set_cycle(float bpm, float air_volume, float ie_ratio)
{
  if (bpm == 0) return;
  float t_in = 0.1;
  float t_tot = 60 / bpm;
  pump_cycle.t_push = 1.0 / (ie_ratio + 1) * (1.0 - t_in) * t_tot;
  pump_cycle.t_hold = 0.1 * t_tot;
  pump_cycle.t_pull = ie_ratio / (ie_ratio + 1) * (1.0 - t_in)  * t_tot;

  pump_cycle.pos_push = air_volume * Pars[13] + Pars[14];
  pump_cycle.pos_pull = Pars[15];
}

// enum pump_state_t {ps_init, ps_back, ps_home, ps_idle, ps_push, ps_hold, ps_pull, ps_sync, ps_error};

// ps_init  ->  Rough Homing
// ps_back  ->  Go Back to prepare homing 
// ps_home  ->  Homing
// ps_idle  ->  Stopped 
// ps_push  ->  Close
// ps_hold  ->  Hold in
// ps_pull  ->  Open 
// ps_sync  ->  Waiting for the end of the cycle
// ps_error ->  Bad things detected

void pump_set_state(byte new_state)
{
  if (new_state != pump.state) {
    // Here if there was a state change

    // Reset Time Entering State (tes)
    pump.tes = millis();

    // Things to do when entering a new state
    if (new_state == ps_push) {
      pump_set_cycle(bmp_par, volume_par, ie_ratio_par);
      trajectory.set(motor.pos, 0, pump_cycle.pos_push, 0, pump_cycle.t_push);
      trajectory.active = 1;
     

    } else if (new_state == ps_hold) {
      trajectory.set(pump_cycle.pos_push, 0, pump_cycle.pos_push, 0, pump_cycle.t_hold);
      trajectory.active = 1;
    
    } else if (new_state == ps_pull) {
      //float m = (pump_cycle.pos_pull - pump_cycle.pos_push) / pump_cycle.t_pull;
      trajectory.set(pump_cycle.pos_push, 0,  pump_cycle.pos_pull, 0, pump_cycle.t_pull * 0.5);
      trajectory.active = 1;

    } else if (new_state == ps_sync) {
      trajectory.set(motor.pos, 0,  pump_cycle.pos_pull, 0, pump_cycle.t_pull * 0.5);
      trajectory.active = 1;
    
    } else if (new_state == ps_idle) {
      PID_pos.active = 0;
      PID_speed.active = 0;
      trajectory.active = 0;
      motor.set_voltage(0.0);
    }

    pump.state = new_state;
  }
}

void pump_fsm_t::progress(void)
{
  pump.tis = millis() - pump.tes;

  if (pump.state == ps_push && trajectory.t > pump_cycle.t_push){
    pump_set_state(ps_hold) ;
  
  } else if (pump.state == ps_hold &&  trajectory.t > pump_cycle.t_hold){
    pump_set_state(ps_pull); 

  } else if (pump.state == ps_pull && trajectory.t > pump_cycle.t_pull * 0.5){
    pump_set_state(ps_sync); 
      
  } else if (pump.state == ps_sync && trajectory.t > pump_cycle.t_pull * 0.5){
    pump_set_state(ps_push) ;

  } else if (pump.state == ps_init && end_switch_A){
    pump_set_state(ps_back);

  } else if (pump.state == ps_back && !end_switch_A){
    pump_set_state(ps_home);

  } else if (pump.state == ps_home && end_switch_A){
    pump_set_state(ps_idle);
    motor.encoder_pos = 0;
    motor.pos = 0;
    motor.pos_ref = 0;
    motor.speed_ref = 0;
  }  

}

void pump_fsm_t::act(void)
{
  switch (pump.state) {
    
    case ps_init:
      motor.set_voltage(-6.0);
      trajectory.active = 0;
    break;
    
    case ps_back:
      motor.set_voltage(6.0);
      trajectory.active = 0;
    break;
    
    case ps_home:
      motor.set_voltage(-5.0);
      trajectory.active = 0;
    break;
    
    case ps_idle:
      trajectory.active = 0;
    break;

    case ps_hold:
      motor.set_voltage(0.0);
    break;
    
    default:
    break;
  }
}



void setup(void)
{
  // set the digital pin as output:
  pinMode(led_pin, OUTPUT);      
  digitalWrite(led_pin, 0);      

  pinMode(end_switch_A_pin, INPUT_PULLUP);
  pinMode(pressure_pin, INPUT); 

  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, 0);      

  //analogReference(INTERNAL);

  /*
  button_bmp_up.set_pin(button_bmp_up_pin);
  button_bmp_down.set_pin(button_bmp_down_pin);

  button_volume_up.set_pin(button_volume_up_pin);
  button_volume_down.set_pin(button_volume_down_pin);
  
  button_start.set_pin(button_start_pin);
  button_stop.set_pin(button_stop_pin);
  */

  button_bmp_down.set_key(0);
  button_bmp_up.set_key(0);
  button_volume_down.set_key(0);
  button_volume_up.set_key(0);
  button_ie_down.set_key(0);
  button_ie_up.set_key(0);
  button_start.set_key(0);
  button_stop.set_key(0);
  
  motor.init(50); // Calls encoder interrupt every 50us
  
  set_interval(25000UL);

  if (!read_pars_from_eeprom()) {
    fill_sane_pars();
  }

  // initialize serial communication:
  Serial.begin(115200);
  
  serial_channels.init(process_serial_packet, serial_write);
  Serial.println("Serial Init");

  //ticks_to_m = 0.00000934016269960832; // To do with extra precision

  //motor.set_PWM(0, 0); 
  motor.set_voltage(0.0);
  manual_motor_voltage = 3.0;

  tm.displayBegin(); 
  tm.displayText("5dpovent");
 
  last_micros = micros();
  Serial.println("Init Done");

  trajectory.set(0, 0, 0, 0, 1);
  
  /*Serial.println("Traj 1"); 
  trajectory.set(0, 0, 4, 0, 2);
  trajectory.show_values(dt, trajectory.t_f, 6);
  Serial.println("Traj 2");   
  trajectory.set(4, -4.0/3.0, 0, -4.0/3.0, 3);
  trajectory.show_values(dt, trajectory.t_f, 6);
  while(1);*/

  pump.state = ps_init;
  save_requested = 0;

  uint8_t keys = tm.readButtons();
  keys = tm.readButtons();
  //keys = 0b10000000;
  //keys = 0b00000001;
  if (keys == 0b10000000) {
    pump.active = 0;
    // Motor manual action
    view_mode = vm_position;
  } else if (keys == 0b00000001) {
    pump.active = 0;
    // Show pressure
    view_mode = vm_pressure;
  } else if (keys == 0b00000011) {
    pump.active = 1;
    // Load defaults (factory Reset)
    fill_sane_pars();
    save_prepare();
  } else {
    pump.active = 1;
  }
  

  //view_mode = vm_pressure;
  //motor.set_voltage(5.0);
  //while(1);
}


void loop()
{
  //byte i; 
  static int decimate_led  = 0;
  unsigned long act_micros, delta;

  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    byte serialByte = Serial.read();
    serial_channels.StateMachine(serialByte);
  }

  if (save_requested) {
    save_pars_to_eeprom();
  }

  float lambda = 0.02;
  pump.pressure_raw = pump.pressure_raw * (1 - lambda) +  analogRead(pressure_pin) * lambda;


  act_micros = micros();

  delta = act_micros - last_micros;
  if (delta >= interval_us) {
    // save the last time 
    last_micros = act_micros;   

    motor.update_encoder();

    pump.pressure_cmH2O = 70.308893732 * (pump.pressure_raw -  Pars[11]) / (0.8 * 1024);
    
    // buzzer on while buzzer_beep is not zero
    if (buzzer_beep) {
      buzzer_beep--;
      digitalWrite(buzzer_pin, 0);
    } else {
      digitalWrite(buzzer_pin, 1);
    }

    // Motor position and speed in SI units
    motor.speed = ticks_to_m * motor.odo / dt;
    motor.pos = ticks_to_m * motor.encoder_pos;

    prev_end_switch_A = end_switch_A;
    end_switch_A = !digitalRead(end_switch_A_pin);

    // Anti bounce buffer
    key_buffer[key_index] = tm.readButtons(); 
    key_index++;
    if (key_index >= KEY_BUFFER_SIZE) key_index = 0;
    uint8_t keys = tm.readButtons();
    // not being used -> reading twice seems to avoid the bounce
    //for (i = 0; i < KEY_BUFFER_SIZE; i++) keys &= key_buffer[i];
    
    button_bmp_down.read_key(keys & (1 << 0));
    button_bmp_up.read_key(keys & (1 << 1));
    button_volume_down.read_key(keys & (1 << 2));
    button_volume_up.read_key(keys & (1 << 3));
    button_ie_down.read_key(keys & (1 << 4));
    button_ie_up.read_key(keys & (1 << 5));
    button_start.read_key(keys & (1 << 6));
    button_stop.read_key(keys & (1 << 7));

    if (pump.active) {
      normal_key_actions();
    } else {
      alt1_key_actions();
    }

    // Pump state machine
    if (pump.active) {
      pump.progress();
      pump.act();
    }

    // Possible controllers
    if (PID_speed.active) {
      motor.set_voltage(speed_control(motor.speed_ref , motor.speed));
    }
    if (PID_pos.active) {
      motor.set_voltage(pos_control(motor.pos_ref , motor.pos));
    }
    
    if (trajectory.active) {
      motor.speed_ref = trajectory.get_speed(trajectory.t);
      motor.set_voltage(speed_control(motor.speed_ref, motor.speed));
      motor.pos_ref = trajectory.get_position(trajectory.t);
      motor.add_voltage(pos_control(motor.pos_ref, motor.pos));
      trajectory.t += dt;
    }

    //pump.pressure = analogRead(pressure_pin);

    // Display things
    //uint32_t st = micros();

    char workStr[16] = {0};
    int ibmp, ivolume;//, ipos;
    ibmp = bmp_par;
    ivolume = volume_par * 1000 + 0.5;
    uint8_t view_mode_must_change = 0;

    if (millis() - last_view_mode_change > 3000) 
      view_mode_must_change = 1;
    
    if (view_mode == vm_bpm) {
      sprintf(workStr, "rpm   %2d", ibmp);
      if (view_mode_must_change) view_mode = vm_volume;

    } else if (view_mode == vm_volume) {
      sprintf(workStr, "vol  %d", ivolume);
      if (view_mode_must_change) view_mode = vm_ie_ratio;

    } else if (view_mode == vm_ie_ratio) {
      sprintf(workStr, "I/E   %2.1f", (double)ie_ratio_par);
      if (view_mode_must_change) view_mode = vm_bpm;

    } else if (view_mode == vm_position) {
      sprintf(workStr, "Pos %5.1f", (double)(motor.pos * 1000));

    } else if (view_mode == vm_voltage) {
      sprintf(workStr, "V   %5.1f", (double)(manual_motor_voltage));
      if (view_mode_must_change) view_mode = vm_position;

    } else if (view_mode == vm_save_req) {
      sprintf(workStr, "SAVE PAR");
      if (view_mode_must_change) view_mode = vm_bpm;

    } else if (view_mode == vm_pressure) {
      sprintf(workStr, "Pr   %4.1f", (double)(pump.pressure_cmH2O));

    }

    if (view_mode_must_change) {
      last_view_mode_change = millis();
    }

    tm.displayText(workStr);
    
    //Pars[11] = (micros() - st) / 1000;
    //tm.display7Seg(7, 0b01000000);
    //tm.setLED(position, value & 1);
    
    // send Pars Table sequentially (four for each loop)
    for (uint8_t i = 0; i < 4; i++) {
      serial_channels.sendFloat('Y', 'i' + send_par_index, Pars[send_par_index]);
      send_par_index++;
      if (send_par_index >= NUM_PARS) send_par_index = 0;
    }

    // send Pressure
    serial_channels.sendFloat('W', 'p', pump.pressure_cmH2O); 
    
    // send speed
    serial_channels.sendFloat('W', 'o', motor.speed); 

    // send speed reference
    serial_channels.sendFloat('W', 'r', motor.speed_ref); 

    // send posiotion reference
    serial_channels.sendFloat('R', 'p', motor.pos_ref); 
    
    // sendPWM
    serial_channels.send('M', 'p', delta / 10, motor.PWM_value);     
    
    // sendPID state
    serial_channels.send('V', 'x', pump.state, PID_speed.active | (PID_pos.active << 1) | (trajectory.active << 2) | (end_switch_A << 3));

    // End MetaPacket
    // sendOdometry
    serial_channels.send('P','o', motor.encoder_pos); 

    decimate_led++;
    if (decimate_led > 10) {
      // if the heartbeat LED is off turn it on and vice-versa:
      led_state = !led_state;
      digitalWrite(led_pin, led_state);
      //Serial.print(led_state);

      decimate_led = 0;
    }

    Serial.println();
  }
}


// Non blocking EEPROM save for the Pars array

void save_prepare(void)
{
  uint8_t i;

  // Lets see the array of floats as an array of bytes
  byte* p = (byte*)Pars;

  // Fill the buffer (reserve 4 bytes for the crc32 check)
  for (i = 4; i < SAVE_BUFFER_SIZE; i++) {
    save_buffer[i] = *p;
    p++;
  }

  // Calculate the crc32 check for the buffer 
  uint32_t crc = crc32c(&(save_buffer[4]), SAVE_BUFFER_SIZE - 4);
  // and store it on the first four bytes
  *((uint32_t*)save_buffer) = crc;

  save_par_byte = 0;
  save_requested = 1;
}

// save_buffer starts with a crc32 of the following data
// 4 * 16 bytes of data (the memory image of 16 floats from Pars[])
void save_pars_to_eeprom(void)
{
  // Never wait for a write
  if (!eeprom_is_ready()) return;

  // Save one byte if necessary
  eeprom_update_byte((uint8_t*)save_par_byte, save_buffer[save_par_byte]);

  // Next time we save the next byte
  save_par_byte++;
  if (save_par_byte >= (int)SAVE_BUFFER_SIZE) {
    save_par_byte = 0;
    save_requested = 0;
  }
}

uint8_t read_pars_from_eeprom(void)
{
  eeprom_read_block((void*)save_buffer, 0, SAVE_BUFFER_SIZE);
  // Calculate the crc32 check for the buffer 
  uint32_t crc = crc32c(&(save_buffer[4]), SAVE_BUFFER_SIZE - 4);
  if (*((uint32_t*)save_buffer) == crc ) {
    byte* p = (byte*)Pars;

    // Fill the buffer 
    for (uint8_t i = 4; i < SAVE_BUFFER_SIZE; i++) {
      *p = save_buffer[i];
      p++;
    }
    return 1;
  } else {
    return 0;
  }
}

void fill_sane_pars(void)
{
  Pars[0] = 200; // Kp
  Pars[1] = 400; // Ki
  Pars[2] = 0;   // Kd
  Pars[3] = 110; // Kf
  Pars[4] = 2000; // PosKp
  Pars[5] = 600; // PosKi
  Pars[6] = 50; // PosKd
  Pars[7] = 20; // BPM
  Pars[8] = 0.5; // Volume
  Pars[9] = 2; // I/E	ratio
  Pars[10] = 0.00000934016269960832; // ticks_to_m 
  Pars[11] = 101; // Pmin Offset
  Pars[12] = 0; // 
  Pars[13] = 0.02; // Air_0
  Pars[14] = 0.02; // Air_1
  Pars[15] = 0.015; // Rest
}

void normal_key_actions(void)
{
  if (button_bmp_up.re()) {
    if (view_mode == vm_bpm) {
      bmp_par += 2;
    } else {
      view_mode = vm_bpm;
    }
    last_view_mode_change = millis();
  }

  if (button_bmp_down.re()) {
    if (view_mode == vm_bpm) {
      bmp_par -= 2;
    } else {
      view_mode = vm_bpm;
    }
    last_view_mode_change = millis();
  }

  if (button_start.re()) {
    if (pump.state == ps_idle) pump_set_state(ps_push);
  }

  if (button_stop.re()) {
    pump_set_state(ps_init);
  }
  
  if (button_volume_up.re()) {
    if (view_mode == vm_volume) {
      volume_par += 0.05;
    } else {
      view_mode = vm_volume;
    }
    last_view_mode_change = millis();
  }
  
  if (button_volume_down.re()) {
    if (view_mode == vm_volume) {
      volume_par -= 0.05;
    } else {
      view_mode = vm_volume;
    }
    last_view_mode_change = millis();
  }

  if (button_ie_up.re()) {
    if (view_mode == vm_ie_ratio) {
      ie_ratio_par += 0.1;
    } else {
      view_mode = vm_ie_ratio;
    }
    last_view_mode_change = millis();
  }

  if (button_ie_down.re()) {
    if (view_mode == vm_ie_ratio) {
      ie_ratio_par -= 0.1;
    } else {
      view_mode = vm_ie_ratio;
    }
    last_view_mode_change = millis();
  }
  
  if (bmp_par < 8) bmp_par = 8;
  if (bmp_par > 26) bmp_par = 26;    
  if (volume_par < 0.25) volume_par = 0.25;
  if (volume_par > 0.7) volume_par = 0.7;
  if (ie_ratio_par < 0.8) ie_ratio_par = 0.8;
  if (ie_ratio_par > 4) ie_ratio_par = 4;    

  // Long press stop to save pars
  if (button_stop.time_pressed() > 4000) {
    save_prepare();
    view_mode = vm_save_req;
    last_view_mode_change = millis();
  }
}

void alt1_key_actions(void)
{
  if (button_bmp_up.re()) {
    motor.set_voltage(manual_motor_voltage);
    //buzzer_beep = 1;
  } else if (button_bmp_up.fe()) {
    motor.set_voltage(0.0);
  }

  if (button_bmp_down.re()) {
    motor.set_voltage(-manual_motor_voltage);
    //buzzer_beep = 1;
  } else if (button_bmp_down.fe()) {
    motor.set_voltage(0.0);
  }

  if (button_start.re()) {
  }

  if (button_stop.re()) {
  }
  
  if (button_volume_up.re()) {
  }
  
  if (button_volume_down.re()) {
  }

  if (button_ie_up.re()) {
    manual_motor_voltage += 0.5;
    if (manual_motor_voltage >= 12.0) manual_motor_voltage = 12.0;
    last_view_mode_change = millis();
    view_mode = vm_voltage;
  }

  if (button_ie_down.re()) {
    manual_motor_voltage -= 0.5;
    if (manual_motor_voltage <= 0.0) manual_motor_voltage = 0.0;
    last_view_mode_change = millis();
    view_mode = vm_voltage;
  }
  
  if (prev_end_switch_A == 0 && end_switch_A) {
    motor.encoder_pos = 0;
    motor.pos = 0;
  }

  if (button_stop.time_pressed() > 4000) {
    save_prepare();
    view_mode = vm_save_req;
    last_view_mode_change = millis();
  }
}