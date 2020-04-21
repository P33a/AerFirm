#ifndef PROJ_TYPES_H
#define PROJ_TYPES_H

#include "Arduino.h"

//#define PRESSURE_SENSOR_ANALOG 1
#define PRESSURE_SENSOR_I2C 2

enum pump_state_t {ps_init, ps_back, ps_home, ps_idle, ps_push, ps_hold, ps_pull, ps_sync, ps_error, ps_direct, ps_PID_speed, ps_PID_position};

enum view_mode_t {vm_bpm, vm_volume, vm_ie_ratio, vm_cycle, vm_position, vm_voltage, vm_save_req, vm_pressure};

class pump_fsm_t{
  public:
  uint32_t tes, tis;
  byte state;//, active;
  float pressure_raw, pressure_cmH2O;
  
  void act(void);
  void progress(void);

};


typedef struct{
  float t_push, t_pull, t_hold;
  float pos_push, pos_pull;
  //float speed_push, speed_pull;
  
} pump_cycle_t;


typedef struct{
  uint8_t r, g, b, a; 
} rgba_t;




#endif // PROJ_TYPES_H
