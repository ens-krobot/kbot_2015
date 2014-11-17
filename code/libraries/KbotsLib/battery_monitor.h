/************************************************************************
 * File : battery_monitor.h                                             *
 *  Class to compute battery state.                                     *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#ifndef __BATTERY_MONITOR_H
#define __BATTERY_MONITOR_H

#include <Arduino.h>
#include <scheduler.h>
#include <math.h>

class BatteryMonitor : public ScheduledTask {
 public:
  // Constructor
  //  Build a new odometer
  // Parameters:
  //  - minimum_voltage: minimum voltage a cell has to reach to trigger an
  //                     alarm.
  //  - period: period of the odometer update in microseconds
  BatteryMonitor(float minimum_voltage,
                 uint8_t cell1_pin,
                 uint8_t cell2_pin,
                 uint8_t cell3_pin,
                 uint8_t buzzer_pin,
                 unsigned long period);

  // Destructor
  //  Does nothing
  virtual ~BatteryMonitor() {};

  // virtual void run():
  //  Main loop
  virtual void run();

  // Accessor methods
  float get_cell1_voltage();
  float get_cell2_voltage();
  float get_cell3_voltage();
  float get_total_voltage();

 protected:
  float min_voltage_, cell_v_[3];
  uint8_t cell_pins_[3], buzz_;
};

#endif // __BATTERY_MONITOR_H
