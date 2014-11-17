/************************************************************************
 * File : battery_monitor.cpp                                           *
 *  Class to compute battery state.                                     *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#include "battery_monitor.h"

BatteryMonitor::BatteryMonitor(float minimum_voltage,
                               uint8_t cell1_pin,
                               uint8_t cell2_pin,
                               uint8_t cell3_pin,
                               uint8_t buzzer_pin,
                               unsigned long period) :
                      ScheduledTask(period) {
  cell_pins_[0] = cell1_pin;
  cell_pins_[1] = cell2_pin;
  cell_pins_[2] = cell3_pin;
  buzz_ = buzzer_pin;

  pinMode(buzz_, OUTPUT);

  for (char i=0; i < 3; i++) {
    cell_v_[i] = NAN;
  }

  min_voltage_ = minimum_voltage;
}

void BatteryMonitor::run() {
  // Get values
  cell_v_[0] = analogRead(cell_pins_[0]) * 0.014445;
  cell_v_[1] = analogRead(cell_pins_[1]) * 0.014445;
  cell_v_[2] = analogRead(cell_pins_[2]) * 0.0048828;

  // Deduce voltage of each element
  cell_v_[0] -= cell_v_[1];
  cell_v_[1] -= cell_v_[2];

  // Alarm if one cell is too low
  char alarm = 0, not_connected = 0;
  for (char i=0; i < 3; i++) {
    if (cell_v_[i] <= min_voltage_) {
      alarm++;
      if (cell_v_[i] <= 2.) {
        not_connected++;
      }
    }
  }
  if (not_connected > 0) {
    tone(buzz_, 440, 200);
  } else if (alarm > 0) {
    tone(buzz_, 1000);
  } else {
    noTone(buzz_);
  }
}

float BatteryMonitor::get_cell1_voltage() {
  return cell_v_[0];
}

float BatteryMonitor::get_cell2_voltage() {
  return cell_v_[1];
}

float BatteryMonitor::get_cell3_voltage() {
  return cell_v_[2];
}

float BatteryMonitor::get_total_voltage() {
  return cell_v_[0]+cell_v_[1]+cell_v_[2];
}
