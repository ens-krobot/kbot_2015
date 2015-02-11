/************************************************************************
 * File : propulsion.h                                                  *
 *  Class to manage propulsion for wheeled robots.                      *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#ifndef __PROPULSION_H
#define __PROPULSION_H

#include <Arduino.h>
#include <scheduler.h>
#include "odometry.h"

// Forward declaration of "higher" classes for friend declaration
class SpeedProfiler;

class Propulsion : public ScheduledTask {
 public:
  // For internal use by the library
  friend class SpeedProfiler;

  // Enumeration for motor description
  enum motors {
    left_motor = 0,
    right_motor = 1,
    front_motor = 2
  };
  enum motor_mode {
    enable,
    break_low,
    break_high,
    disable
  };

  // Constructor:
  //  Builds a new Propulsion object
  // Parameters:
  //  - period: period of the control loop update in microseconds
  Propulsion(unsigned long period);

  // void begin(uint8_t left_in1, uint8_t left_in2, uint8_t left_en,
  //            uint8_t right_in1, uint8_t right_in2, uint8_t right_en,
  //            float Kp, float Ki,
  //            Odometry *odometer,
  //            float shaft_width, float left_wheel_radius, float right_wheel_radius):
  //  Initialize the object for differential drives.
  // Parameters:
  //  - left_in1, right_in1: pin number of the first H-bridge input for both motors
  //  - left_in2, right_in2: pin number of the second H-bridge input for both motors
  //  - left_en, right_en: pin number of the H-bridge's enable for both motors
  //  - Kp: proportionnal gain of the wheel position's PI controller
  //  - Ki: integral gain of the wheel position's PI controller
  //  - odometer: pointer to the odometry object following the robot movements
  //  - shaft_width, left_wheel_radius, right_wheel_radius: physical parameters
  //                           of the differential drive
  //  - period: period of the control loop update in microseconds
  void begin(uint8_t left_in1, uint8_t left_in2, uint8_t left_en,
             uint8_t right_in1, uint8_t right_in2, uint8_t right_en,
             float Kp, float Ki,
             Odometry *odometer,
             float shaft_width, float left_wheel_radius, float right_wheel_radius);

  // void begin(uint8_t left_in1, uint8_t left_in2, uint8_t left_en,
  //            uint8_t right_in1, uint8_t right_in2, uint8_t right_en,
  //            uint8_t front_in1, uint8_t front_in2, uint8_t, front_en,
  //            float Kp, float Ki,
  //            Odometry *odometer,
  //            float robot_radius,
  //            float left_wheel_radius, float right_wheel_radius, float front_wheel_radius):
  //  Initialize the object for omnidirectional drives.
  // Parameters:
  //  - left_in1, right_in1, front_in1: pin number of the first H-bridge input for motors
  //  - left_in2, right_in2, front_in2: pin number of the second H-bridge input for motors
  //  - left_en, right_en, front_en: pin number of the H-bridge's enable for motors
  //  - Kp: proportionnal gain of the wheel position's PI controller
  //  - Ki: integral gain of the wheel position's PI controller
  //  - odometer: pointer to the odometry object following the robot movements
  //  - robot_radius, left_wheel_radius, right_wheel_radius, front_wheel_radius:
  //            physical parameters of the differential drive
  //  - period: period of the control loop update in microseconds
  void begin(uint8_t left_in1, uint8_t left_in2, uint8_t left_en,
             uint8_t right_in1, uint8_t right_in2, uint8_t right_en,
             uint8_t front_in1, uint8_t front_in2, uint8_t front_en,
             float Kp, float Ki,
             Odometry *odometer,
             float robot_radius,
             float left_wheel_radius, float right_wheel_radius, float front_wheel_radius);

  // void set_speeds(float linear_speed, float rotational_speed):
  //  Set the reference speeds for the control loop in differential drive mode
  // Parameters:
  //  - linear_speed: linear speed of the robot in m/s
  //  - rotational_speed: rotational speed of the robot in rad/s
  void set_speeds(float linear_speed, float rotational_speed);

  // void set_speeds(float linear_speed_X, float linear_speed_Y, float rotational_speed):
  //  Set the reference speeds for the control loop in omnidirectional drive mode
  // Parameters:
  //  - linear_speed_X: linear speed of the robot along the X axis in m/s
  //  - linear_speed_Y: linear speed of the robot along the Y axis in m/s
  //  - rotational_speed: rotational speed of the robot in rad/s
  void set_speeds(float linear_speed_X, float linear_speed_Y, float rotational_speed);

  // void set_motor_mode(motor_mode mode):
  //  Set the working mode of the motor (enabled, free rolling, breaking)
  void set_motor_mode(motor_mode mode);

  // void set_max_command(motors motor_id, int max_cmd):
  //  Set the maximum command to be applied to one of the motors.
  // Parameters:
  //  - motor_id: ID of the motor
  //  - max_command: This value will be used to saturate the command applied to
  //                 the motor in [-max_cmd; max_cmd]
  void set_max_command(motors motor_id, int max_cmd);

  // void invert_motors_command(boolean left, boolean right):
  //  Invert or not the command applied to each motor.
  // Parameters:
  //  - left, right, front: if true the corresponding motor command will be inverted
  //                 (defaults to false in the constructor)
  void invert_motor_commands(boolean left, boolean right);
  void invert_motor_commands(boolean left, boolean right, boolean front);

  // void set_dead_zones(int left, int right):
  //  Set the dead zones for each motors. This value will be added to the command
  // sent to the motors when driving them.
  void set_dead_zones(int left, int right);
  void set_dead_zones(int left, int right, int front);

  // void set_max_integrator(float max_integrator):
  //  Set the maximum value of the integral contribution of the PI controller
  void set_max_integrator(float max_integrator);

  // void reset_controller():
  //  Set the control loop errors to zero.
  void reset_controller();

  // virtual void run():
  //  Control loop method
  virtual void run();

  // void set_motor_cmd(motors motor_id, int vel):
  //  Set the command applied to one of the motors.
  // Parameters:
  //  - motor_id: ID of the motor
  //  - vel: command to apply (signed). This value will be saturated in
  //         [-max_motor_cmd; max_motor_cmd]
  void set_motor_cmd(motors motor_id, int vel);
 protected:
  enum PropulsionType {
    differential,
    omnidirectional
  };

  uint8_t pin_in1_[3], pin_in2_[3], pin_en_[3];
  float pos_ref_[3], corr_int_[3];
  float lin_speed_X_ref_, lin_speed_Y_ref_, lin_speed_ref_, rot_speed_ref_;
  float shaft_, left_radius_, right_radius_, front_radius_;
  char last_dir_[3];
  int max_cmd_[3], inv_cmd_[3], dead_zones_[3];
  float max_int_, Kp_, Ki_;
  Odometry *odometer_;
  unsigned long last_control_;
  PropulsionType type_;
};

#endif
