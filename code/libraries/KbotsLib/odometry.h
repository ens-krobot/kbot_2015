/************************************************************************
 * File : odometry.h                                                    *
 *  Class to manage odometry for wheeled robots.                        *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <Arduino.h>
#include <scheduler.h>
#include <math.h>

// Forward declaration of "higher" classes for friend declaration
class DifferentialDrive;
class SpeedProfiler;
class Propulsion;

class Odometry : public ScheduledTask {
 public:
  // For internal use by the library
  friend class DifferentialDrive;
  friend class Propulsion;
  friend class SpeedProfiler;

  // Constructor
  //  Build a new odometer
  // Parameters:
  //  - period: period of the odometer update in microseconds
  Odometry(unsigned long period);

  // void begin(float left_gain, float left_radius,
  //            float right_gain, float right_radius,
  //            float shaft_width,
  //            uint8_t left_cod_A, uint8_t left_cod_interrupt_A,
  //            uint8_t left_cod_B, uint8_t left_cod_interrupt_B,
  //            uint8_t right_cod_A, uint8_t right_cod_interrupt_A,
  //            uint8_t right_cod_B, uint8_t right_cod_interrupt_B);
  //  Initialize the Odometry object for a differential drive robot
  // Parameters:
  //  - left_gain, right_gain: encoder gains for both wheels
  //  - left_radius, right_radius: wheels' radius
  //  - shaft_width: distance between the wheels' center
  void begin(float left_gain, float left_radius,
             float right_gain, float right_radius,
             float shaft_width,
             uint8_t left_cod_A, uint8_t left_cod_interrupt_A,
             uint8_t left_cod_B, uint8_t left_cod_interrupt_B,
             uint8_t right_cod_A, uint8_t right_cod_interrupt_A,
             uint8_t right_cod_B, uint8_t right_cod_interrupt_B);

  // void begin(float left_gain, float left_radius,
  //            float right_gain, float right_radius,
  //            float shaft_width,
  //            uint8_t left_cod_A, uint8_t left_cod_interrupt_A,
  //            uint8_t left_cod_B, uint8_t left_cod_interrupt_B,
  //            uint8_t right_cod_A, uint8_t right_cod_interrupt_A,
  //            uint8_t right_cod_B, uint8_t right_cod_interrupt_B);
  //  Initialize the Odometry object for an omnidirectional drive robot
  // Parameters:
  //  - left_gain, right_gain, front_gain: encoder gains for wheels
  //  - left_radius, right_radius, front_radius: wheels' radius
  //  - robot_radius: radius of the omnidirectional drive
  void begin(float left_gain, float left_radius,
             float right_gain, float right_radius,
             float front_gain, float front_radius,
             float robot_radius,
             uint8_t left_cod_A, uint8_t left_cod_interrupt_A,
             uint8_t left_cod_B, uint8_t left_cod_interrupt_B,
             uint8_t right_cod_A, uint8_t right_cod_interrupt_A,
             uint8_t right_cod_B, uint8_t right_cod_interrupt_B,
             uint8_t front_cod_A, uint8_t front_cod_interrupt_A,
             uint8_t front_cod_B, uint8_t front_cod_interrupt_B);

  // Destructor
  //  Does nothing
  virtual ~Odometry() {};

  // virtual void run():
  //  Main loop
  virtual void run();

  // Accessor methods
  //   Accessors to get one parameter
  //   Distances are in meters and angles in radians
  float get_x();
  float get_y();
  float get_theta();
  float get_left_angle();
  float get_right_angle();
  float get_front_angle();

  // void get_position(float *x, float *y, float *theta):
  //   Accessor method to get all state variables from the robot
  // Parameters:
  //  - x: pointer to the variable in which to store value of X (in meters)
  //  - y: pointer to the variable in which to store value of Y (in meters)
  //  - theta: pointer to the variable in which to store value of theta (in radians)
  void get_position(float *x, float *y, float *theta);


  // void get_angles(float *left, float *right):
  //  Accessor method to get current positions of the motors
  // Parameters
  //  - left: pointer to the variable in which
  //          to store value of the left encoder (in radians)
  //  - right: pointer to the variable in which
  //           to store value of the right encoder (in radians)
  void get_angles(float *left, float *right);

  // void get_angles(float *left, float *right, float *front):
  //  Accessor method to get current positions of the motors
  // Parameters
  //  - left: pointer to the variable in which
  //          to store value of the left encoder (in radians)
  //  - right: pointer to the variable in which
  //           to store value of the right encoder (in radians)
  //  - front: pointer to the variable in which
  //           to store value of the front encoder (in radians)
  void get_angles(float *left, float *right, float *front);

  // void reset(float x, float y, float theta):
  //  Reset the odometry to the given values [x,y,theta]^T
  //  with x,y in meters and theta in radians
  void reset(float x, float y, float theta);

  // static void enable_encoders(boolean state):
  //  Activate/deactivate counting tops on the encoders
  // Parameters:
  //  - state: if true the encoders will be activated
  static void enable_encoders(boolean state);

  volatile static uint8_t left_A_, left_B_, right_A_, right_B_, front_A_, front_B_;
  volatile static unsigned int left_enc_, right_enc_, front_enc_;
  volatile static boolean enable_encoders_;
 protected:
  enum DriveType {
    differential,
    omnidirectional
  };
  float x_, y_, theta_;
  float left_angle_, right_angle_, front_angle_;
  float left_gain_, right_gain_, front_gain_;
  float right_radius_, left_radius_, front_radius_;
  float shaft_;
  unsigned int last_left_, last_right_, last_front_;
  DriveType type_;
};

#endif // __ODOMETRY_H
