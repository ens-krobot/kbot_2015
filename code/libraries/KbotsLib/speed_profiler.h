/************************************************************************
 * File : speed_profiler.h                                              *
 *  Class to manage speed profiles for fine motor control.              *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#ifndef SPEED_PROFILER_H__
#define SPEED_PROFILER_H__

#include <Arduino.h>
#include <scheduler.h>
#include "propulsion.h"
#include "odometry.h"
#include <math.h>

#define DEFAULT_KP_THETA 3.0

class SpeedProfiler : public ScheduledTask {
 public:
  // Enumeration for internal state machine
  enum profile_type {
    none = 0,
    linear,
    rotation,
    linear_theta,
  };

  // Constructor:
  //  Builds a new SpeedProfiler object
  // Parameters:
  //  - period: period of the control loop update in microseconds
  SpeedProfiler(unsigned long period);

  // void begin(Odometry *odometer, DifferentialDrive *ddrive,
  //            float Kp_theta):
  //  Initialier the object.
  // Parameters:
  //  - ddrive: pointer to the DifferentialDrive object controlling the motors
  //  - odometer: pointer to the Odometry object estimating the robot's position
  //  - Kp_theta: Gain of the proportional controller used to drive in straigth
  //              lines (defaults to DEFAULT_KP_THETA which is tuned for standard
  //              operations)
  void begin(Odometry *odometer, DifferentialDrive *ddrive,
             float Kp_theta = DEFAULT_KP_THETA);

  // char is_following_profile():
  //  Return the type of the profile currently followed by the object.
  //  (the value will be non-zero if the object is currently following
  // a speed profile).
  char is_following_profile();

  // char start_linear_profile(float d, float vmax, float amax):
  //  Starts a profile on the linear speed.
  // Parameters:
  //  - d: distance to move of in meters. Positive values move forward
  //       and negative values move backwards.
  //  - vmax: maximum velocity to achieve during the movement in m/s.
  //          This value should be positive.
  //  - amax: maximum acceleration in m/s^2 allowed during the movement.
  //          This value should be positive.
  // Return value:
  //  -1 if the object is already following a profile.
  //  Zero if no error is encountered.
  char start_linear_profile(float d, float vmax, float amax);

  // char start_linear_profile_theta(float d, float vmax, float amax,
  //                                 float theta_cons):
  //  Starts a profile on the linear speed, controlling the orientation of
  //  the robot to keep a straight line.
  // Parameters:
  //  - d: distance to move of in meters. Positive values move forward
  //       and negative values move backwards.
  //  - vmax: maximum velocity to achieve during the movement in m/s.
  //          This value should be positive.
  //  - amax: maximum acceleration in m/s^2 allowed during the movement.
  //          This value should be positive.
  //  - theta_cons: heading the robot should keep during the movement in radians.
  // Return value:
  //  -1 if the object is already following a profile.
  //  Zero if no error is encountered.
  char start_linear_profile_theta(float d, float vmax, float amax, float theta_cons);

  // float find_closest_direction(float theta, unsigned int n_directions):
  //  Helper method for start_linear_profile_theta.
  //  Divides the total space the robot in a given number of directions and returns
  //  the heading corresponding to the closest one.
  //  For instance, to move along North/East/South/West, use n_directions = 4.
  //  The method will return the value between [0., M_PI/2, M_PI, -M_PI] closest
  //  to the heading argument.
  // Parameters:
  //  - theta: current heading of the robot in ]-M_PI; M_PI].
  //  - n_directions: number of directions allowed to the robot.
  //                  if this value is 0, the function will return theta.
  // Return value:
  //  Closest direction allowed to the robot to the current heading.
  //  This value will be expressed in ]-M_PI; M_PI]
  float find_closest_segment(float theta, unsigned int n_directions);

  // float automatic_heading(unsigned int n_directions):
  //  Same as find_closest_segment but will get the current heading from
  //  the Odometery object
  // Parameters:
  //  - n_directions: number of directions allowed to the robot.
  //                  if this value is 0, the function will return theta.
  // Return value:
  //  Closest direction allowed to the robot to the current heading.
  //  This value will be expressed in ]-M_PI; M_PI]
  float automatic_heading(unsigned int n_directions);

  // char start_rotation_profile(float alpha, float omega_max, float amax):
  //  Starts a profile on the rotational speed.
  // Parameters:
  //  - alpha: angle to turn of in radians. Positive values turn in the direct
  //           direction (CCW) and negative values turn in the indirect direction
  //           (CW).
  //  - omega_max: maximum velocity to achieve during the movement in rad/s.
  //               This value should be positive.
  //  - amax: maximum acceleration in rad/s^2 allowed during the movement.
  //          This value should be positive.
  // Return value:
  //  -1 if the object is already following a profile.
  //  Zero if no error is encountered.
  char start_rotation_profile(float alha, float omega_max, float amax);

  // void stop_motion():
  //  Stop the robot "instantly", i.e. without slowing down profile.
  void stop_motion();

  // virtual void run():
  //  Profile generation loop method
  virtual void run();

 protected:
  float vmax_, amax_, theta_ref_, Kp_;
  char is_following_, profile_sign_;
  unsigned long start_time_, duration_;
  Odometry *odometer_;
  DifferentialDrive *ddrive_;
};

#endif
