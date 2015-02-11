/************************************************************************
 * File : speed_profiler.cpp                                            *
 *  Class to manage speed profiles for fine motor control.              *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#include "speed_profiler.h"

SpeedProfiler::SpeedProfiler(unsigned long period) :
            ScheduledTask(period, 0) {
}

void SpeedProfiler::begin(Odometry *odometer, Propulsion *ddrive,
                          float Kp_theta) {
  odometer_ = odometer;
  ddrive_ = ddrive;
  Kp_ = Kp_theta;
  is_following_ = none;

  // start task now that the object has been initialized
  start_task();
}

char SpeedProfiler::is_following_profile() {
  return is_following_;
}


char SpeedProfiler::start_linear_profile(float d, float vmax, float amax) {
  if (is_following_ != none) {
    return -1;
  } else {
    float dlim = vmax*vmax/amax;
    if (fabs(d)>dlim) {
      // Normal profile
      duration_ = (unsigned long)roundf((fabs(d)/vmax+vmax/amax)*1e6);
    } else {
      // Degenerated profile (triangular)
      duration_ = (unsigned long)roundf(sqrt(4.*fabs(d)/amax)*1e6);
    }
    vmax_ = d >= 0. ? vmax : -vmax;
    amax_ = d >= 0. ? amax : -amax;
    profile_sign_ = d >= 0. ? 1 : -1;
    start_time_ = micros();
    is_following_ = linear;
  }

  return 0;
}

char SpeedProfiler::start_linear_profile_theta(float d, float vmax,
                                               float amax, float theta_cons) {
  char res = start_linear_profile(d, vmax, amax);
  if (res == 0) {
    theta_ref_ = theta_cons;
    is_following_ = linear_theta;
  }

  return res;
}

float SpeedProfiler::find_closest_segment(float theta, unsigned int n_directions) {
  // Find the closest direction
  float elementary_alpha = 2.*M_PI/n_directions;
  int n_quarters = (int)roundf(theta / elementary_alpha);
  // Choose the correct orientation if we are in the limit case
  if (n_directions % 2 == 0 && n_quarters == -(int)(n_directions/2)) {
    n_quarters = n_directions/2;
  }

  // Compute heading
  return elementary_alpha*n_quarters;
}

float SpeedProfiler::automatic_heading(unsigned int n_directions) {
  return find_closest_segment(odometer_->theta_, n_directions);
}

char SpeedProfiler::start_rotation_profile(float alpha, float omega_max, float amax) {
  if (is_following_ != none) {
    return -1;
  } else {
    float alphalim = omega_max*omega_max/amax;
    if (fabs(alpha)>alphalim) {
      // Normal profile
      duration_ = (unsigned long)roundf((fabs(alpha)/omega_max+omega_max/amax)*1e6);
    } else {
      // Degenerated profile (triangular)
      duration_ = (unsigned long)roundf(sqrt(4.*fabs(alpha)/amax)*1e6);
    }
    vmax_ = alpha >= 0. ? omega_max : -omega_max;
    amax_ = alpha >= 0. ? amax : -amax;
    profile_sign_ = alpha >= 0. ? 1 : -1;
    start_time_ = micros();
    is_following_ = rotation;
  }

  return 0;
}

void SpeedProfiler::stop_motion() {
  is_following_ = none;
  ddrive_->set_speeds(0., 0.);
}

void SpeedProfiler::run(void) {
  unsigned long cur_time = micros();
  float new_speed, angle_error, left_speed, right_speed;
  char end_profile = 0;

  if (is_following_ != none) {
    float profile1 = amax_ * (cur_time - start_time_) * 1e-6;
    float profile2 = amax_ * (duration_ - cur_time + start_time_) * 1e-6;
    if (profile_sign_ > 0) {
      new_speed = min(vmax_, min(profile1, profile2));
    } else {
      new_speed = max(vmax_, max(profile1, profile2));
    }
    if (cur_time - start_time_ >= duration_) {
      end_profile = 1;
      new_speed = 0.;
    }
    switch (is_following_) {
    case linear:
      ddrive_->lin_speed_ref_ = new_speed;
      break;
    case linear_theta:
      ddrive_->lin_speed_ref_ = new_speed;
      angle_error = (odometer_->theta_ - theta_ref_);
      if (angle_error > M_PI) {
        angle_error -= 2.*M_PI;
      } else if (angle_error < -M_PI) {
        angle_error += 2.*M_PI;
      }
      angle_error *= - Kp_;
      left_speed = (2.0*new_speed-angle_error*ddrive_->shaft_) / (2.0 * ddrive_->left_radius_);
      right_speed = (2.0*new_speed+angle_error*ddrive_->shaft_) / (2.0 * ddrive_->right_radius_);
      if (left_speed * right_speed < 0) {
        angle_error = 0.;
      }
      ddrive_->rot_speed_ref_ = angle_error;
      break;
    case rotation:
      ddrive_->rot_speed_ref_ = new_speed;
      break;
    default:
      break;
    }
    if (end_profile) {
      ddrive_->set_speeds(0., 0.);
      is_following_ = none;
    }
  }
}

