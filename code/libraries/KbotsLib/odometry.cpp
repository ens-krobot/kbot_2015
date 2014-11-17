/************************************************************************
 * File : odometry.cpp                                                  *
 *  Class to manage odometry for wheeled robots.                        *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#include "odometry.h"

volatile uint8_t Odometry::left_A_;
volatile uint8_t Odometry::left_B_;
volatile uint8_t Odometry::right_A_;
volatile uint8_t Odometry::right_B_;
volatile unsigned int Odometry::left_enc_;
volatile unsigned int Odometry::right_enc_;
volatile boolean Odometry::enable_encoders_;

void interrupt_left_enc_A() {
  if (Odometry::enable_encoders_) {
    if (digitalRead(Odometry::left_A_)) {
      if (digitalRead(Odometry::left_B_)) {
        Odometry::left_enc_++;
      } else {
        Odometry::left_enc_--;
      }
    } else {
      if (digitalRead(Odometry::left_B_)) {
        Odometry::left_enc_--;
      } else {
        Odometry::left_enc_++;
      }
    }
  }
}

void interrupt_left_enc_B() {
  if (Odometry::enable_encoders_) {
    if (digitalRead(Odometry::left_B_)) {
      if (digitalRead(Odometry::left_A_)) {
        Odometry::left_enc_--;
      } else {
        Odometry::left_enc_++;
      }
    } else {
      if (digitalRead(Odometry::left_A_)) {
        Odometry::left_enc_++;
      } else {
        Odometry::left_enc_--;
      }
    }
  }
}

void interrupt_right_enc_A() {
  if (Odometry::enable_encoders_) {
    if (digitalRead(Odometry::right_A_)) {
      if (digitalRead(Odometry::right_B_)) {
        Odometry::right_enc_--;
      } else {
        Odometry::right_enc_++;
      }
    } else {
      if (digitalRead(Odometry::right_B_)) {
        Odometry::right_enc_++;
      } else {
        Odometry::right_enc_--;
      }
    }
  }
}

void interrupt_right_enc_B() {
  if (Odometry::enable_encoders_) {
    if (digitalRead(Odometry::right_B_)) {
      if (digitalRead(Odometry::right_A_)) {
        Odometry::right_enc_++;
      } else {
        Odometry::right_enc_--;
      }
    } else {
      if (digitalRead(Odometry::right_A_)) {
        Odometry::right_enc_--;
      } else {
        Odometry::right_enc_++;
      }
    }
  }
}

Odometry::Odometry(unsigned long period) :
  ScheduledTask(period, 0) {
}

void Odometry::begin(float left_gain, float left_radius,
                     float right_gain, float right_radius,
                     float shaft_width,
                     uint8_t left_cod_A, uint8_t left_cod_interrupt_A,
                     uint8_t left_cod_B, uint8_t left_cod_interrupt_B,
                     uint8_t right_cod_A, uint8_t right_cod_interrupt_A,
                     uint8_t right_cod_B, uint8_t right_cod_interrupt_B) {

  left_gain_ = left_gain;
  left_radius_ = left_radius;
  right_gain_ = right_gain;
  right_radius_ = right_radius;
  shaft_ = shaft_width;

  left_enc_ = 0;
  last_left_ = 0;
  right_enc_ = 0;
  last_right_ = 0;

  enable_encoders_ = true;

  reset(0., 0., 0.);
  left_angle_ = 0.;
  right_angle_ = 0.;

  left_A_ = left_cod_A;
  left_B_ = left_cod_B;
  right_A_ = right_cod_A;
  right_B_ = right_cod_B;

  pinMode(left_A_, INPUT);
  pinMode(left_B_, INPUT);
  pinMode(right_A_, INPUT);
  pinMode(right_B_, INPUT);

  attachInterrupt(left_cod_interrupt_A,interrupt_left_enc_A,CHANGE);
  attachInterrupt(left_cod_interrupt_B,interrupt_left_enc_B,CHANGE);
  attachInterrupt(right_cod_interrupt_A,interrupt_right_enc_A,CHANGE);
  attachInterrupt(right_cod_interrupt_B,interrupt_right_enc_B,CHANGE);

  // start task now that the object has been initialized
  start_task();
}

void Odometry::run(void) {

  static int count = 0;

  unsigned int left_enc = left_enc_;
  unsigned int right_enc = right_enc_;
  unsigned int left_delta = left_enc_ - last_left_;
  unsigned int right_delta = right_enc_ - last_right_;
  float delta_l = left_gain_ * (left_delta >= 32768 ? ((long)left_delta) - 65536 : left_delta);
  float delta_r = right_gain_ * (right_delta >= 32768 ? ((long)right_delta) - 65536 : right_delta);

  // New state computation
  left_angle_ += delta_l;
  right_angle_ += delta_r;
  x_ += (right_radius_ * delta_r + left_radius_ * delta_l) / 2.0 * cos(theta_);
  y_ += (right_radius_ * delta_r + left_radius_ * delta_l) / 2.0 * sin(theta_);
  theta_ += (right_radius_ * delta_r - left_radius_ * delta_l) / shaft_;

  // Normalization of theta
  if (theta_ > M_PI) {
    theta_ -= 2.*M_PI;
  } else if (theta_ < -M_PI) {
    theta_ += 2.*M_PI;
  }

  last_left_ = left_enc_;
  last_right_ = right_enc_;
}

float Odometry::get_x() {
  return x_;
}
float Odometry::get_y() {
  return y_;
}
float Odometry::get_theta() {
  return theta_;
}
float Odometry::get_left_angle() {
  return left_angle_;
}
float Odometry::get_right_angle() {
  return right_angle_;
}

void Odometry::get_position(float *x, float *y, float *theta) {
  *x = x_;
  *y = y_;
  *theta = theta_;
}

void Odometry::get_angles(float *left, float *right) {
  *left = left_angle_;
  *right = right_angle_;
}

void Odometry::reset(float x, float y, float theta) {
  x_ = x;
  y_ = y;
  theta_ = theta;
}

void Odometry::enable_encoders(boolean state) {
    enable_encoders_ = state;
}

