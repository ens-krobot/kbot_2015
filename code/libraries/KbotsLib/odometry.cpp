/************************************************************************
 * File : odometry.cpp                                                  *
 *  Class to manage odometry for wheeled robots.                        *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#include "odometry.h"

#define COS_FACT 0.6666666666666667 // 1/(1-cos(2*pi/3))
#define SIN_FACT 0.5773502691896257 // 1/(2*sin(2*pi/3))

volatile uint8_t Odometry::left_A_;
volatile uint8_t Odometry::left_B_;
volatile uint8_t Odometry::right_A_;
volatile uint8_t Odometry::right_B_;
volatile uint8_t Odometry::front_A_;
volatile uint8_t Odometry::front_B_;
volatile unsigned int Odometry::left_enc_;
volatile unsigned int Odometry::right_enc_;
volatile unsigned int Odometry::front_enc_;
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

void interrupt_front_enc_A() {
  if (Odometry::enable_encoders_) {
    if (digitalRead(Odometry::front_A_)) {
      if (digitalRead(Odometry::front_B_)) {
        Odometry::front_enc_++;
      } else {
        Odometry::front_enc_--;
      }
    } else {
      if (digitalRead(Odometry::front_B_)) {
        Odometry::front_enc_--;
      } else {
        Odometry::front_enc_++;
      }
    }
  }
}

void interrupt_front_enc_B() {
  if (Odometry::enable_encoders_) {
    if (digitalRead(Odometry::front_B_)) {
      if (digitalRead(Odometry::front_A_)) {
        Odometry::front_enc_--;
      } else {
        Odometry::front_enc_++;
      }
    } else {
      if (digitalRead(Odometry::front_A_)) {
        Odometry::front_enc_++;
      } else {
        Odometry::front_enc_--;
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

  type_ = differential;

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
  front_angle_ = 0.;

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

void Odometry::begin(float left_gain, float left_radius,
                     float right_gain, float right_radius,
                     float front_gain, float front_radius,
                     float robot_radius,
                     uint8_t left_cod_A, uint8_t left_cod_interrupt_A,
                     uint8_t left_cod_B, uint8_t left_cod_interrupt_B,
                     uint8_t right_cod_A, uint8_t right_cod_interrupt_A,
                     uint8_t right_cod_B, uint8_t right_cod_interrupt_B,
                     uint8_t front_cod_A, uint8_t front_cod_interrupt_A,
                     uint8_t front_cod_B, uint8_t front_cod_interrupt_B) {

  type_ = omnidirectional;

  left_gain_ = left_gain;
  left_radius_ = left_radius;
  right_gain_ = right_gain;
  right_radius_ = right_radius;
  front_gain_ = front_gain;
  front_radius_ = front_radius;
  shaft_ = robot_radius;

  left_enc_ = 0;
  last_left_ = 0;
  right_enc_ = 0;
  last_right_ = 0;
  front_enc_ = 0;
  last_front_ = 0;

  enable_encoders_ = true;

  reset(0., 0., 0.);
  left_angle_ = 0.;
  right_angle_ = 0.;
  front_angle_ = 0.;

  left_A_ = left_cod_A;
  left_B_ = left_cod_B;
  right_A_ = right_cod_A;
  right_B_ = right_cod_B;
  front_A_ = front_cod_A;
  front_B_ = front_cod_B;

  pinMode(left_A_, INPUT);
  pinMode(left_B_, INPUT);
  pinMode(right_A_, INPUT);
  pinMode(right_B_, INPUT);
  pinMode(front_A_, INPUT);
  pinMode(front_B_, INPUT);

  attachInterrupt(left_cod_interrupt_A,interrupt_left_enc_A,CHANGE);
  attachInterrupt(left_cod_interrupt_B,interrupt_left_enc_B,CHANGE);
  attachInterrupt(right_cod_interrupt_A,interrupt_right_enc_A,CHANGE);
  attachInterrupt(right_cod_interrupt_B,interrupt_right_enc_B,CHANGE);
  attachInterrupt(front_cod_interrupt_A,interrupt_front_enc_A,CHANGE);
  attachInterrupt(front_cod_interrupt_B,interrupt_front_enc_B,CHANGE);

  // start task now that the object has been initialized
  start_task();
}

void Odometry::run(void) {

  if (type_ == differential) {
    unsigned int left_enc = left_enc_;
    unsigned int right_enc = right_enc_;
    unsigned int left_delta = left_enc - last_left_;
    unsigned int right_delta = right_enc - last_right_;
    float delta_l = left_gain_ * (left_delta >= 32768 ? ((long)left_delta) - 65536 : left_delta);
    float delta_r = right_gain_ * (right_delta >= 32768 ? ((long)right_delta) - 65536 : right_delta);

    // New state computation
    left_angle_ += delta_l;
    right_angle_ += delta_r;
    x_ += (right_radius_ * delta_r + left_radius_ * delta_l) / 2.0 * cos(theta_);
    y_ += (right_radius_ * delta_r + left_radius_ * delta_l) / 2.0 * sin(theta_);
    theta_ += (right_radius_ * delta_r - left_radius_ * delta_l) / shaft_;

    last_left_ = left_enc;
    last_right_ = right_enc;
  } else {
    unsigned int left_enc = left_enc_;
    unsigned int right_enc = right_enc_;
    unsigned int front_enc = front_enc_;
    unsigned int left_delta = left_enc - last_left_;
    unsigned int right_delta = right_enc - last_right_;
    unsigned int front_delta = front_enc - last_front_;
    float delta_l = left_gain_ * (left_delta >= 32768 ? ((long)left_delta) - 65536 : left_delta);
    float delta_r = right_gain_ * (right_delta >= 32768 ? ((long)right_delta) - 65536 : right_delta);
    float delta_f = front_gain_ * (front_delta >= 32768 ? ((long)front_delta) - 65536 : front_delta);

    // New state computation
    left_angle_ += delta_l;
    right_angle_ += delta_r;
    front_angle_ += delta_f;

    // Displacement according to the robot's reference frame
    float lx = (- front_radius_ * delta_f
                + left_radius_ * delta_l / 2.
                + right_radius_ * delta_r /2.) * COS_FACT;
    float ly = (- left_radius_ * delta_l + right_radius_ * delta_r) * SIN_FACT;

    // Compute absolute displacement
    x_ += lx*cos(theta_) - ly*sin(theta_);
    y_ += lx*sin(theta_) + ly*cos(theta_);
    theta_ += (front_radius_*delta_f
               + left_radius_ * delta_l
               + right_radius_ * delta_r) / (2.*shaft_) * COS_FACT;

    last_left_ = left_enc;
    last_right_ = right_enc;
    last_front_ = front_enc;
  }

  // Normalization of theta
  if (theta_ > M_PI) {
    theta_ -= 2.*M_PI;
  } else if (theta_ < -M_PI) {
    theta_ += 2.*M_PI;
  }
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
float Odometry::get_front_angle() {
  return front_angle_;
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

