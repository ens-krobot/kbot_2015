/************************************************************************
 * File : propulsion.cpp                                                *
 *  Class to manage propulsion for wheeled robots.                      *
 *                                                                      *
 * This class is part of the KbotsLib for Arduino.                      *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/
#include "propulsion.h"

#define COS_2PI_3 (-0.5)
#define SIN_2PI_3 0.8660254037844387

#define DEFAULT_MAX_MOTOR_CMD 255
#define DEFAULT_MAX_INTEGRATOR 255.
#define DEFAULT_DEAD_ZONE 0

Propulsion::Propulsion(unsigned long period) :
  ScheduledTask(period, 0) {
}

void Propulsion::begin(uint8_t left_in1, uint8_t left_in2,
                       uint8_t left_en,
                       uint8_t right_in1, uint8_t right_in2,
                       uint8_t right_en,
                       float Kp, float Ki,
                       Odometry *odometer,
                       float shaft_width,
                       float left_wheel_radius,
                       float right_wheel_radius) {

  type_ = differential;

  pin_in1_[left_motor] = left_in1;
  pin_in2_[left_motor] = left_in2;
  pin_en_[left_motor] = left_en;
  last_dir_[left_motor] = 0;
  max_cmd_[left_motor] = DEFAULT_MAX_MOTOR_CMD;
  pos_ref_[left_motor] = 0.;
  corr_int_[left_motor] = 0.;
  inv_cmd_[left_motor] = 1;
  dead_zones_[left_motor] = DEFAULT_DEAD_ZONE;

  pin_in1_[right_motor] = right_in1;
  pin_in2_[right_motor] = right_in2;
  pin_en_[right_motor] = right_en;
  last_dir_[right_motor] = 0;
  max_cmd_[right_motor] = DEFAULT_MAX_MOTOR_CMD;
  pos_ref_[right_motor] = 0.;
  corr_int_[right_motor] = 0.;
  inv_cmd_[right_motor] = 1;
  dead_zones_[right_motor] = DEFAULT_DEAD_ZONE;

  Kp_ = Kp;
  Ki_ = Ki;
  shaft_ = shaft_width;
  left_radius_ = left_wheel_radius;
  right_radius_ = right_wheel_radius;
  max_int_ = DEFAULT_MAX_INTEGRATOR;
  odometer_ = odometer;

  for (char i=0; i < 2; i++) {
    pinMode(pin_in1_[i], OUTPUT);
    pinMode(pin_in2_[i], OUTPUT);
    pinMode(pin_en_[i], OUTPUT);
    digitalWrite(pin_in1_[i], LOW);
    digitalWrite(pin_in2_[i], LOW);
    digitalWrite(pin_en_[i], LOW);
  }

  last_control_ = 0;

  // start task now that the object has been initialized
  start_task();
}

void Propulsion::begin(uint8_t left_in1, uint8_t left_in2,
                       uint8_t left_en,
                       uint8_t right_in1, uint8_t right_in2,
                       uint8_t right_en,
                       uint8_t front_in1, uint8_t front_in2,
                       uint8_t front_en,
                       float Kp, float Ki,
                       Odometry *odometer,
                       float robot_radius,
                       float left_wheel_radius,
                       float right_wheel_radius,
                       float front_wheel_radius) {

  type_ = omnidirectional;

  pin_in1_[left_motor] = left_in1;
  pin_in2_[left_motor] = left_in2;
  pin_en_[left_motor] = left_en;
  last_dir_[left_motor] = 0;
  max_cmd_[left_motor] = DEFAULT_MAX_MOTOR_CMD;
  pos_ref_[left_motor] = 0.;
  corr_int_[left_motor] = 0.;
  inv_cmd_[left_motor] = 1;
  dead_zones_[left_motor] = DEFAULT_DEAD_ZONE;

  pin_in1_[right_motor] = right_in1;
  pin_in2_[right_motor] = right_in2;
  pin_en_[right_motor] = right_en;
  last_dir_[right_motor] = 0;
  max_cmd_[right_motor] = DEFAULT_MAX_MOTOR_CMD;
  pos_ref_[right_motor] = 0.;
  corr_int_[right_motor] = 0.;
  inv_cmd_[right_motor] = 1;
  dead_zones_[right_motor] = DEFAULT_DEAD_ZONE;

  pin_in1_[front_motor] = front_in1;
  pin_in2_[front_motor] = front_in2;
  pin_en_[front_motor] = front_en;
  last_dir_[front_motor] = 0;
  max_cmd_[front_motor] = DEFAULT_MAX_MOTOR_CMD;
  pos_ref_[front_motor] = 0.;
  corr_int_[front_motor] = 0.;
  inv_cmd_[front_motor] = 1;
  dead_zones_[front_motor] = DEFAULT_DEAD_ZONE;

  Kp_ = Kp;
  Ki_ = Ki;
  shaft_ = robot_radius;
  left_radius_ = left_wheel_radius;
  right_radius_ = right_wheel_radius;
  front_radius_ = front_wheel_radius;
  max_int_ = DEFAULT_MAX_INTEGRATOR;
  odometer_ = odometer;

  for (char i=0; i < 3; i++) {
    pinMode(pin_in1_[i], OUTPUT);
    pinMode(pin_in2_[i], OUTPUT);
    pinMode(pin_en_[i], OUTPUT);
    digitalWrite(pin_in1_[i], LOW);
    digitalWrite(pin_in2_[i], LOW);
    digitalWrite(pin_en_[i], LOW);
  }

  last_control_ = 0;

  // start task now that the object has been initialized
  start_task();
}

void Propulsion::set_speeds(float linear_speed, float rotational_speed) {
  lin_speed_ref_ = linear_speed;
  rot_speed_ref_ = rotational_speed;
}

void Propulsion::set_speeds(float linear_speed_X,
                            float linear_speed_Y,
                            float rotational_speed) {
  lin_speed_X_ref_ = linear_speed_X;
  lin_speed_Y_ref_ = linear_speed_Y;
  rot_speed_ref_ = rotational_speed;
}

void Propulsion::set_motor_mode(Propulsion::motor_mode mode) {
  switch(mode) {
  case enable:
    // Stop motor before enabling
    digitalWrite(pin_in1_[0], LOW);
    digitalWrite(pin_in2_[0], LOW);
    digitalWrite(pin_in1_[1], LOW);
    digitalWrite(pin_in2_[1], LOW);
    // enable motors
    digitalWrite(pin_en_[0], HIGH);
    digitalWrite(pin_en_[1], HIGH);
    if (type_ == omnidirectional) {
      digitalWrite(pin_in1_[2], LOW);
      digitalWrite(pin_in2_[2], LOW);
      digitalWrite(pin_en_[2], HIGH);
    }
    break;
  case break_high:
    digitalWrite(pin_in1_[0], HIGH);
    digitalWrite(pin_in2_[0], HIGH);
    digitalWrite(pin_in1_[1], HIGH);
    digitalWrite(pin_in2_[1], HIGH);
    if (type_ == omnidirectional) {
      digitalWrite(pin_in1_[2], HIGH);
      digitalWrite(pin_in2_[2], HIGH);
    }
    break;
  case break_low:
    digitalWrite(pin_in1_[0], LOW);
    digitalWrite(pin_in2_[0], LOW);
    digitalWrite(pin_in1_[1], LOW);
    digitalWrite(pin_in2_[1], LOW);
    if (type_ == omnidirectional) {
      digitalWrite(pin_in1_[2], LOW);
      digitalWrite(pin_in2_[2], LOW);
    }
    break;
  case disable:
    digitalWrite(pin_en_[0], LOW);
    digitalWrite(pin_en_[1], LOW);
    if (type_ == omnidirectional) {
      digitalWrite(pin_en_[2], LOW);
    }
    break;
  }
  last_dir_[0] = 0;
  last_dir_[1] = 0;
  last_dir_[2] = 0;
}

void Propulsion::set_max_command(Propulsion::motors motor_id,
                                        int max_cmd) {
  if (max_cmd > 0) {
    max_cmd_[motor_id] = min(max_cmd, DEFAULT_MAX_MOTOR_CMD);
  }
}

void Propulsion::invert_motor_commands(boolean left, boolean right) {
  inv_cmd_[left_motor] = left ? -1 : 1;
  inv_cmd_[right_motor] = right ? -1 : 1;
}
void Propulsion::invert_motor_commands(boolean left, boolean right, boolean front) {
  invert_motor_commands(left, right);
  inv_cmd_[front_motor] = front ? -1 : 1;
}

void Propulsion::set_dead_zones(int left, int right) {
  dead_zones_[left_motor] = left;
  dead_zones_[right_motor] = right;
}
void Propulsion::set_dead_zones(int left, int right, int front) {
  set_dead_zones(left, right);
  dead_zones_[front_motor] = front;
}

void Propulsion::set_max_integrator(float max_integrator) {
  max_int_ = max_integrator;
}

void Propulsion::reset_controller() {
  pos_ref_[left_motor] = odometer_->left_angle_;
  pos_ref_[right_motor] = odometer_->right_angle_;
  pos_ref_[front_motor] = odometer_->front_angle_;
  corr_int_[left_motor] = 0.;
  corr_int_[right_motor] = 0.;
  corr_int_[front_motor] = 0.;
}

void Propulsion::run(void) {
  static unsigned long count;

  unsigned long cur_time = micros();
  float dt = (cur_time - last_control_) / 1e6;
  float speed_ref[3], measures[3];
  char max_mots = 0;

  if (type_ == differential) {
    // Get wheel angles measurements
    measures[0] = odometer_->left_angle_;
    measures[1] = odometer_->right_angle_;

    // Compute wheels speed depending on robot's global speeds
    speed_ref[left_motor] = (2.0*lin_speed_ref_-rot_speed_ref_*shaft_) / (2.0 * left_radius_);
    speed_ref[right_motor] = (2.0*lin_speed_ref_+rot_speed_ref_*shaft_) / (2.0 * right_radius_);

    max_mots = 2;
  } else {
    // Get wheel angles measurements
    measures[0] = odometer_->left_angle_;
    measures[1] = odometer_->right_angle_;
    measures[2] = odometer_->front_angle_;

    // Compute robot speed in local frame of reference
    float vx = lin_speed_X_ref_*cos(odometer_->theta_) + lin_speed_Y_ref_*sin(odometer_->theta_);
    float vy = -lin_speed_X_ref_*sin(odometer_->theta_) + lin_speed_Y_ref_*cos(odometer_->theta_);

    // Compute wheels speed depending on robot's global speeds
    speed_ref[left_motor] =
      (-1 * COS_2PI_3 * vx
       -1 * SIN_2PI_3 * vy
       + shaft_ * rot_speed_ref_) / left_radius_;
    speed_ref[right_motor] =
      (-1 * COS_2PI_3 * vx
       +1 * SIN_2PI_3 * vy
       + shaft_ * rot_speed_ref_) / right_radius_;
    speed_ref[front_motor] =
      (- vx
       + shaft_ * rot_speed_ref_) / front_radius_;

    max_mots = 3;
  }

  // Control both motors
  for (char i=0; i < max_mots; i++) {
    // Compute new position reference depending on wheel speed
    pos_ref_[i] += speed_ref[i] * dt;

    // Compute position error
    float error = pos_ref_[i] - measures[i];

    // Compute integral terms
    corr_int_[i] += Ki_ * error;
    if (corr_int_[i] > max_int_) {
      corr_int_[i] = max_int_;
    } else if (corr_int_[i] < -max_int_) {
      corr_int_[i] = -max_int_;
    }

    // Compute and apply commands
    set_motor_cmd((motors)i, Kp_*error + corr_int_[i]);
  }

  last_control_ = cur_time;
}

void Propulsion::set_motor_cmd(Propulsion::motors motor_id, int vel) {
  vel *= inv_cmd_[motor_id];
  if (vel > 0) {
    vel += dead_zones_[motor_id];
  } else if (vel < 0) {
    vel -= dead_zones_[motor_id];
  }
  if (vel > max_cmd_[motor_id]) vel = max_cmd_[motor_id];
  if (vel < -max_cmd_[motor_id]) vel = -max_cmd_[motor_id];

  if (vel >= 0 && last_dir_[motor_id] != 1) {
    digitalWrite(pin_in2_[motor_id], LOW);
    last_dir_[motor_id] = 1;
  } else if (vel < 0 && last_dir_[motor_id] != -1) {
    digitalWrite(pin_in1_[motor_id], LOW);
    last_dir_[motor_id] = -1;
  }
  if (vel >= 0) {
    analogWrite(pin_in1_[motor_id], vel);
  } else {
    analogWrite(pin_in2_[motor_id], -vel);
  }
}

