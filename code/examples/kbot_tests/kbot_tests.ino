#include <scheduler.h>
#include <KbotsLib.h>

#include "config.h"
#include "blinker.h"
#include "user_control.h"

#define LEFT_RADIUS 0.0376 // 0.037686 //0.036513
#define RIGHT_RADIUS 0.0378 // 0.037686 //0.036513
#define SHAFT_WIDTH 0.1995 // 0.198
#define LEFT_ENCODER_GAIN -0.026180
#define RIGHT_ENCODER_GAIN -0.026180

#define KP 40.0
#define KI 0.
#define KP_THETA 2.

#define N_POSSIBLE_DIRECTIONS 4

Blinker blinker(LED_IND, 1000*Scheduler::millisecond);

BatteryMonitor batt_mon(3.0, BATT_1, BATT_2, BATT_3, BUZZER, 1000*Scheduler::millisecond);

Odometry odometer(10*Scheduler::millisecond);
DifferentialDrive dd_drive(10*Scheduler::millisecond);
SpeedProfiler speed_profiler(10*Scheduler::millisecond);
UserControl user_control(100*Scheduler::microsecond);

void setup() {
  //Serial.begin(115200);

  odometer.begin(LEFT_ENCODER_GAIN, LEFT_RADIUS,
                 RIGHT_ENCODER_GAIN, RIGHT_RADIUS,
                 SHAFT_WIDTH,
                 COD1_A, COD1_A_INTERRUPT,
                 COD1_B, COD1_B_INTERRUPT,
                 COD2_A, COD2_A_INTERRUPT,
                 COD2_B, COD2_B_INTERRUPT);
  
  speed_profiler.begin(&odometer, &dd_drive, KP_THETA);
  
  dd_drive.begin(MOT1_1, MOT1_2, MOT1_EN, MOT2_1, MOT2_2, MOT2_EN,
                 KP, KI,
                 &odometer,
                 SHAFT_WIDTH, LEFT_RADIUS, RIGHT_RADIUS);
  dd_drive.invert_motor_commands(false, true);
  dd_drive.set_motor_mode(DifferentialDrive::enable);
  dd_drive.set_dead_zones(40, 40);

  // USB
  /*user_control.begin(&Serial, 115200,
                     &batt_mon, &speed_profiler, &dd_drive, &odometer,
                     UI_BTN_UP, UI_BTN_DOWN, UI_BTN_CENTER, UI_BTN_LEFT, UI_BTN_RIGHT,
                     UI_LED0,
                     N_POSSIBLE_DIRECTIONS);*/
  // Bluetooth
  user_control.begin(&Serial2, 9600,
                     &batt_mon, &speed_profiler, &dd_drive, &odometer,
                     UI_BTN_UP, UI_BTN_DOWN, UI_BTN_CENTER, UI_BTN_LEFT, UI_BTN_RIGHT,
                     UI_LED0,
                     N_POSSIBLE_DIRECTIONS);
                 
  Scheduler::begin();
  Scheduler::add_task(&blinker);
  Scheduler::add_task(&batt_mon);
  Scheduler::add_task(&odometer);
  Scheduler::add_task(&speed_profiler);
  Scheduler::add_task(&dd_drive);
  Scheduler::add_task(&user_control);
}

void loop() {
  Scheduler::update();
}
