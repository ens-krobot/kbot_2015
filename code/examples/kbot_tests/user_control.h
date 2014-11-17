#include <Arduino.h>
#include <scheduler.h>
#include <KbotsLib.h>

#define DISTANCE 2.
#define LIN_SPEED (0.5)
#define LIN_ACC (LIN_SPEED/2.)

#define ROT_SPEED M_PI/4.
#define ROT_ACC (ROT_SPEED)

class UserControl : public ScheduledTask {
 public:
  UserControl(unsigned long period) : ScheduledTask(period) { }
  
  void begin(HardwareSerial *ui_serial, long baudrate,
             BatteryMonitor *batt,
             SpeedProfiler *speed_profiler,
             DifferentialDrive *ddrive,
             Odometry *odometer,
             uint8_t up, uint8_t down,
             uint8_t center,
             uint8_t left, uint8_t right,
             uint8_t following_led,
             unsigned int n_directions) {
               
    ui_serial_ = ui_serial;
    batt_ = batt;
    speed_profiler_ = speed_profiler;
    ddrive_ = ddrive;
    odometer_ = odometer;
    up_ = up;
    down_ = down;
    center_ = center;
    left_ = left;
    right_ = right;
    follow_led_ = following_led;
    n_dir_ = n_directions;
    
    ui_serial_->begin(baudrate);
    pinMode(up, INPUT_PULLUP);
    pinMode(down, INPUT_PULLUP);
    pinMode(center, INPUT_PULLUP);
    pinMode(left, INPUT_PULLUP);
    pinMode(right, INPUT_PULLUP);
    pinMode(following_led, OUTPUT);
    
    ui_serial_->println("Kbot ready for orders !");
    ui_serial_->println("     Is it time for world domination today ?");
    ui_serial_->println("(By the way, my \"joking light\" is broken... sorry... :))");
  }
  
  virtual void run() {
    // Display SpeedProfiler state on one LED
    digitalWrite(follow_led_, speed_profiler_->is_following_profile() != SpeedProfiler::none ? HIGH : LOW);

    // Manage the inputs from the "keyboard":
    if (speed_profiler_->is_following_profile() == SpeedProfiler::none) {
      if (digitalRead(up_) == LOW) {
        speed_profiler_->start_linear_profile_theta(DISTANCE, LIN_SPEED, LIN_ACC, speed_profiler_->automatic_heading(n_dir_));
      }
      if (digitalRead(down_) == LOW) {
        speed_profiler_->start_linear_profile_theta(-DISTANCE, LIN_SPEED, LIN_ACC, speed_profiler_->automatic_heading(n_dir_));
      }
      if (digitalRead(left_) == LOW) {
        speed_profiler_->start_rotation_profile(2.*M_PI/n_dir_, ROT_SPEED, ROT_ACC);
      }
      if (digitalRead(right_) == LOW) {
        speed_profiler_->start_rotation_profile(-2.*M_PI/n_dir_, ROT_SPEED, ROT_ACC);
      }
    } else {
      if (digitalRead(center_) == LOW) {
        speed_profiler_->stop_motion();
      }
    }

    if (ui_serial_->available()) {
      byte c = ui_serial_->read();
      float theta;
      switch(c) {
        case 's':
          // Stop the robot
          ui_serial_->println("Stopping the robot.");
          speed_profiler_->stop_motion();
          break;
        case 'f':
          // Move forward:
          theta = speed_profiler_->automatic_heading(n_dir_);
          ui_serial_->print("Forward: my orientation is ");
          ui_serial_->print(odometer_->get_theta()*180./M_PI);
          ui_serial_->print(", moving with heading ");
          ui_serial_->println(theta*180./M_PI);
          //speed_profiler_->start_linear_profile_theta(DISTANCE, LIN_SPEED, LIN_ACC, theta);
          speed_profiler_->start_linear_profile(DISTANCE, LIN_SPEED, LIN_ACC);
          break;
        case 'b':
          // Move backward:
          theta = speed_profiler_->automatic_heading(n_dir_);
          ui_serial_->print("Backward: my orientation is ");
          ui_serial_->print(odometer_->get_theta()*180./M_PI);
          ui_serial_->print(", moving with heading ");
          ui_serial_->println(theta*180./M_PI);
          //speed_profiler_->start_linear_profile_theta(-DISTANCE, LIN_SPEED, LIN_ACC, theta);
          speed_profiler_->start_linear_profile(-DISTANCE, LIN_SPEED, LIN_ACC);
          break;
        case 'l':
          // Turn left:
          theta = speed_profiler_->automatic_heading(n_dir_);
          ui_serial_->print("Turning left of ");
          ui_serial_->print(360/n_dir_);
          ui_serial_->println(" deg");
          speed_profiler_->start_rotation_profile(2.*M_PI/n_dir_, ROT_SPEED, ROT_ACC);
          break;
        case 'r':
          // Turn right:
          theta = speed_profiler_->automatic_heading(n_dir_);
          ui_serial_->print("Turning right of ");
          ui_serial_->print(360/n_dir_);
          ui_serial_->println(" deg");
          speed_profiler_->start_rotation_profile(-2.*M_PI/n_dir_, ROT_SPEED, ROT_ACC);
          break;
        case 'o':
          // Display odometry values
          ui_serial_->print("I'm supposed to be at X=");
          ui_serial_->print(odometer_->get_x());
          ui_serial_->print(", Y=");
          ui_serial_->print(odometer_->get_y());
          ui_serial_->print(", theta=");
          ui_serial_->println(odometer_->get_theta()*180./M_PI);
          break;
        case 'p':
          // Printing battery info
          ui_serial_->println("Here is my power status:");
          ui_serial_->print("Cell 1: ");
          ui_serial_->print(batt_->get_cell1_voltage());
          ui_serial_->print(" V, Cell2: ");
          ui_serial_->print(batt_->get_cell2_voltage());
          ui_serial_->print(" V, Cell3: ");
          ui_serial_->print(batt_->get_cell3_voltage());
          ui_serial_->print(" V -> Total: ");
          ui_serial_->print(batt_->get_total_voltage());
          ui_serial_->println(" V.");
          break;
        case 'h':
          // Printing help
          ui_serial_->println("Here is what I can do:");
          ui_serial_->println("----------------------");
          ui_serial_->println("s: Stopping my trajectory");
          ui_serial_->println("f: moving forward");
          ui_serial_->println("b: moving backward");
          ui_serial_->println("l: turning left");
          ui_serial_->println("r: turning right");
          ui_serial_->println("o: displaying odometry values");
          ui_serial_->println("p: display battery infos");
          break;
        default:
          ui_serial_->println("I'm sorry but I don't understand what you mean there...");
      }
    }
  }
  
 protected:
  HardwareSerial *ui_serial_;
  SpeedProfiler *speed_profiler_;
  DifferentialDrive *ddrive_;
  Odometry *odometer_;
  BatteryMonitor *batt_;
  uint8_t up_, down_, center_, left_, right_, follow_led_;
  unsigned int n_dir_;
};
