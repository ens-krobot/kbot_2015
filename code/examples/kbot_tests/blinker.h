#include <scheduler.h>

class Blinker : public ScheduledTask {
 public:
  Blinker(int pin, unsigned long period) : ScheduledTask(period/2) {
    pin_ = pin;
    state_ = 0;
    pinMode(pin, OUTPUT);
  }
  virtual void run() {
    digitalWrite(pin_, state_ ? LOW : HIGH);
    state_ = !state_;
  }
 protected:
  int pin_;
  char state_;
};

