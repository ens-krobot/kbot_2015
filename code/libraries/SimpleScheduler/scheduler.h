/************************************************************************
 * File : scheduler.h                                                   *
 *  Arduino library for simple cooperative threads.                     *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

// Maximum number of tasks which can be managed by the scheduler
#define SCHEDULER_MAX_TASKS 10
#define MAX_CHRONOS 3

// Forward declaration of class Scheduler for friend reference
class Scheduler;

// Abstract implementation of a ScheduledTask. A task has to derivate
// this class in order to implement its own 'run' method.
class ScheduledTask {
  friend class Scheduler;
 public:

  // Constructor
  //  Build a new task
  // Parameters:
  //  - period: period of the task in microseconds
  //  - run: the task will be running when created if non-zero ? (defaults to 1)
  ScheduledTask(unsigned long period, char run = 1);

  // Destructor
  //  Does nothing in the base implementation
  virtual ~ScheduledTask() {};

  // virtual void run(void):
  //  Main function executed by the thread. This function has to be
  //  overridden by a child class to actually do something.
  //  It will be called every 'period' microseconds.
  virtual void run(void) = 0;

  // void set_period(unsigned long period):
  //  Change the current period of the task
  // Parameters:
  //  - period: new period
  void set_period(unsigned long period);

  // void start_task():
  //  Start the task
  void start_task();

  // void stop_task():
  //  Stop the task
  void stop_task();

 protected:
  unsigned long period_, next_run_;
  unsigned char is_running_;
};

class Scheduler {
 public:
  // Helpers to express time for use in defining periods
  static const unsigned long millisecond;
  static const unsigned long microsecond;

  // Default constructor and destructor.
  // These should not be used as everything in the class is static
  Scheduler() {};
  ~Scheduler() {};

  // Initialize the Scheduler
  static void begin();

  // Update the Scheduler state.
  //  This method has to be called in the main arduino program's loop.
  //  It will take care of calling the tasks' run methods when needed.
  static void update();

  // char add_task(ScheduledTask *task)
  //  This method will add a task to the scheduler.
  // Parameters:
  //  - task: Pointer to the task object to add to the scheduler
  // Return value:
  //  - 0: no error
  //  - 1: no more available slot in scheduler manager
  static char add_task(ScheduledTask *task);

  // char start_chrono(unsigned long duration)
  //  This method starts a chrono if one is available.
  // Parameters:
  //  - duration: duration in microseconds in which the chrono is
  //              supposed to elapse
  // Return value:
  //  Index of the chrono assigned, -1 if no chrono is available
  static char start_chrono(unsigned long duration);

  // bool has_chrono_elapsed(char chrono_idx)
  //  This method checks if a chrono has elapsed
  // Parameters:
  //  - chrono_idx: index of the chrono to check.
  // Return value:
  //  'true' if the chrono has elapsed, 'false' otherwise.
  static bool has_chrono_elapsed(char chrono_idx);

  // char reset_chrono(char chrono_idx)
  //  This methods resets a given chrono
  // Parameters:
  //  - chrono_idx: index of the chrono to check
  // Return value:
  //  zero if the reset was 'needed'
  static char reset_chrono(char chrono_idx);

  // void cleanup_chronos():
  //  Clean up all chronos
  static void cleanup_chronos();

 protected:
  static ScheduledTask *queued_tasks_[SCHEDULER_MAX_TASKS];
  static unsigned char num_tasks_;
  static unsigned long timer_ends_[MAX_CHRONOS];
  static unsigned char is_timer_active_[MAX_CHRONOS];
};

#endif /* __SCHEDULER_H */
