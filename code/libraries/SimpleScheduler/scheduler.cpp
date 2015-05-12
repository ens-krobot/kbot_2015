/************************************************************************
 * File : scheduler.cpp                                                 *
 *  Arduino library for simple cooperative threads.                     *
 *                                                                      *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/

#include "scheduler.h"
#include <Arduino.h>

// ScheduledTask definitions

ScheduledTask::ScheduledTask(unsigned long period, char run) :
  period_(period), is_running_(run) {
  next_run_ = micros();
}

void ScheduledTask::set_period(unsigned long period) {
  period_ = period;
}

void ScheduledTask::start_task() {
  is_running_ = 1;
  next_run_ = micros();
}

void ScheduledTask::stop_task() {
  is_running_ = 0;
}

// Scheduler definitions

ScheduledTask *Scheduler::queued_tasks_[SCHEDULER_MAX_TASKS];
unsigned char Scheduler::num_tasks_;
unsigned long Scheduler::timer_ends_[MAX_CHRONOS];
unsigned char Scheduler::is_timer_active_[MAX_CHRONOS];

const unsigned long Scheduler::millisecond = 1000UL;
const unsigned long Scheduler::microsecond = 1UL;

void Scheduler::begin() {
  for (unsigned char i=0; i < SCHEDULER_MAX_TASKS; i++) {
    queued_tasks_[i] = NULL;
  }
  num_tasks_ = 0;
  cleanup_chronos();
}

void Scheduler::update() {
  unsigned long cur_time = 0;
  for (unsigned char i = 0; i < num_tasks_; i++) {
    cur_time = micros();
    if (cur_time >= queued_tasks_[i]->next_run_) {
      queued_tasks_[i]->run();
      queued_tasks_[i]->next_run_ = cur_time + queued_tasks_[i]->period_;
    }
  }
}

char Scheduler::add_task(ScheduledTask *task) {
  if (num_tasks_ == SCHEDULER_MAX_TASKS) {
    return 1;
  }
  queued_tasks_[num_tasks_] = task;
  num_tasks_++;
  return 0;
}

char Scheduler::start_chrono(unsigned long duration) {
  for (char idx = 0; idx < MAX_CHRONOS; idx++) {
    if (!is_timer_active_[idx]) {
      timer_ends_[idx] = micros()+duration;
      is_timer_active_[idx] = 1;
      return idx;
    }
  }
  // No chrono available
  return -1;
}

bool Scheduler::has_chrono_elapsed(char chrono_idx) {
  if (chrono_idx >= 0 && chrono_idx < MAX_CHRONOS) {
    return (micros() >= timer_ends_[chrono_idx]);
  } else {
    return true;
  }
}

char Scheduler::reset_chrono(char chrono_idx) {
  if (chrono_idx >= 0 && chrono_idx < MAX_CHRONOS) {
    if (is_timer_active_[chrono_idx]) {
      is_timer_active_[chrono_idx] = 0;
      return 0;
    } else {
      return -1;
    }
  } else {
    return -2;
  }
}

void Scheduler::cleanup_chronos() {
  for (unsigned int idx=0; idx < MAX_CHRONOS; idx++) {
    is_timer_active_[idx] = 0;
  }
}
