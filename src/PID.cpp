#include "PID.h"
#include <iostream>

/*
* The PID class.
*/

PID::PID() { bTwittleEnabled = false; }

PID::~PID() {
  log_fs_.close();
  data_fs_.close();
  std::cout << "best gains: [" << best_gains_[0] << ", " << best_gains_[1]
            << ", " << best_gains_[0] << "]" << std::endl;
}

void PID::Init(double Kp, double Kd, double Ki, bool enableTwiddle,
               float thresh, std::string name) {
  best_gains_[0] = gains_[0] = Kp;
  best_gains_[1] = gains_[1] = Kd;
  best_gains_[2] = gains_[2] = Ki;
  errors_[0] = errors_[1] = errors_[2] = 0.0;
  bTwittleEnabled = enableTwiddle;
  state_ = pid::twiddle_state::INIT;
  threshold_ = thresh;
  step_ = 0;
  index_ = 0;
  avg_error_ = 0;
  best_error_ = 1e9;
  max_steps_ = 8000;
  max_index_ = 2;
  best_step_ = 0;
  runs_ = 0;
  name_ = name;
  start_time_ = std::chrono::system_clock::now();

  for (int i = 0; i < pid::kParams; i++) {
    potentials_[i] = 0.5 * gains_[i];
  }

  char filename[256] = {0};
  snprintf(filename, 256, "%s_log.csv", name_.c_str());
  log_fs_.open(filename);
}

pid::return_code PID::UpdateError(double cte, bool crash) {
  std::chrono::_V2::system_clock::time_point this_time =
      std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds;

  //
  // PID Control
  //
  // initialize first step and calculate the D error using time delta
  if (step_ == 0) {
    errors_[0] = errors_[1] = errors_[2] = 0.0;
    avg_error_ = 0;
    last_time_ = this_time;
    char filename[256] = {0};
    snprintf(filename, 256, "%s_data_%03d.csv", name_.c_str(), runs_);
    data_fs_.open(filename);
    runs_++;
  } else {
    elapsed_seconds = this_time - last_time_;
    errors_[1] = (cte - errors_[0]) / elapsed_seconds.count();
    last_time_ = this_time;
  }

  // assign P and I errors
  errors_[0] = cte;
  errors_[2] += cte;

  // save run data to file
  SaveData(cte, gains_[0] * errors_[0], gains_[1] * errors_[1],
           gains_[2] * errors_[2], TotalError());

  // set default return code
  pid::return_code ret = pid::return_code::CONTINUE;

  step_++;

  //
  // Twiddle Algorithm
  //
  if (bTwittleEnabled) {
    double sum_dp = potentials_[0] + potentials_[1] + potentials_[2];
    // exit condition is the sum of potentials
    if (sum_dp > threshold_) {

      if (step_ == 1 &&
          state_ == pid::twiddle_state::INIT) // first run = no changes
      {
        PrintCurrentGains();
      } else if (step_ == 1 && state_ == pid::twiddle_state::INDEX) {
        gains_[index_] += potentials_[index_];
        PrintCurrentGains();
        state_ = pid::twiddle_state::INCREMENT;
      }

      // sum the squared error
      avg_error_ += cte * cte;

      // waiting 300 steps gives the simulator time to respond to a restart
      if ((step_ >= 300 && step_ >= max_steps_) || (crash && step_ > 300)) {

        // sum of squared error over time
        avg_error_ /= (last_time_ - start_time_).count();

        // ensure crashes do not count as success
        if (crash) {
          avg_error_ = 1e9;
        }

        // twiddle algorithm
        PrintResults();
        if (state_ == pid::twiddle_state::INCREMENT) {
          if (avg_error_ < best_error_) { // better
            best_error_ = avg_error_;
            UpdateSuccess();
            potentials_[index_] *= 1.1;
            index_ = (index_ + 1) % pid::kParams; // move to next parameter
            state_ = pid::twiddle_state::INDEX;
          } else { // worse
            state_ = pid::twiddle_state::DECREMENT;
            gains_[index_] -= 2 * potentials_[index_];
          }
        } else if (state_ == pid::twiddle_state::DECREMENT) {
          if (avg_error_ < best_error_) { // better
            best_error_ = avg_error_;
            UpdateSuccess();
            potentials_[index_] *= 1.1;
          } else { // worse
            gains_[index_] += potentials_[index_];
            potentials_[index_] *= 0.9;
          }
          index_ = (index_ + 1) % pid::kParams; // move to next parameter
          state_ = pid::twiddle_state::INDEX;
        } else if (state_ == pid::twiddle_state::INIT) {
          state_ = pid::twiddle_state::INDEX;
          index_ = 0;
        }
        // reset the car and start again
        ret = pid::return_code::RESTART;
        step_ = 0;
        avg_error_ = 0;
        data_fs_.close();
        start_time_ = std::chrono::system_clock::now();
      }
    } else {
      // stop here
      PrintResults();
      ret = pid::return_code::FINISHED;
    }
  }

  return ret;
}

double PID::TotalError() {
  return -gains_[0] * errors_[0] - gains_[1] * errors_[1] -
         gains_[2] * errors_[2];
}

void PID::UpdateSuccess() {
  for (int i = 0; i < pid::kParams; i++) {
    best_gains_[i] = gains_[i];
  }
  log_fs_ << "*******Improvmenent*******\n Run [" << runs_ << "] Gains: ["
          << gains_[0] << ", " << gains_[1] << ", " << gains_[2]
          << "] Average Error: [" << avg_error_ << "] Distance: [" << step_
          << "]" << std::endl
          << std::endl;
}

void PID::PrintResults() {
  log_fs_ << "steps [" << step_ << "] Avg error [" << avg_error_ << "]"
          << "] Best error [" << best_error_ << "]" << std::endl
          << std::endl;
}

void PID::PrintCurrentGains() {
  double sum_dp = potentials_[0] + potentials_[1] + potentials_[2];
  log_fs_ << "Using gains [" << gains_[0] << ", " << gains_[1] << ", "
          << gains_[2] << "] "
          << "Index [" << index_ << "] Best error [" << best_error_ << "]"
          << std::endl;
  log_fs_ << "Using potentials [" << potentials_[0] << ", " << potentials_[1]
          << ", " << potentials_[2] << "] "
          << "Sum potentials_ [" << sum_dp << "]" << std::endl
          << std::endl;
}

void PID::SaveData(double sensor, double p, double d, double i, double output) {
  data_fs_ << sensor << ", " << p << ", " << d << ", " << i << ", " << output
           << std::endl;
}
