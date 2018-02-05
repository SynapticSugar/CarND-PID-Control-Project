#ifndef PID_H
#define PID_H
#include <chrono>
#include <ctime>
#include <fstream>
#include <string>

namespace pid {
// Twiddle state machine
enum twiddle_state { INIT = 0, INCREMENT = 1, DECREMENT = 2, INDEX = 3 };
// Twiddle feedback codes (used for restarting simulator)
enum return_code { FINISHED = 0, CONTINUE = 1, RESTART = 2 };
// Number of controller parameters (gains)
constexpr int kParams = 3;
}

class PID {
public:
  /*
  * Errors
  */
  double errors_[pid::kParams]; //{d_error, p_error, i_error}

  /*
  * Coefficients
  */
  double gains_[pid::kParams];      //{Kp, Kd, Ki}
  double potentials_[pid::kParams]; //{Dp, Dd, Di}
  double best_gains_[pid::kParams]; //{Kp, Kd, Ki}

  int step_;                 // the current step
  bool bTwittleEnabled;      // run twiddle algorithm if true
  int index_;                // the current twiddle parameter index
  double threshold_;         // the threshold for twiddle success
  double avg_error_;         // average sum of squared error over time
  double best_error_;        // the current best error
  pid::twiddle_state state_; // the current twiddle state
  int max_steps_;            // maximum number of feedback updates (distance)
  int max_index_;            // max index to twiddle up to
  int best_step_; // largest number of updates achieved with this twiddle tune
  std::chrono::system_clock::time_point
      start_time_; // start time for the current run
  std::chrono::system_clock::time_point
      last_time_;         // the time of the last step
  std::ofstream log_fs_;  // contains run log and tuning results
  std::ofstream data_fs_; // error and pid control csv file for the run
  int runs_;              // current number of tune iterations
  std::string
      name_; // itentifier used in the log name for this particluar PID instance

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  *
  * @param Kp Proportional gain
  * @param Kd Differential gain
  * @param Ki Integral gain
  * @param enableTwiddle true if twiddling
  * @param thresh the sum of the potantials to exit twiddle tuning
  * @param name name prefix to use for logging
  */
  void Init(double Kp, double Kd, double Ki, bool enableTwiddle, float thresh,
            std::string name);

  /*
  * Update the PID error variables given cross track error.
  *
  * @param cte the sensor reading
  * @param crash true if vehicle crashed
  * @return the twiddle return code. Feedback for restarting the simulator.
  */
  pid::return_code UpdateError(double cte, bool crash);

  /*
   * Get the current control output
   *
   * @return the final control output for this step
   */
  double TotalError();

  /*
  * Print the twiddle success message with best tune results.
  *
  */
  void UpdateSuccess();

  /*
  * Prints the current twiddle results to log file
  */
  void PrintResults();

  /*
  * Prints the current twiddle gains to log file
  */
  void PrintCurrentGains();

  /*
  * Log the PID metrics to a csv file.
  *
  * @param sensor the sensor reading
  * @param p the current p error
  * @param d the current d error
  * @param i the current i error
  * @param output the current total PID output
  */
  void SaveData(double sensor, double p, double d, double i, double output);
};
#endif /* PID_H */
