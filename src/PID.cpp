#include "PID.h"
#include <iostream>
#include <limits>

using namespace std;

#define SETTLE_STEPS    100
#define ACCUM_STEPS     200

PID::PID() : is_firsttime(true), to_twiddle(false) {
  p            = { 0.15, 0.0, 3.0 };
  dp           = { 0.01, 0.0, 0.01 };
  best_error   = std::numeric_limits<double>::max();
  sum_of_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool to_twiddle) {
  this->Kp         = Kp;
  this->Ki         = Ki;
  this->Kd         = Kd;
  this->to_twiddle = to_twiddle;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::Twiddle() {
  static const double tolerance = 0.002;
  static int curr_index         = 0;
  static int state              = 0;

  // current mean square error
  double curr_err = sum_of_error / ACCUM_STEPS;

  // stop twiddling if it is good enough
  if ((dp[0] + dp[1] + dp[2]) < tolerance) {
    std::cout << "final: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    return;
  }

  if (state == 0) {
    best_error = curr_err;

    // first kick
    curr_index     = 0;
    p[curr_index] += dp[curr_index];
    state          = 1;
  } else if (state == 1) {
    if (curr_err < best_error) {
      best_error = curr_err;

      // we can go a bit more aggressive next time for this paramter.
      dp[curr_index] *= 1.1;

      // Twiddle next parameter
      curr_index     = (++curr_index) % 3;
      p[curr_index] += dp[curr_index];
      state          = 1;
    } else {
      // Tune in another direction for the same parameter
      p[curr_index] -= (2 * dp[curr_index]);
      state          = -1;
    }
  } else if (state == -1) {
    if (curr_err < best_error) {
      best_error = curr_err;

      // we can go a bit more aggressive next time for this paramter.
      dp[curr_index] *= 1.1;
    } else {
      // set back to original value
      p[curr_index] += dp[curr_index];

      // make it a bit less aggressive
      dp[curr_index] *= 0.9;
    }

    // Twiddle next parameter
    curr_index     = (++curr_index) % 3;
    p[curr_index] += dp[curr_index];
    state          = 1;
  }

  std::cout << "pid: " << p[0] << ", " << p[1] << ", " << p[2] <<
  "\tbest_error: " << best_error << std::endl;
  std::cout << "d_pid: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
}

void PID::UpdateError(double cte) {
  static int count = 0;

  if (is_firsttime) {
    p_error      = cte;
    is_firsttime = false;
  }

  d_error  = cte - p_error;
  p_error  = cte;
  i_error += cte;

  if (to_twiddle) {
    if (count < SETTLE_STEPS) {
      // wait a bit to let current parameters settle down
      count++;
    } else if (count < (SETTLE_STEPS + ACCUM_STEPS)) {
      // collect error for next twiddle
      sum_of_error += cte * cte;

      if (count == (SETTLE_STEPS + ACCUM_STEPS - 1)) {
        Twiddle();

        // reset for next twiddle
        count        = 0;
        sum_of_error = 0;
      } else {
        count++;
      }
    }
  }
}

double PID::Update() const {
  double ret;

  if (!to_twiddle) {
    ret = (-Kp * p_error - Kd * d_error - Ki * i_error);
  } else {
    ret = (-p[0] * p_error - p[2] * d_error - p[1] * i_error);
  }
  return ret;
}

double PID::TotalError() {
  return 0;
}
