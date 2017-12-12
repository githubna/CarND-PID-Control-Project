#ifndef PID_H
#define PID_H

#include <array>

class PID {
public:

  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

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
   */
  void Init(double Kp,
            double Ki,
            double Kd,
            bool   to_twiddle = false);

  /*
   * Update the PID error variables given cross track error.
   */
  void                         UpdateError(double cte);

  /*
   * Calculate pid control value
   */
  double                       Update() const;

  /*
   * Calculate the total PID error.
   */
  double                       TotalError();

  const std::array<double, 3>& GetTunnedParams() const {
    return p;
  }

private:

  /*
   * Twiddle pid parameters
   */
  void Twiddle();
  
  bool is_firsttime;

  // whether tuning the pid parameters
  bool to_twiddle;

  double sum_of_error;

  double best_error;

  /**
   * Twiddle step parameters
   */
  std::array<double, 3> dp;
  std::array<double, 3> p;
};

#endif /* PID_H */
