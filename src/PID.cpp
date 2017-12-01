#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  first_measurement = true;
  total_err = 0;
}

void PID::UpdateError(double cte) {

  if (first_measurement){
    first_measurement = false;
    cte_prev = cte;
  }

  p_error = cte;
  i_error = i_error+cte;
  d_error = cte-cte_prev;
  cte_prev = cte;

  total_err = total_err +  cte*cte;
}

double PID::TotalError() {

  return total_err;
}

