#ifndef VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM
#define VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM

class Reversal_LIPM_Planner{
public:
  Reversal_LIPM_Planner();
  ~Reversal_LIPM_Planner();

  void getNextFootLocation(double x, double xdot, double px, double swing_time, double t_prime, double & out_swing_time, double & out_next_px);
};

#endif
