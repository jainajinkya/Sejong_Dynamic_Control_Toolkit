#ifndef LIPM_3D_SYSTEM_H
#define LIPM_3D_SYSTEM_H

#include "Dyn_System.h"

// Linear Inverted Pendulm
class LIPM_3D_System: public Dynamic_System{
public:
  LIPM_3D_System();
  virtual ~LIPM_3D_System(){}

  double GetHeight() {return h_; }
  // For test;
  int num_step_;
protected:
  double _leglength(double x, double y);

  // state: (x, y, xdot, ydot)
  // x: x0 - xp: assume that xp0 = 0
  void _compute_x_switching(double x, double xdot, double xp, double v_apex, double & t_switch, double & x_switch, double & xdot_switch);

  void _compute_y_switching(double t_switch, double t_apex,
                            double y, double ydot, double ydot_des,
                            double & yp, double & y_switch,
                            double & ydot_switch);

  void _compute_y_switching(const double t_switch,
                            const double y0,
                            const double ydot0,
                            double & y_switch,
                            double & ydot_switch);

  double _switching_state_pos(double x0, double x0dot, double xp_nx, double v_apex);
  double _find_yp( double y0, double y0dot, double t_apex, double ydot_des);
  double _find_vel(double x, double x0, double x0dot);
  double _find_time(double x, double xdot,
                    double x0, double x0dot);

  void _CalcState(double t,
                  const double* state,
                  double* nx_state);

  double leg_limit_;
  double h_;
  double g_;
  double omega_;
};

#endif
