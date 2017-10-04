#ifndef LIPM_3D_H
#define LIPM_3D_H

#include "Environment.h"

// Linear Inverted Pendulm
class LIPM_3D: public Environment{
public:
  LIPM_3D();
  virtual ~LIPM_3D(){}

  double Time_Until_HittingKinLimit(const sejong::Vector & state);
  double Vel_error_at_MinLength(const sejong::Vector & state,
                                const sejong::Vector & vel_des_);

  void getState(const sejong::Vector & ini_state,
                const double & time,
                sejong::Vector & curr_state);

  double GetHeight() {return h_; }
protected:
  double _leglength(double x, double y);
  void _CalcState(double t,
                  const sejong::Vector & state,
                  const sejong::Vector & foot,
                  sejong::Vector & nx_state);
  // x: x0 - xp: assume that xp0 = 0
  void _compute_x_switching(double x, double xdot, double xp, double v_apex, double & t_switch, double & x_switch, double & xdot_switch);

  void _compute_y_switching(double t_switch, double t_apex,
                            double y, double ydot,
                            double ydot_des,
                            double & yp, double & y_switch, double & ydot_switch);

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
