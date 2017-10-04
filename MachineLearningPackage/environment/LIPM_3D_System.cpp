#include "LIPM_3D_System.h"
#include <stdio.h>
#include <math.h>

LIPM_3D_System::LIPM_3D_System():
  leg_limit_(1.5),
  h_(1.005294),
  g_(9.81)
{
  omega_ = sqrt(g_/h_);
}

double LIPM_3D_System::_leglength(double x, double y) {
  return sqrt(x * x + y * y + h_ * h_);
}

// Assume xp, yp are zero
void LIPM_3D_System::_CalcState(double t,
                                const double* state,
                                double* nx_state){
  // printf("omega: %f \n", omega_);
  // X
  double x0(state[0]);
  double x0dot(state[2]);

  double Ax( 0.5 * ( x0 + 1./omega_ * x0dot) );
  double Bx( 0.5 * ( x0 - 1./omega_ * x0dot) );

  // x, xdot
  nx_state[0] = Ax * exp( omega_ * t) + Bx * exp(-omega_ * t);
  nx_state[2] = omega_ * (Ax * exp( omega_ * t) - Bx * exp(-omega_ * t));

  // Y
  double y0(state[1]);
  double y0dot(state[3]);

  double Ay( 0.5 * ( y0 + 1./omega_ * y0dot) );
  double By( 0.5 * ( y0 - 1./omega_ * y0dot) );

  // y, ydot
  nx_state[1] = Ay * exp( omega_ * t) + By * exp(-omega_ * t);
  nx_state[3] = omega_ * (Ay * exp( omega_ * t) - By * exp(-omega_ * t));
}

double LIPM_3D_System::_switching_state_pos(double x0, double x0dot, double xp_nx, double v_apex){
  double C ( pow(x0, 2.) + (pow(v_apex, 2.) - pow(x0dot, 2.))/pow(omega_, 2.) );
  double x_switch( 0.5 * (C / (xp_nx)  + xp_nx) );

  return x_switch;
}

void LIPM_3D_System::_compute_x_switching(double x, double xdot, double xp, double v_apex, double & t_switch, double & x_switch, double & xdot_switch){
  x_switch = _switching_state_pos(x, xdot, xp, v_apex);
  // if(x_switch > xp){
  //   printf("[LIPM 3D System] X_switch is larger than Xp: %f, %f \n", x_switch, xp);
  // }
  xdot_switch = _find_vel(x_switch, x, xdot);
  t_switch = _find_time(x_switch, xdot_switch, x, xdot);
}

double LIPM_3D_System::_find_time(double x, double xdot,
                           double x0, double x0dot){
  double A ( 0.5 * ( x0 + x0dot/omega_) );
  double time = 1./omega_ * log ( (x + xdot/omega_)/(2*A) );
  return time;
}

void LIPM_3D_System::_compute_y_switching(double t_switch, double t_apex, double y, double ydot, double ydot_des, double & yp, double & y_switch, double & ydot_switch){
  double A (0.5 * ( y + ydot/omega_) );
  double B (0.5 * ( y - ydot/omega_) );

  y_switch = A*exp(omega_ * t_switch) + B*exp(-omega_ * t_switch);
  ydot_switch = omega_ * ( A * exp(omega_ * t_switch) - B * exp (-omega_ * t_switch));
  yp = _find_yp(y_switch, ydot_switch, t_apex, ydot_des);
}

void LIPM_3D_System::_compute_y_switching(const double t_switch,
                                          const double y0,
                                          const double ydot0,
                                          double & y_switch,
                                          double & ydot_switch){
  double A (0.5 * ( y0 + ydot0/omega_) );
  double B (0.5 * ( y0 - ydot0/omega_) );

  y_switch = A*exp(omega_ * t_switch) + B*exp(-omega_ * t_switch);
  ydot_switch = omega_ * ( A * exp(omega_ * t_switch) - B * exp (-omega_ * t_switch));
}


double LIPM_3D_System::_find_yp( double y0, double y0dot, double t_apex,double ydot_des){
  double yp ( y0 - (1 + exp( 2* omega_ * t_apex))/ (1 - exp( 2* omega_ * t_apex)) * (y0dot/ omega_)  + 2 * ydot_des/(( exp(-omega_ * t_apex) - exp(omega_*t_apex)  ) * omega_) );
  return yp;
}

double LIPM_3D_System::_find_vel(double x, double x0, double x0dot){
  double sign(1.0);
  if(x0dot < 0) sign = -1.0;

  double xdot = sign * sqrt( pow(omega_,2) * ( pow(x, 2) - pow(x0, 2) ) + pow(x0dot, 2) );

  return xdot;
}
