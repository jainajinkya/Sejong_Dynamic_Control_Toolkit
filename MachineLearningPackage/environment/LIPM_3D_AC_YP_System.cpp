#include "LIPM_3D_AC_YP_System.h"
#include <stdio.h>
#include <math.h>

#define ERROR_MESSAGE_PRINT 0

LIPM_3D_AC_YP_System::LIPM_3D_AC_YP_System():
  LIPM_3D_System(),
  v_des_(0.2),
  terminal_reward_(-5.0)
  // terminal_reward_(-15.0) //keep this number (good)
  // terminal_reward_(-20.0)
{
}

LIPM_3D_AC_YP_System::~LIPM_3D_AC_YP_System(){}

bool LIPM_3D_AC_YP_System::Transition(const double* state,
                                    const double* action,
                                    double & reward,
                                    double* nx_state, bool b_print){
  double x = 0.;
  double y = state[0];
  double xdot = state[1];
  double ydot = state[2];

  double act_xp = action[0];
  double act_v_apex = action[1];
  double act_yp = action[2];

  yp_ = act_yp;

  double x_switch, xdot_switch;
  // Switching State
  _compute_x_switching(x, xdot, act_xp, act_v_apex, t_switch_, x_switch, xdot_switch);
  double t_apex = _find_time(0., act_v_apex, x_switch-act_xp, xdot_switch);

  // Checking Required Condition
  if(t_apex < 0.12){
    reward = terminal_reward_;
    return true;
  }

  // Compute y switching from yp
  double y_switch, ydot_switch;
  _compute_y_switching(t_switch_, y, ydot, y_switch, ydot_switch);

  double full_nx_state[4];
  double full_switch_state[4];
  full_switch_state[0] = x_switch - act_xp;
  full_switch_state[1] = y_switch - act_yp;
  full_switch_state[2] = xdot_switch;
  full_switch_state[3] = ydot_switch;

  _CalcState(t_apex, full_switch_state, full_nx_state);

  nx_state[0] = -full_nx_state[1];
  nx_state[1] = full_nx_state[2];
  nx_state[2] = -full_nx_state[3];


  if(_is_terminal(t_switch_, act_xp, act_yp)){
    reward = terminal_reward_ ;
    return true;
  }

  // Reward
  double des_step_size = 0.3;
  double forward_velocity = act_xp/(t_switch_ + t_apex);
  double nx_ydot = nx_state[2];

  if(fabs(nx_ydot) > 1.5){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] y vel is too large: %f\n\n", nx_ydot);
#endif
    reward = terminal_reward_;
    return true;
  }

  reward = 0.
    - 0.0 * ((v_des_ - act_v_apex)*(v_des_ - act_v_apex))
    - 0. * ((act_yp - des_step_size)*(act_yp - des_step_size))
    - 100.0 * (nx_ydot * nx_ydot);

  return false;
}

bool LIPM_3D_AC_YP_System::_is_terminal(double t_switch, double xp, double yp){
  // T_switch limitation
  if( t_switch < 0.12){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] switching time is too short: %f\n\n", t_switch);
#endif
    return true;
  }

  // Kinematic limitation
  double stride_len = sqrt(xp*xp + yp*yp);

  if(yp> 0.6){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM ydot] yp is too long: %f\n\n", yp);
#endif
    return true;
  }

  if(yp< 0.05){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM ydot] yp is too short: %f\n\n", yp);
#endif
    return true;
  }

  if(stride_len< 0.20){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM ydot] stride length is too short: %f\n\n", stride_len);
#endif
    return true;
  }

  if(stride_len> 0.95){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM ydot] stride length is too long: %f\n\n", stride_len);
#endif
    return true;
  }
  return false;
}
