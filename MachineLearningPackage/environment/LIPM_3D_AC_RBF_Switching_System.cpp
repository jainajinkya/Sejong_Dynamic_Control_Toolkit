#include "LIPM_3D_AC_RBF_Switching_System.h"
#include <stdio.h>
#include <math.h>

LIPM_AC_RBF_Switching_System::LIPM_AC_RBF_Switching_System():
  LIPM_3D_System(),
  v_des_(0.25),
  terminal_reward_(-8.0)
{
}

LIPM_AC_RBF_Switching_System::~LIPM_AC_RBF_Switching_System(){}

bool LIPM_AC_RBF_Switching_System::Transition(const double* state,
                                    const double* action,
                                    double & reward,
                                              double* nx_state, bool b_print){
  double x = state[0];
  double y = state[1];
  double xdot = state[2];
  double ydot = state[3];

  double xp = action[0];
  double v_apex = action[1];
  double t_switch, x_switch, xdot_switch;
  // Switching State
  _compute_x_switching(x, xdot, xp, v_apex, t_switch, x_switch, xdot_switch);
  // Checking Required Condition
  if(t_switch < 0.){
    printf("[LIPM RBF] switching time is Negative: %f \n", t_switch);
    reward = terminal_reward_;
    return true;
  }

  double t_apex = _find_time(0., v_apex, x_switch-xp, xdot_switch);

  // Checking Required Condition
  if(t_apex < 0.){
    printf("[LIPM RBF] apex time is Negative: %f \n", t_apex);
    reward = terminal_reward_;
    return true;
  }

  if(xp < 0.){
    printf("[LIPM RBF] Foot Step is Negative: %f \n", xp);
    reward = terminal_reward_;
    return true;
  }
  // Compute yp
  double yp, y_switch, ydot_switch;
  _compute_y_switching(t_switch, t_apex, y, ydot, 0., yp, y_switch, ydot_switch);

  // Switching Velocity must be positive
  if(ydot_switch < 0.){
    printf("[LIPM RBF] Y Switching Velocity is Negative: %f \n", ydot_switch);
    reward = terminal_reward_;
    return true;
  }

  printf("switch state: (%f, %f, %f, %f, [t_apex]: %f) \n", x_switch, y_switch, xdot_switch, ydot_switch, t_apex);
  printf("t switch, xp, yp: (%f, %f, %f) \n", t_switch, xp, yp);

  double full_nx_state[4];
  double full_switch_state[4];

  full_nx_state[0] = x_switch - xp;
  full_nx_state[1] = y_switch - yp;
  full_nx_state[2] = xdot_switch;
  full_nx_state[3] = ydot_switch;

  // Check
  printf("nx state: %f, %f, %f, %f \n", full_nx_state[0], full_nx_state[1], full_nx_state[2], full_nx_state[3]);

  nx_state[0] = full_nx_state[0];
  nx_state[1] = -full_nx_state[1];
  nx_state[2] = full_nx_state[2];
  nx_state[3] = -full_nx_state[3];

  if(_is_terminal(t_switch, xp, yp)){
    reward = terminal_reward_ ;
    return true;
  }
  // Reward
  double swing_speed = sqrt(xp*xp + yp*yp)/t_switch;

  // reward = -sqrt((v_des_-v_apex)*(v_des_-v_apex)) - swing_speed;
  reward = -sqrt((v_des_-v_apex)*(v_des_-v_apex)) - 1.5*sqrt(xp*xp + yp*yp);
  // reward =  - swing_speed;
  // reward = -sqrt((v_des_-v_apex)*(v_des_-v_apex)) - sqrt( (yp-0.5)*(yp-0.5) );
  // reward = - sqrt( (yp-0.5)*(yp-0.5) );

  // reward = -yp;
  // reward = v_apex;

  return false;
}

bool LIPM_AC_RBF_Switching_System::_is_terminal(double t_switch, double xp, double yp){
  // T_switch limitation
  if( t_switch < 0.55){
    printf("[LIPM RBF] switching time is too short: %f\n", t_switch);
    return true;
  }
  // Kinematic limitation
  double stride_len = sqrt(xp*xp + yp*yp);
  if(stride_len> 0.56){
    printf("[LIPM RBF] stride length is too long: %f\n", stride_len);
    return true;
  }
  if(stride_len< 0.22){
    printf("[LIPM RBF] stride length is too short: %f\n", stride_len);
    return true;
  }

  // Swing Velocity
  if(stride_len/t_switch > 1.8){
    printf("[LIPM RBF] swing speed is too high: %f\n", stride_len/t_switch);
    return true;
  }
  return false;
}
