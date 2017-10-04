#include "LIPM_3D_AC_RBF_System.h"
#include <stdio.h>
#include <math.h>

#define ERROR_MESSAGE_PRINT 0

LIPM_AC_RBF_System::LIPM_AC_RBF_System():
  LIPM_3D_System(),
  v_des_(0.2),
  terminal_reward_(-10.0)
  // terminal_reward_(-15.0) //keep this number (good)
  // terminal_reward_(-20.0)
{
}

LIPM_AC_RBF_System::~LIPM_AC_RBF_System(){}

bool LIPM_AC_RBF_System::Transition(const double* state,
                                    const double* action,
                                    double & reward,
                                    double* nx_state, bool b_print){
  double x = 0.;
  double y = state[0];
  double xdot = state[1];
  double ydot = state[2];

  double xp = action[0];
  double v_apex = action[1];
  double x_switch, xdot_switch;
  // Switching State
  _compute_x_switching(x, xdot, xp, v_apex, t_switch_, x_switch, xdot_switch);
  double t_apex = _find_time(0., v_apex, x_switch-xp, xdot_switch);

  // Checking Required Condition
  if(t_apex < 0.12){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] Apex Time is Too short: %f \n", t_apex);
#endif
    reward = terminal_reward_;
    return true;
  }
  // Compute yp
  double y_switch, ydot_switch;
  _compute_y_switching(t_switch_, t_apex, y, ydot, 0., yp_, y_switch, ydot_switch);

  // Switching Velocity must be positive
  if(ydot_switch < 0.){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] Y Switching Velocity is Negative: %f \n", ydot_switch);
#endif
    reward = terminal_reward_;
    return true;
  }
  if(b_print){
    printf("switch state: (%f, %f, %f, %f, [t_apex]: %f) \n", x_switch, y_switch, xdot_switch, ydot_switch, t_apex);
    printf("t switch, xp, yp: (%f, %f, %f) \n", t_switch_, xp, yp_);
  }

  double full_nx_state[4];
  double full_switch_state[4];
  full_switch_state[0] = x_switch - xp;
  full_switch_state[1] = y_switch - yp_;
  full_switch_state[2] = xdot_switch;
  full_switch_state[3] = ydot_switch;

  _CalcState(t_apex, full_switch_state, full_nx_state);
  // Check
  // printf("nx state: %f, %f, %f, %f \n", full_nx_state[0], full_nx_state[1], full_nx_state[2], full_nx_state[3]);

  nx_state[0] = -full_nx_state[1];
  nx_state[1] = full_nx_state[2];
  nx_state[2] = -full_nx_state[3];

  if(_is_terminal(t_switch_, xp, yp_)){
    reward = terminal_reward_ ;
    return true;
  }
  // Reward
  double swing_speed = sqrt(xp*xp + yp_*yp_)/t_switch_;
  double des_step_size = 0.3;
  double forward_velocity = xp/(t_switch_ + t_apex);
  // reward = -sqrt((v_des_-v_apex)*(v_des_-v_apex)) - swing_speed;
  // reward = -((v_des_-v_apex)*(v_des_-v_apex)) - 15. * (sqrt(xp*xp + yp_*yp_) - des_step_size) * (sqrt(xp*xp + yp_*yp_) - des_step_size);
  // reward = -sqrt((v_des_-v_apex)*(v_des_-v_apex)) - sqrt(xp*xp + yp_*yp_);
  // reward = -1.0 * sqrt((v_des_-v_apex)*(v_des_-v_apex)) - 100.*sqrt((yp_-des_step_size)*(yp_-des_step_size));// + 0.5 * num_step_;
  reward = - 1. * ((v_des_-v_apex)*(v_des_-v_apex))
           - 10.*((yp_-des_step_size)*(yp_-des_step_size));// + 0.5 * num_step_;
  // printf("reward: %f\n", reward);
  // reward = -sqrt((v_des_-forward_velocity)*(v_des_-forward_velocity)) - 20.*sqrt((yp_-des_step_size)*(yp_-des_step_size));
  // reward = -7.*((v_des_-forward_velocity)*(v_des_-forward_velocity)) - 30.*((yp_-des_step_size)*(yp_-des_step_size));

  // reward = -sqrt((v_des_-forward_velocity)*(v_des_-forward_velocity)) - sqrt((nx_state[0]-0.1)*(nx_state[0]-0.1));

  // reward = -sqrt((v_des_-forward_velocity)*(v_des_-forward_velocity)) - (exp(5.*fabs(yp_-des_step_size))-1);


  // reward = -sqrt((v_des_-v_apex)*(v_des_-v_apex));// -  sqrt((yp_ - des_step_size) * (yp_ - des_step_size));

  // reward =  - sqrt((yp_ - des_step_size) * (yp_ - des_step_size));

  // reward = - ((yp_ - des_step_size) * (yp_ - des_step_size));

  // reward =  - swing_speed;
  // reward = -sqrt((v_des_-v_apex)*(v_des_-v_apex)) - sqrt( (yp-0.5)*(yp-0.5) );
  // reward = - sqrt( (yp-0.5)*(yp-0.5) );

  // reward = -yp;
  // reward = v_apex;

  return false;
}

bool LIPM_AC_RBF_System::_is_terminal(double t_switch, double xp, double yp){
  // T_switch limitation
  if( t_switch < 0.12){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] switching time is too short: %f\n", t_switch);
#endif
    return true;
  }
  // Kinematic limitation
  double stride_len = sqrt(xp*xp + yp*yp);
  // if(stride_len> 0.55){
  //   printf("[LIPM RBF] stride length is too long: %f\n", stride_len);
  //   return true;
  // }
  // if(stride_len< 0.23){
  //   printf("[LIPM RBF] stride length is too short: %f\n", stride_len);
  //   return true;
  // }
  // TEST
  if(yp> 0.65){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] yp is too long: %f\n", yp);
#endif
    return true;
  }
  if(yp< 0.15){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] yp is too short: %f\n", yp);
#endif
    return true;
  }

  if(stride_len> 0.85){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] stride length is too long: %f\n", stride_len);
#endif
    return true;
  }
//   if(stride_len< 0.33){
// #if ERROR_MESSAGE_PRINT
//     printf("[LIPM RBF] stride length is too short: %f\n", stride_len);
// #endif
//     return true;
//   }

  // Swing Velocity
  if(stride_len/t_switch > 5.2){
#if ERROR_MESSAGE_PRINT
    printf("[LIPM RBF] swing speed is too high: %f\n", stride_len/t_switch);
#endif
    return true;
  }
  return false;
}
