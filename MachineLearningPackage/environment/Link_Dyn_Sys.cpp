#include "Link_Dyn_Sys.h"
#include <stdio.h>


Linkage_System::Linkage_System():
  Dynamic_System(),
  dt_(0.01),
  des_pos_(-0.1),
  grav_(9.81),
  mass_(0.5)
{

}

bool Linkage_System::Transition(const double* state,
                                const double* action,
                                double & reward,
                                double* nx_state, bool b_print){
  nx_state[0] = state[0] + state[1] * dt_;
  // nx_state[1] = state[1] + (action[0]/mass_ - grav_) * dt_;
  nx_state[1] = state[1] + (action[0]/mass_) * dt_;

  reward =
    //- 0.001 * action[0]*action[0]
    - 1.*(des_pos_ - nx_state[0]) * (des_pos_ - nx_state[0])
    - 1. * nx_state[1] * nx_state[1];

  if(is_terminal(nx_state, action)){
    reward = -5.5;
    return true;
  }
  return false;
}
bool Linkage_System::is_terminal(const double* state, const double* action){
  if(state[0] < POS_MIN || state[0] > POS_MAX){
    printf("[Linkage System] Position hit the limit: %f\n", state[0]);
    return true;
  }
  if(state[1] < VEL_MIN || state[1] > VEL_MAX){
    printf("[Linkage System] Velocity hit the limit: %f\n", state[1]);
    return true;
  }
  return false;
}
