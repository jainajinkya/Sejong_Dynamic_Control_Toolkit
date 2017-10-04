#include "LIPM_3D_Act_Only.h"
#include <Utils/utilities.h>

LIPM_3D_Act_Only::LIPM_3D_Act_Only():
  LIPM_3D(),
  vel_des_(2)
{
  vel_des_[0] = 0.4;
  vel_des_[1] = 0.0;
}

LIPM_3D_Act_Only::~LIPM_3D_Act_Only(){
}

bool LIPM_3D_Act_Only::Transition(const sejong::Vector & state,
                                  const sejong::Vector & action,
                                  double & reward,
                                  sejong::Vector & nx_state){
  double t_switch(action[0]);
  double xp(action[1]);
  double yp(action[2]);

  sejong::Vector foot(2);
  foot.setZero();
  _CalcState(t_switch, state, foot, nx_state);
  // Capture Point y direction
  // double y_cp = nx_state[1] + sqrt(h_/ 9.81) * nx_state[3];
  // sejong::pretty_print(nx_state, std::cout, "nx state");
  // printf("capture y: %f \n", y_cp);
  reward = 0.;
  // if ( y_cp > 0.){
  //   if( yp < y_cp ){
  //     reward = -13.0;
  //     return true;
  //   }
  // } else {
  //   if ( yp > y_cp){
  //     reward = -13.0;
  //     return true;
  //   }
  // }

  double leg_length_switch(_leglength(nx_state[0], nx_state[1]));
  double leg_length_nx(_leglength(nx_state[0]-xp, nx_state[1] - yp) );

  nx_state[0] -= xp;
  nx_state[1] -= yp;

  if(leg_length_switch > leg_limit_) {
    reward = -20.0;
    return true;
  }
  if(leg_length_nx > leg_limit_) {
    reward = -20.0;
    return true;
  }
  if (t_switch < 0.6){
    reward = -10.;
    return true;
  }
  double vel_error = Vel_error_at_MinLength(nx_state, vel_des_);
  // reward = t_switch / sqrt( pow(xp, 2) + pow(yp, 2)) - vel_error;
  // reward = -vel_error + t_switch / sqrt( pow(xp, 2) + pow(yp, 2));

  reward =  - vel_error - sqrt( pow(xp, 2) + pow(yp, 2))/t_switch;
  // reward =  - vel_error;
  // reward =   - sqrt( pow(xp, 2) + pow(yp, 2))/t_switch;

  return false;
}
