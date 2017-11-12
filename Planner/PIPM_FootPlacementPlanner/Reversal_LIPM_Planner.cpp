#include "Reversal_LIPM_Planner.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Configuration.h>


Reversal_LIPM_Planner::Reversal_LIPM_Planner():
  Planner(),
  com_vel_limit_(2),
  b_set_omega_(false)
{

}
Reversal_LIPM_Planner::~Reversal_LIPM_Planner(){
  
}
void Reversal_LIPM_Planner::_computeSwitchingState(double swing_time,
   const sejong::Vect3& com_pos,  const sejong::Vect3& com_vel,
   sejong::Vect2 & switching_x_state, sejong::Vect2 & switching_y_state){


}

void Reversal_LIPM_Planner::getNextFootLocation(
                              const sejong::Vect3 & com_pos,
                              const sejong::Vect3 & com_vel,
                              sejong::Vect3 & target_pos,
                              const void* additional_input,
                              void* additional_output){
  if(!b_set_omega_){
    printf("[Reversal Planner] Omega is not set\n");
    exit(0);
  }
  ParamReversalPL* _input = ((ParamReversalPL*) additional_input);

  sejong::Vect2 switch_state_x;
  sejong::Vect2 switch_state_y;
  _computeSwitchingState(_input-> swing_time, com_pos, com_vel,
                         switch_state_x, switch_state_x);

}


void Reversal_LIPM_Planner::PlannerInitialization(const std::string & file){
  std::vector<double> tmp_vec;
  ParamHandler handler(CONFIG_PATH + file + ".yaml");
  
  handler.getValue("t_prime_x", t_prime_x_);
  handler.getValue("t_prime_y", t_prime_y_);

  handler.getValue("kappa_x", kappa_x_);
  handler.getValue("kappa_y", kappa_y_);

  handler.getValue("step_length_limit", step_length_limit_);
  handler.getVector("com_velocity_limit", com_vel_limit_);
}
