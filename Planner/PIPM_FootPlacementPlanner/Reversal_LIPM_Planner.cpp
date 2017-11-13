#include "Reversal_LIPM_Planner.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Configuration.h>
#include <Eigen/Eigenvalues>

Reversal_LIPM_Planner::Reversal_LIPM_Planner():
  Planner(),
  com_vel_limit_(2),
  b_set_omega_(false)
{

}
Reversal_LIPM_Planner::~Reversal_LIPM_Planner(){
  
}
void Reversal_LIPM_Planner::_computeSwitchingState(
  double swing_time,
  const sejong::Vect3 & com_pos,  const sejong::Vect3 & com_vel,
  const sejong::Vect3 & stance_foot_loc,
  std::vector<sejong::Vect2> & switching_state){

  double A, B;
  for(int i(0); i<2; ++i){
    A = ((com_pos[i] - stance_foot_loc[i]) + com_vel[i]/omega_)/2.;
    B = ((com_pos[i] - stance_foot_loc[i]) - com_vel[i]/omega_)/2.;
    switching_state[i][0] = A * exp(omega_ * swing_time) + B * exp(-omega_ * swing_time) + stance_foot_loc[i];
    switching_state[i][1] = omega_ * ( A * exp(omega_ * swing_time) - B * exp(-omega_ * swing_time) );
  }
}

// global CoM pos
void Reversal_LIPM_Planner::getNextFootLocation(
                              const sejong::Vect3 & com_pos,
                              const sejong::Vect3 & com_vel,
                              sejong::Vect3 & target_loc,
                              const void* additional_input,
                              void* additional_output){
  if(!b_set_omega_){
    printf("[Reversal Planner] Omega is not set\n");
    exit(0);
  }
  ParamReversalPL* _input = ((ParamReversalPL*) additional_input);

  std::vector<sejong::Vect2> switch_state(2);
  _computeSwitchingState(_input-> swing_time, com_pos, com_vel, _input->stance_foot_loc, switch_state);

  for(int i(0); i<2; ++i){
    double exp_weight = (exp(omega_ * t_prime_[i]) + exp(-omega_ * t_prime_[i]))/(exp(omega_ * t_prime_[i]) - exp(-omega_ * t_prime_[i]));

    target_loc[i] = switch_state[i][0] + (switch_state[i][1]/omega_) * exp_weight + kappa_[i] * (_input->des_loc[i] - switch_state[i][0]);
  }
  target_loc[2] = 0.;

  // _StepLengthCheck(target_loc, switch_state);
  _StepLengthCheck(target_loc, _input->b_positive_sidestep, _input->stance_foot_loc);

}

void Reversal_LIPM_Planner::_StepLengthCheck(sejong::Vect3 & target_loc,
                                             bool b_positive_sidestep,
                                             const sejong::Vect3 & stance_foot){
  // X limit check
  double x_step_length (target_loc[0] - stance_foot[0]);
  if(x_step_length < x_step_length_limit_[0]){
    target_loc[0] = stance_foot[0] + x_step_length_limit_[0];
    printf("x step length hit min: %f\n", x_step_length);
    printf("new x step: %f, %f \n", target_loc[0], stance_foot[0]);
  }
  if(x_step_length > x_step_length_limit_[1]){
    target_loc[0] = stance_foot[0] + x_step_length_limit_[1];
    printf("x step length hit max: %f\n", x_step_length);
    printf("new x step: %f, %f \n", target_loc[0], stance_foot[0]);
  }

  // Y limit check
  double y_step_length (target_loc[1] - stance_foot[1]);
  if(b_positive_sidestep){ // move to left
    if(y_step_length < y_step_length_limit_[0]){
      target_loc[1] = stance_foot[1] + y_step_length_limit_[0];
      printf("y step length hit min: %f\n", y_step_length);
      printf("new y step: %f, %f \n", target_loc[1], stance_foot[1]);
    }

    if(y_step_length > y_step_length_limit_[1]){
      target_loc[1] = stance_foot[1] + y_step_length_limit_[1];
      printf("y step length hit max: %f\n", y_step_length);
      printf("new y step: %f, %f \n", target_loc[1], stance_foot[1]);
    }

  } else { // move to right
    if(-y_step_length < y_step_length_limit_[0]){
      target_loc[1] = stance_foot[1] - y_step_length_limit_[0];
      printf("y step length hit min: %f\n", y_step_length);
      printf("new y step: %f, %f \n", target_loc[1], stance_foot[1]);
    }

    if(-y_step_length > y_step_length_limit_[1]){
      target_loc[1] = stance_foot[1] - y_step_length_limit_[1];
      printf("y step length hit max: %f\n", y_step_length);
      printf("new y step: %f, %f \n", target_loc[1], stance_foot[1]);
    }

  }
}

void Reversal_LIPM_Planner::_StepLengthCheck(sejong::Vect3 & target_loc, const std::vector<sejong::Vect2> & switch_state){
  // X limit check
  double x_step_length (target_loc[0] - switch_state[0][0]);
  if(x_step_length < x_step_length_limit_[0]){
    target_loc[0] = switch_state[0][0] + x_step_length_limit_[0];
    printf("x step length hit minimum: %f\n", x_step_length);
    printf("new x step: %f, %f \n", target_loc[0], switch_state[0][0]);
  }
  if(x_step_length > x_step_length_limit_[1]){
    target_loc[0] = switch_state[0][0] + x_step_length_limit_[1];
    printf("x step length hit max: %f\n", x_step_length);
    printf("new x step: %f, %f \n", target_loc[0], switch_state[0][0]);
  }


  // Y limit check
  double y_step_length (target_loc[1] - switch_state[1][0]);
  if(switch_state[1][1] > 0){ // move to left
    if(y_step_length < y_step_length_limit_[0]){
      target_loc[1] = switch_state[1][0] + y_step_length_limit_[0];
    }

    if(y_step_length > y_step_length_limit_[1]){
      target_loc[1] = switch_state[1][0] + y_step_length_limit_[1];
    }

  } else { // move to right
    if(-y_step_length < y_step_length_limit_[0])
      target_loc[1] = switch_state[1][0] - y_step_length_limit_[0];
    if(-y_step_length > y_step_length_limit_[1])
      target_loc[1] = switch_state[1][0] - y_step_length_limit_[1];
  }
}

void Reversal_LIPM_Planner::PlannerInitialization(const std::string & file){
  ParamHandler handler(CONFIG_PATH + file + ".yaml");

  handler.getVector("t_prime", t_prime_);
  handler.getVector("kappa", kappa_);

  handler.getVector("x_step_length_limit", x_step_length_limit_);
  handler.getVector("y_step_length_limit", y_step_length_limit_);
  handler.getVector("com_velocity_limit", com_vel_limit_);

  printf("[Reversal Planner] Parameter Setup is completed\n");
}

void Reversal_LIPM_Planner::CheckEigenValues(double swing_time){
  sejong::Matrix A(2,2);
  printf("omega, swing_time: %f, %f\n", omega_, swing_time);

  for(int i(0); i<2; ++i){
    double coth = cosh(omega_*t_prime_[i])/ sinh(omega_*t_prime_[i]);

    A(0, 0) = 1 - kappa_[i] + kappa_[i] * cosh(omega_ * swing_time);
    A(0, 1) = (sinh(omega_*swing_time) + (1. - cosh(omega_*swing_time))* coth)/omega_;
    A(1, 0) = kappa_[i] * omega_ * sinh(omega_ * swing_time);
    A(1, 1) = cosh(omega_ * swing_time) - sinh(omega_ * swing_time) * coth;

    Eigen::VectorXcd eivals = A.eigenvalues();
    printf("%d - axis eigen value:\n", i);
    std::cout<<eivals<<std::endl;
  }

}
