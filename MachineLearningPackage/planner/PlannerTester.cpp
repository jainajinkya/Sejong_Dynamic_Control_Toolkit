#include "PlannerTester.h"

#include <environment/LIPM_3D.h>
#include <Utils/utilities.h>

PlannerTester::PlannerTester(const sejong::Vector & ini,
                             LIPM_3D* env):
  ini_state_(6),
  pipm_env_(env),
  num_step_(5)
{
  ini_state_[0] = ini[0];
  ini_state_[1] = ini[1];
  ini_state_[2] = pipm_env_->GetHeight();
  ini_state_[3] = ini[2];
  ini_state_[4] = ini[3];
  ini_state_[5] = 0.;


  planner_ = new Planner_Analytic();
  param_ = new AnalyticPL_Param();
  terrain_ = new FlatTerrain();
  com_surf_ = new ConstHeightSurface();
}

void PlannerTester::check(){
  _SetParam();
  planner_->DoLocomotionPlanning(terrain_, com_surf_, param_);
  _TestFunction();
}

void PlannerTester::_SetParam(){
  (dynamic_cast<ConstHeightSurface*>(com_surf_))->SetCoMHeight(ini_state_[2]);

  AnalyticPL_Param* ana_param = static_cast<AnalyticPL_Param*> (param_);


  // CoM ini
  ana_param->curr_com_state = ini_state_;

  // Curr Foot pos
  sejong::Vector ini_foot(3);
  ini_foot.setZero();
  ana_param->fixed_pivot = ini_foot;

  // (xp, apex_vel)
  std::vector<double> xp_list(num_step_);
  std::vector<double> apex_vel_list(num_step_);
  for(int i(0); i<num_step_; ++i){
    xp_list[i] = 0.14 + 0.14 * i;
    apex_vel_list[i] = 0.2;
  }
  ana_param->xp_list = xp_list;
  ana_param->apex_vel_list = apex_vel_list;

  // Is initial?
  ana_param->is_initial = false;
}

void PlannerTester::_TestFunction(){
  sejong::Vector com_state;
  sejong::Vector foot_pos;
  double dt(0.003);
  int count(0);
  double curr_time;
  for(int i(0); i<num_step_; ++i){
    count = 0;
    planner_->GetFootPlacement(i, foot_pos);
    sejong::saveVector(foot_pos, "foot_pos");

    while (count * dt <  planner_->GetFinTime(i)  ){
      curr_time = count * dt;
      planner_->GetCoMState(i, curr_time, com_state);
      sejong::saveVector(com_state, "com_path");
      ++count;
      if(count > 10000){
        break;
      }
    }
  }
  // exit(0);
}
