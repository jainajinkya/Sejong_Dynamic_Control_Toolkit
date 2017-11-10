#ifndef BODY_FOOT_CONTROL
#define BODY_FOOT_CONTROL

#include <Controller.hpp>
#include <Utils/BSplineBasic.h>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>

class WBDC;
class WBDC_ExtraData;
class WBDC_Task;
class WBDC_ContactSpec;

class BodyFootPlanningCtrl:public Controller{
public:
  BodyFootPlanningCtrl(int swing_foot);
  ~BodyFootPlanningCtrl();
  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(std::string setting_file_name);

  void setSwingTime(double swing_time){ end_time_ = swing_time; }
  void setPrimeTimeX(double t_p_x){ t_prime_x_ = t_p_x; }
  void setPrimeTimeY(double t_p_y){ t_prime_y_ = t_p_y; }

protected:
  void _SetBspline(const sejong::Vect3 & st_pos,
                   const sejong::Vect3 & st_vel,
                   const sejong::Vect3 & st_acc,
                   const sejong::Vect3 & target_pos);

  int swing_foot_;
  bool b_update_target_;
  double swing_height_;
  double update_time_;
  sejong::Vect3 default_target_loc_;

  double planning_frequency_;
  int num_planning_;
  double t_prime_x_;
  double t_prime_y_;

  WBDC* wbdc_;
  WBDC_ExtraData* wbdc_data_;
  WBDC_Task* body_foot_task_;
  WBDC_ContactSpec* single_contact_;

  Reversal_LIPM_Planner planner_;
  void _CheckPlanning();
  void _Replanning();

  sejong::Vect3 ini_com_pos_;
  sejong::Vect3 curr_foot_pos_des_;
  sejong::Vect3 curr_foot_vel_des_;
  sejong::Vect3 curr_foot_acc_des_;
  sejong::Vect3 target_foot_pos_;

  BS_Basic<3, 3, 1, 2, 2> foot_traj_;

  double end_time_;
  void _task_setup();
  void _single_contact_setup();
  void _body_foot_ctrl(sejong::Vector & gamma);
};

#endif
