#ifndef BODY_FOOT_CONTROL
#define BODY_FOOT_CONTROL

#include <Controller.hpp>
#include <Utils/BSplineBasic.h>

class WBDC;
class WBDC_ExtraData;
class WBDC_Task;
class WBDC_ContactSpec;

class BodyFootCtrl:public Controller{
public:
  BodyFootCtrl(int swing_foot);
  ~BodyFootCtrl();
  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(std::string setting_file_name);

  void setSwingTime(double swing_time){ end_time_ = swing_time; }
protected:
  int swing_foot_;
  bool b_compute_target_;
  double swing_height_;

  WBDC* wbdc_;
  WBDC_ExtraData* wbdc_data_;
  WBDC_Task* body_foot_task_;
  WBDC_ContactSpec* single_contact_;

  sejong::Vector body_pos_ini_;
  sejong::Vect3 ini_com_pos_;
  sejong::Vect3 ini_foot_pos_;
  sejong::Vect3 target_foot_pos_;

  BS_Basic<3, 3, 1, 2, 2> foot_traj_;

  double end_time_;
  void _task_setup();
  void _single_contact_setup();
  void _body_foot_ctrl(sejong::Vector & gamma);
};

#endif
