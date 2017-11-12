#ifndef JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL
#define JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL

#include <Controller.hpp>

class WBDC;
class WBDC_ExtraData;
class WBDC_Task;
class WBDC_ContactSpec;

class JPosTargetCtrl: public Controller{
public:
  JPosTargetCtrl();
  virtual ~JPosTargetCtrl();

  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setMovingTime(double time) { end_time_ = time; }
  void setTargetPosition(const std::vector<double> & jpos);
protected:
  double end_time_;

  WBDC* wbdc_;
  WBDC_ExtraData* wbdc_data_;
  WBDC_Task* jpos_task_;
  WBDC_ContactSpec* fixed_body_contact_;

  sejong::Vector jpos_ini_;
  sejong::Vector jpos_target_;
  
  void _jpos_task_setup();
  void _fixed_body_contact_setup();
  void _jpos_ctrl(sejong::Vector & gamma);
};

#endif
