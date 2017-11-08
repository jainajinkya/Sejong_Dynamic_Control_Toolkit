#ifndef TRANSITION_CONTROLLER
#define TRANSITION_CONTROLLER

#include <Controller.hpp>
class WBDC;
class WBDC_ExtraData;
class WBDC_Task;
class WBDC_ContactSpec;

class TransitionCtrl: public Controller{
public:
  TransitionCtrl(int moving_foot);
  virtual ~TransitionCtrl();

  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(std::string setting_file_name);

protected:
  int moving_foot_;
  bool b_increase_; // Increasing or decreasing reaction force

  WBDC* wbdc_;
  WBDC_ExtraData* wbdc_data_;
  WBDC_Task* body_task_;
  WBDC_ContactSpec* double_contact_;

  sejong::Vector body_pos_ini_;
  sejong::Vect3 ini_com_pos_;

  double end_time_;
  void _body_task_setup();
  void _double_contact_setup();
  void _body_ctrl(sejong::Vector & gamma);

};
#endif
