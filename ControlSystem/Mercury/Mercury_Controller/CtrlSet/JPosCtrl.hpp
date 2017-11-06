#ifndef JOINT_POSITION_CTRL
#define JOINT_POSITION_CTRL

#include <Controller.hpp>

class WBDC;
class WBDC_ExtraData;
class WBDC_Task;
class WBDC_ContactSpec;

class JPosCtrl: public Controller{
public:
  JPosCtrl();
  virtual ~JPosCtrl();

  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(std::string setting_file_name);

protected:
  WBDC* wbdc_;
  WBDC_ExtraData* wbdc_data_;
  WBDC_Task* jpos_task_;
  WBDC_ContactSpec* fixed_body_contact_;

  sejong::Vector jpos_ini_;

  void _jpos_task_setup();
  void _fixed_body_contact_setup();
  void _jpos_ctrl(sejong::Vector & gamma);
};

#endif
