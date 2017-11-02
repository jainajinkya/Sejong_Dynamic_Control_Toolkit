#ifndef BODY_CONTRL_VALKYRIE
#define BODY_CONTRL_VALKYRIE

#include <Controller.hpp>

class WBDC;
class WBDC_ExtraData;
class WBDC_Task;
class WBDC_ContactSpec;

class BodyCtrl: public Controller{
public:
  BodyCtrl();
  virtual ~BodyCtrl();

  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(std::string setting_file_name);

protected:
  WBDC* wbdc_;
  WBDC_ExtraData* wbdc_data_;
  WBDC_Task* body_task_;
  WBDC_ContactSpec* foot_contact_;
  
  sejong::Vect3 body_pos_ini_;

  sejong::Vect2 body_lin_amp_;
  sejong::Vect2 body_lin_omega_;

  double end_time_;
  void _body_task_setup();
  void _foot_contact_setup();
  void _body_ctrl(sejong::Vector & gamma);
  void _jpos_ctrl(sejong::Vector & gamma);
};

#endif
