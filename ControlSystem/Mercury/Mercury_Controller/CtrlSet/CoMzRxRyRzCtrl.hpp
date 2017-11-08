#ifndef COM_HEIGHT_RX_RY_RZ_CONTROL
#define COM_HEIGHT_RX_RY_RZ_CONTROL

#include <Controller.hpp>
class WBDC;
class WBDC_ExtraData;
class WBDC_Task;
class WBDC_ContactSpec;

class CoMzRxRyRzCtrl: public Controller{
public:
  CoMzRxRyRzCtrl();
  virtual ~CoMzRxRyRzCtrl();

  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(std::string setting_file_name);

protected:
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
