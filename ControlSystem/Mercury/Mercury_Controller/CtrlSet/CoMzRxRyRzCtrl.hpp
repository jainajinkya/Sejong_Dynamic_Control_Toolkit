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

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setStanceTime(double stance_time){ end_time_ = stance_time; }
  void setAmp(const std::vector<double> & amp);
  void setFrequency(const std::vector<double> & freq);
  void setPhase(const std::vector<double> & phase);

  void setStanceHeight(double height) {
    des_com_height_ = height;
    b_set_height_target_ = true;
  }

protected:
  sejong::Vector amp_, freq_, phase_;

  int dim_ctrl_;
  bool b_set_height_target_;
  double des_com_height_;

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
