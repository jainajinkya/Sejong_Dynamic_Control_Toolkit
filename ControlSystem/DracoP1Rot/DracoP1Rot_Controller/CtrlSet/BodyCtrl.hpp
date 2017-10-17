#ifndef BODY_CONTRL_DRACO_P1_ROT
#define BODY_CONTRL_DRACO_P1_ROT

#include <DracoController.hpp>

class BodyCtrl: public DracoController{
public:
  BodyCtrl();
  virtual ~BodyCtrl();

  virtual void OneStep(sejong::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(std::string setting_file_name);

protected:
  sejong::Vect2 body_lin_amp_;
  sejong::Vect2 body_lin_omega_;

  double end_time_;
};

#endif
