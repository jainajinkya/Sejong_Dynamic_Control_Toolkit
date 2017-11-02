#ifndef BODY_CONTROL_TEST
#define BODY_CONTROL_TEST

#include <MercuryTest.hpp>

enum BCPhase{
  initial_transition = 0,
  shake = 1,
  move_up = 2,
  stay_up = 3,
  NUM_PHASE
};

class BodyCtrlTest: public MercuryTest{
public:
  BodyCtrlTest();
  virtual ~BodyCtrlTest();
  virtual void TestInitialization();

protected:
  int phase_;
  virtual int _NextPhase(const int & phase);

  MercuryController* body_ini_tran_;
  MercuryController* body_shake_ctrl_;
  MercuryController* body_up_ctrl_;
  MercuryController* body_stay_;
};

#endif
