#ifndef BODY_CONTROL_TEST
#define BODY_CONTROL_TEST

#include <Test.hpp>

enum BCPhase{
  initial_transition = 0,
  shake = 1,
  move_up = 2,
  stay_up = 3,
  NUM_PHASE
};

class BodyCtrlTest: public Test{
public:
  BodyCtrlTest();
  virtual ~BodyCtrlTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);

  Controller* body_ini_tran_;
  Controller* body_shake_ctrl_;
  Controller* body_up_ctrl_;
  Controller* body_stay_;
};

#endif
