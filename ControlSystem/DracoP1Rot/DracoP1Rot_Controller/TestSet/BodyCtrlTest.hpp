#ifndef BODY_CONTROL_TEST_DRACO_P1_ROT
#define BODY_CONTROL_TEST_DRACO_P1_ROT

#include <DracoTest.hpp>

enum BCPhase{
  initial_transition = 0,
  shake = 1,
  move_up = 2,
  stay_up = 3,
  NUM_PHASE
};

class BodyCtrlTest: public DracoTest{
public:
  BodyCtrlTest();
  virtual ~BodyCtrlTest();
  virtual void TestInitialization();

protected:
  int phase_;
  virtual int _NextPhase(const int & phase);

  DracoController* body_ini_tran_;
  DracoController* body_shake_ctrl_;
  DracoController* body_up_ctrl_;
  DracoController* body_stay_;
};

#endif
