#ifndef STEP_TEST
#define STEP_TEST

#include <Test.hpp>
class StateProvider;

enum STPhase{
  initiation = 0,
  lift_up = 1,
  double_contact_1 = 2,
  right_swing_start_trans = 3,
  right_swing = 4,
  right_swing_end_trans = 5,
  double_contact_2 = 6,
  left_swing_start_trans = 7,
  left_swing = 8,
  left_swing_end_trans = 9,
  NUM_STPHASE
};

class StepTest: public Test{
public:
  StepTest();
  virtual ~StepTest();
  virtual void TestInitialization();

protected:
  StateProvider* sp_;
  virtual int _NextPhase(const int & phase);

  Controller* jpos_ctrl_;
  Controller* body_up_ctrl_;
  Controller* body_fix_ctrl_;
  // Right
  Controller* right_swing_start_trans_ctrl_;
  Controller* right_swing_ctrl_;
  Controller* right_swing_end_trans_ctrl_;
  // Left
  Controller* left_swing_start_trans_ctrl_;
  Controller* left_swing_ctrl_;
  Controller* left_swing_end_trans_ctrl_;
};
#endif
