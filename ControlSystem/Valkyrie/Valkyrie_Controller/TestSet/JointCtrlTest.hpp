#ifndef JOINT_POSITION_TEST
#define JOINT_POSITION_TEST

#include <Test.hpp>

class JointCtrlTest: public Test{
public:
  JointCtrlTest();
  virtual ~JointCtrlTest();

  virtual void TestInitialization();
  void getTorqueInput(sejong::Vector & gamma);

protected:
  virtual int _NextPhase(const int & phase);

  Controller* jpos_ctrl_;
};

#endif
