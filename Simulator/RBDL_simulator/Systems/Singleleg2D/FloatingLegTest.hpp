#ifndef FLOATING_LEG_TEST_H
#define FLOATING_LEG_TEST_H

#include <plotter/CtrlTester.hpp>

#if CTRL_MODEL == FLOATING_LEG

class FloatingLegTest: public CtrlTester{
public:
  FloatingLegTest();
  virtual ~FloatingLegTest(){}
  virtual void Initialization();

protected:
  virtual void _ExtraProcess();
  virtual void _DataPrint();
};

#endif
#endif
