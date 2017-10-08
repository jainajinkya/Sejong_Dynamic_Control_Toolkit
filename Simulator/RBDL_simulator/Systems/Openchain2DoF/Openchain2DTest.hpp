#ifndef OPENCHAIN_2D_TEST_H
#define OPENCHAIN_2D_TEST_H

#include <plotter/CtrlTester.hpp>

#if CTRL_MODEL == OPENCHAIN_2D

class Openchain2DTest: public CtrlTester{
public:
  Openchain2DTest();
  virtual ~Openchain2DTest(){}
  virtual void Initialization();

protected:
  virtual void _ExtraProcess();
  virtual void _DataPrint();
};

#endif // CTRL_MODEL == OPENCHAIN_2D
#endif // OPENCHAIN_2D_TEST_H
