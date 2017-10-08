#ifndef DYNAMIC_OPENCHAIN_2D_TEST_H
#define DYNAMIC_OPENCHAIN_2D_TEST_H

#include <plotter/DynCtrlTester.hpp>

#if CTRL_MODEL == OPENCHAIN_2D

class DynOpenchain2DTest: public DynCtrlTester{
public:
  DynOpenchain2DTest();
  virtual ~DynOpenchain2DTest(){}
  virtual void Initialization();

protected:
  virtual void _ExtraProcess();
  virtual void _DataPrint();
  virtual void _ComputeQdot(const VectorA & jpos_cmd,
                            const VectorA & jvel_cmd,
                            const VectorA & jeff_cmd,
                            VectorQ & qdot);
};

#endif
#endif
