#ifndef DYNAMIC_FLOATING_LEG_TEST_H
#define DYNAMIC_FLOATING_LEG_TEST_H

#include <plotter/DynCtrlTester.hpp>

#if CTRL_MODEL == FLOATING_LEG

class DynFloatingLegTest: public DynCtrlTester{
public:
  DynFloatingLegTest();
  virtual ~DynFloatingLegTest(){}
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
