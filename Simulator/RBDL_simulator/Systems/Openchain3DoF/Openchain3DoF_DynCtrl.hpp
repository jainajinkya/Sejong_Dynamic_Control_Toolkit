#ifndef OPENCHAIN_3D_SIM_TEST_H
#define OPENCHAIN_3D_SIM_TEST_H

#include <SimManager/DynControlTester.hpp>
#include <Openchain3DoF_Controller/OC3_interface.hpp>
#include <Utils/wrap_eigen.hpp>

class Openchain3DoF_DynCtrl: public DynControlTester{
public:
  Openchain3DoF_DynCtrl();
  virtual ~Openchain3DoF_DynCtrl();

  virtual void Initialization();

protected:
  virtual void _UpdateLinePosition();
  virtual void _ExtraProcess();
  virtual void _DataPrint();
  virtual void _UpdateCommand(); // m_cmd update
  virtual void _MakeOneStepUpdate(); // model advance one step
  virtual void _UpdateExtraData();

  std::vector<double> jpos;
  std::vector<double> jvel;
  std::vector<double> torque;
  std::vector<double> vec_cmd_;

  sejong::Vector m_jpos_ini;
  sejong::Vector m_jvel_ini;

  OC3_interface interface_;
};

#endif

