#ifndef WALKER_2D_SIM_TEST_H
#define WALKER_2D_SIM_TEST_H

#include <SimManager/DynControlTester.hpp>
#include <Walker2D_Controller/Walker2D_interface.hpp>
#include <Utils/wrap_eigen.hpp>

class Walker2D_DynCtrl: public DynControlTester{
public:
  Walker2D_DynCtrl();
  virtual ~Walker2D_DynCtrl();

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
  std::vector<double> body_pos;
  std::vector<double> body_vel;
  double body_ori;
  double body_ang_vel;
  std::vector<double> vec_cmd_;

  sejong::Vector m_jpos_ini;
  sejong::Vector m_jvel_ini;

  Walker2D_interface interface_;
};

#endif

