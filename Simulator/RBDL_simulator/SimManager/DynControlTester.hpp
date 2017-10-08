#ifndef DYNAMIC_CONTROLLER_TESTER
#define DYNAMIC_CONTROLLER_TESTER

#include "PlottingManager.hpp"
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class Plotting_System;

class DynControlTester{
 public:
  DynControlTester();
  virtual ~DynControlTester(){}

  virtual void Initialization() = 0;
  void OneStepTest();
  int m_usleep_time;

 protected:
  virtual void _ExtraProcess() = 0;
  virtual void _UpdateLinePosition() = 0;
  virtual void _DataPrint() = 0;
  virtual void _UpdateCommand() = 0; // m_cmd update
  virtual void _MakeOneStepUpdate() = 0; // model advance one step
  virtual void _UpdateExtraData() = 0;

  // Data "Must" be updated
  sejong::Vector m_cmd;
  std::vector<double> extra_data_;
  std::vector<double>  st_x_;
  std::vector<double>  st_y_;
  std::vector<double>  end_x_;
  std::vector<double>  end_y_;

  // plotting system must be allocated
  Plotting_System* plot_sys_;
  void _UDPDataSend();

  int m_count;
  int m_frequency_data_send;
  int m_ratio_SERVO_sim_rate;
  double m_sim_rate;

  sejong::Vector m_q;
  sejong::Vector m_qdot;
  sejong::Vector m_tau;

  sejong::Vector cori_;
  sejong::Vector grav_;
  sejong::Matrix A_;

  PlottingManager m_plot_manager;
};

#endif
