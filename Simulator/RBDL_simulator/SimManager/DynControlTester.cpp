#include "DynControlTester.hpp"

DynControlTester::DynControlTester():m_usleep_time(100), m_count(0),
                                     m_frequency_data_send(100),
                                     m_ratio_SERVO_sim_rate(10),
                                     m_cmd(NUM_ACT_JOINT),
                                     m_q(NUM_Q), m_qdot(NUM_QDOT),
                                     m_tau(NUM_QDOT), cori_(NUM_QDOT),
                                     grav_(NUM_QDOT), A_(NUM_QDOT, NUM_QDOT){
  m_q.setZero();  m_qdot.setZero();  m_tau.setZero();
  cori_.setZero(); grav_.setZero(); A_.setZero();

  m_sim_rate = (SERVO_RATE/(double)m_ratio_SERVO_sim_rate);
  printf("[Dynamic Control Test] Constructed\n");
}

void DynControlTester::_UDPDataSend(){
  double time(0.);
  time = ((double)m_count)*m_sim_rate;
  _UpdateExtraData();
  _UpdateLinePosition();
  m_plot_manager.SendData(time, st_x_, st_y_, end_x_, end_y_, extra_data_);
}

void DynControlTester::OneStepTest(){
  ++m_count;

  if(m_count == 1){ // initial torque command update
    _UpdateCommand();
  }else {
    _UpdateCommand(); // torque command update
  }
  // Update
  _MakeOneStepUpdate();

  if (m_count % m_frequency_data_send == 1){
    _UDPDataSend();
    _DataPrint();
  }
  _ExtraProcess();

}
