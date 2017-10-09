#include "Openchain2DoF_DynCtrl.hpp"
#include "OC2_Sim_Model.hpp"
#include <Utils/utilities.hpp>
#include <RBDL_Sim_Configuration.h>

Openchain2DoF_DynCtrl::Openchain2DoF_DynCtrl():DynControlTester(),
                                               jpos(NUM_ACT_JOINT),
                                               jvel(NUM_ACT_JOINT),
                                               torque(NUM_ACT_JOINT),
                                               vec_cmd_(NUM_ACT_JOINT),
                                               m_jpos_ini(NUM_ACT_JOINT),
                                               m_jvel_ini(NUM_ACT_JOINT)
{
  plot_sys_ = new PlottingSystem();
  printf("[Openchain 2DoF Dynamic Control Test] Constructed\n");
}
Openchain2DoF_DynCtrl::~Openchain2DoF_DynCtrl(){}

void Openchain2DoF_DynCtrl::Initialization(){
  m_jpos_ini.setZero();
  m_jvel_ini.setZero();

  m_jpos_ini[0] = -1.0;
  m_jpos_ini[1] = -1.0;


  m_q = m_jpos_ini;
  m_qdot.setZero();
}

void Openchain2DoF_DynCtrl::_UpdateLinePosition(){
  plot_sys_->GetStEndPt_Link(st_x_, st_y_, end_x_, end_y_);
}

void Openchain2DoF_DynCtrl::_UpdateCommand(){ // m_cmd update
  double time;
  time = ((double)m_count)*m_sim_rate;
  for(int i(0);i<NUM_ACT_JOINT; ++i){
    jpos[i] = m_q[i];
    jvel[i] = m_qdot[i];
    torque[i] = m_tau[i];
  }

  interface_.GetCommand(_VAR_SENSOR_DATA_, vec_cmd_);
  for (int i(0); i<NUM_ACT_JOINT; ++i){
    m_cmd[i] = vec_cmd_[i];
  }
}

void Openchain2DoF_DynCtrl::_MakeOneStepUpdate(){ // model advance one step
  OC2_Sim_Model* model = OC2_Sim_Model::GetOC2_Sim_Model();
  model->UpdateModel(m_q, m_qdot);
  model->getCoriolis(cori_);
  model->getGravity(grav_);
  model->getMassInertia(A_);

  sejong::Vector qddot(NUM_QDOT);
  qddot = A_.inverse()*(m_cmd - cori_ - grav_);

  m_q = m_q + m_qdot * m_sim_rate;
  m_qdot = m_qdot + qddot * m_sim_rate;
}

void Openchain2DoF_DynCtrl::_ExtraProcess(){}
void Openchain2DoF_DynCtrl::_UpdateExtraData(){}

void Openchain2DoF_DynCtrl::_DataPrint(){
  // bool b_plot = true;
  bool b_plot = false;

  if(b_plot){
    sejong::pretty_print(m_q, std::cout, "q");
    sejong::pretty_print(m_qdot, std::cout, "qdot");

    std::cout<<"grav:\n"<<grav_<<std::endl;
    std::cout<<"cori:\n"<<cori_<<std::endl;
    std::cout<<"Inertia:\n"<<A_<<std::endl;

    // printf("jpos cmd: %f, %f, %f\n", m_jpos_cmd[0], m_jpos_cmd[1], m_jpos_cmd[2]);
    // printf("jvel cmd: %f, %f, %f\n", m_jvel_cmd[0], m_jvel_cmd[1], m_jvel_cmd[2]);
    // printf("jeff_cmd: %f, %f, %f\n", m_jeff_cmd[0], m_jeff_cmd[1], m_jeff_cmd[2]);
  }
}
