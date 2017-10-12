#include "Walker2D_DynCtrl.hpp"
#include "Walker2D_Sim_Model.hpp"
#include <Utils/utilities.hpp>
#include <RBDL_Sim_Configuration.h>

Walker2D_DynCtrl::Walker2D_DynCtrl():DynControlTester(),
                                     jpos(NUM_ACT_JOINT),
                                     jvel(NUM_ACT_JOINT),
                                     torque(NUM_ACT_JOINT),
                                     vec_cmd_(NUM_ACT_JOINT),
                                     m_jpos_ini(NUM_ACT_JOINT),
                                     m_jvel_ini(NUM_ACT_JOINT),
                                     body_pos(2), // (x, z)
                                     body_vel(2)
{
  plot_sys_ = new PlottingSystem();
  printf("[Openchain 3DoF Dynamic Control Test] Constructed\n");
}
Walker2D_DynCtrl::~Walker2D_DynCtrl(){}

void Walker2D_DynCtrl::Initialization(){
  m_q.setZero();
  m_qdot.setZero();

  m_jpos_ini.setZero();
  m_jvel_ini.setZero();

  m_jpos_ini[0] = -1.1;
  m_jpos_ini[1] = 1.0;

  // m_jpos_ini[1] = -2.0;
  m_jpos_ini[2] = 0.6;
  m_jpos_ini[3] = 1.0;

  m_q.tail(NUM_ACT_JOINT) = m_jpos_ini;

  // height
  m_q[1] = 0.9;
  // ori
  m_q[2] = 0.2;
}

void Walker2D_DynCtrl::_UpdateLinePosition(){
  plot_sys_->GetStEndPt_Link(st_x_, st_y_, end_x_, end_y_);
}

void Walker2D_DynCtrl::_UpdateCommand(){ // m_cmd update
  double time;
  time = ((double)m_count)*m_sim_rate;
  for(int i(0);i<NUM_ACT_JOINT; ++i){
    jpos[i] = m_q[i + NUM_VIRTUAL];
    jvel[i] = m_qdot[i + NUM_VIRTUAL];
    torque[i] = m_tau[i + NUM_VIRTUAL];
  }
  for(int i(0); i<2; ++i){
    body_pos[i] = m_q[i];
    body_vel[i] = m_qdot[i];
  }
  body_ori = m_q[2];
  body_ang_vel = m_qdot[2];

  interface_.GetCommand(_VAR_SENSOR_DATA_, vec_cmd_);
  for (int i(0); i<NUM_ACT_JOINT; ++i){
    m_cmd[i + NUM_VIRTUAL] = vec_cmd_[i];
  }
  // fix in the air
  m_cmd[0] = 200. * (0.0 - m_q[0]) + 20.*(-m_qdot[0]);
  m_cmd[1] = 200. * (0.9 - m_q[1]) + 20.*(-m_qdot[1]);
  m_cmd[2] = 200. * (0.2 - m_q[2]) + 20.*(-m_qdot[2]);

}


void Walker2D_DynCtrl::_MakeOneStepUpdate(){ // model advance one step
  Walker2D_Sim_Model* model = Walker2D_Sim_Model::GetWalker2D_Sim_Model();
  model->UpdateModel(m_q, m_qdot);
  model->getCoriolis(cori_);
  model->getGravity(grav_);
  model->getMassInertia(A_);

  sejong::Vector qddot(NUM_QDOT);
  qddot = A_.inverse()*(m_cmd - cori_ - grav_);

  m_q = m_q + m_qdot * m_sim_rate;
  m_qdot = m_qdot + qddot * m_sim_rate;

  // Contact Model might need to be here:
  // ......
  // ......
}

void Walker2D_DynCtrl::_ExtraProcess(){}
void Walker2D_DynCtrl::_UpdateExtraData(){}

void Walker2D_DynCtrl::_DataPrint(){
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
