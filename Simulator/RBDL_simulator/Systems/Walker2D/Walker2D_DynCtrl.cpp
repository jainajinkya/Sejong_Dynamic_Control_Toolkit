#include "Walker2D_DynCtrl.hpp"
#include "Walker2D_Sim_Model.hpp"
#include <Utils/utilities.hpp>
#include <RBDL_Sim_Configuration.h>
#include "Optimizer/lcp/MobyLCP.h"

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
  //m_cmd[0] = 200. * (0.0 - m_q[0]) + 20.*(-m_qdot[0]);
  //m_cmd[1] = 200. * (0.9 - m_q[1]) + 20.*(-m_qdot[1]);
  //m_cmd[2] = 200. * (0.2 - m_q[2]) + 20.*(-m_qdot[2]);

}


void Walker2D_DynCtrl::_MakeOneStepUpdate(){ // model advance one step
  Walker2D_Sim_Model* model = Walker2D_Sim_Model::GetWalker2D_Sim_Model();
  model->UpdateModel(m_q, m_qdot);
  model->getCoriolis(cori_);
  model->getGravity(grav_);
  model->getMassInertia(A_);

/*  sejong::Vector qddot(NUM_QDOT);
  qddot = A_.inverse()*(m_cmd - cori_ - grav_);

  m_q = m_q + m_qdot * m_sim_rate;
  m_qdot = m_qdot + qddot * m_sim_rate;*/

  // Contact Model : Time step integrator Linear Complimentary Problem
  sejong::Vect3 lf_pos;
  sejong::Matrix J_lf(3,NUM_QDOT);
  sejong::Matrix J_lfc(1,NUM_QDOT);  

  sejong::Vect3 rf_pos;  
  sejong::Matrix J_rf(3,NUM_QDOT);
  sejong::Matrix J_rfc(1,NUM_QDOT);  

  model->getPos(SJ_SIM_LinkID::LK_SIM_LEFT_FOOT, lf_pos);
  model->getPos(SJ_SIM_LinkID::LK_SIM_RIGHT_FOOT, rf_pos);  
  model->getFullJacobian(SJ_SIM_LinkID::LK_SIM_LEFT_FOOT, J_lf);
  model->getFullJacobian(SJ_SIM_LinkID::LK_SIM_RIGHT_FOOT, J_rf);

  J_lfc.block(0,0, 1, NUM_QDOT) = J_lf.block(1, 0, 1, NUM_QDOT); // Z
  J_rfc.block(0,0, 1, NUM_QDOT) = J_rf.block(1, 0, 1, NUM_QDOT); // Z

  sejong::Matrix J_c(2,NUM_QDOT);  
  sejong::Vector phi(2); 

  J_c.block(0,0, 1, NUM_QDOT) = J_lfc;
  J_c.block(1,0, 1, NUM_QDOT) = J_rfc;     
  phi[0] = lf_pos[1];
  phi[1] = rf_pos[1];

  // Prepare the LCP problem
  MobyLCPSolver l;
  sejong::Matrix alpha(phi.size(), phi.size());
  sejong::Vector beta(phi.size());
  sejong::Vector lambda(phi.size());
  lambda.setZero();

  sejong::Matrix A_inv = A_.inverse();
  alpha = J_c*A_inv*J_c.transpose()*m_sim_rate;
  beta = phi/m_sim_rate + J_c*(m_qdot + m_sim_rate*A_inv*(m_cmd - cori_ - grav_));

  bool result = l.lcp_lemke(alpha, beta, &lambda);
  std::cout << "LCP result" << result << " (l1, l2) = " << lambda[0] << ", " << lambda[1] << std::endl;
  // ----
  // Perform Semi Implicit Integration
  // qdot_{t+1} = qddot_t*dt + qdot_{t}
  // q_{t+1} = qdot_{t+1}*dt + q_{t}
  sejong::Vector qddot(NUM_QDOT);
  qddot = A_.inverse()*(m_cmd - cori_ - grav_ + J_c.transpose()*lambda);
  m_qdot = qddot * m_sim_rate + m_qdot;
  m_q = m_qdot * m_sim_rate + m_q;

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
