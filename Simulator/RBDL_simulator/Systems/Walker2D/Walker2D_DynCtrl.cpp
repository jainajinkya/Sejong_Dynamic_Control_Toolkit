#include "Walker2D_DynCtrl.hpp"
#include "Walker2D_Sim_Model.hpp"
#include <Utils/utilities.hpp>
#include <RBDL_Sim_Configuration.h>
#include "Optimizer/lcp/MobyLCP.h"
#include <chrono>

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
  m_jpos_ini[2] = 0.1;//0.6;
  m_jpos_ini[3] = 1.0;

  m_q.tail(NUM_ACT_JOINT) = m_jpos_ini;

  // height
  m_q[1] = 0.45;//0.9;
  // ori
  m_q[2] = 0.0; //0.2;
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

  sejong::Matrix J_phi(2,NUM_QDOT);  
  sejong::Vector phi(2); 

  J_phi.block(0,0, 1, NUM_QDOT) = J_lfc; // Jacobian of distance to contact point 1
  J_phi.block(1,0, 1, NUM_QDOT) = J_rfc; // Jacobian of distance to contact point 2    
  phi[0] = lf_pos[1]; // Distance to contact point 1
  phi[1] = rf_pos[1]; // Distance to contact point 2


  sejong::Vector qddot(NUM_QDOT);
  sejong::Matrix A_inv = A_.inverse();
  // With Friction Constraints ---------------------------------------------------------------------
  const int p = 2; // Number of Contacts
  const int d = 2; // Number of Friction Basis Vectors


  double mu_static = 1.0;//0.1; // Friction Coefficient
  sejong::Vector n1(2); n1[1] = 1.0; // Normal Vector for contact 1
  sejong::Vector n2(2); n2[1] = 1.0; // Normal Vector for contact 2  
  sejong::Matrix J_c1 = J_lf.block(0, 0, 2, NUM_QDOT); // Jacobian (x,z) at contact point 1
  sejong::Matrix J_c2 = J_rf.block(0, 0, 2, NUM_QDOT); // Jacobian (x,z) at contact point 2  

  // Set Projected Normal Forces Direction
  sejong::Matrix N(NUM_QDOT, p);
  N.block(0,0, NUM_QDOT, 1) = J_c1.transpose()*n1;
  N.block(0,1, NUM_QDOT, 1) = J_c2.transpose()*n2;

  // Set Basis Vectors of Friction Cone 
  sejong::Matrix D1(d, d); // Basis for contact 1 
  D1.setZero();
  D1(0,0) = 1; 
  D1(0,1) = -1;
  sejong::Matrix D2 = D1; // Basis for contact 2
  
  // Set Projected Tangential Forces Direction
  sejong::Matrix B(NUM_QDOT, p*d); 
  B.block(0,0, NUM_QDOT, d) = J_c1.transpose()*D1;
  B.block(0,2, NUM_QDOT, d) = J_c2.transpose()*D2;  

  // Unit e vecs and E binary matrix
  sejong::Vector e(d); // same size as number of friction basis direction 
  e.setOnes();
  sejong::Matrix E(p*d, d); 
  E.setZero();
  E.block(0, 0, e.size(),1) = e; 
  E.block(2, 1, e.size(),1) = e;

  // mu Matrix
  //sejong::Matrix Mu(p,p);
  //Mu.setOnes();
  sejong::Matrix Mu = sejong::Matrix::Identity(p, p);
  Mu *= mu_static;

  // Prepare the LCP problem
  double h = m_sim_rate; // timestep
  // Prepare Alpha Mu
  // 1st Row
  sejong::Matrix alpha_mu(p + p*d + p, p + p*d + p);
  alpha_mu.setZero();
  alpha_mu.block(0, 0, p, p) =  h*J_phi*A_inv*N;
  alpha_mu.block(0, p, p, p*d) =  h*J_phi*A_inv*B;

  // 2nd Row
  alpha_mu.block(p, 0,     p*d, p) = h*B.transpose()*A_inv*N;
  alpha_mu.block(p, p,     p*d, p*d) = h*B.transpose()*A_inv*B;
  alpha_mu.block(p, p+p*d, p*d, p) = E;

  // 3rd Row (Scaling for stabilization according to Tan, Jie. et al "Contact Handling for Articulated Rigid Bodies using LCP")
  alpha_mu.block(p+p*d, 0, p, p) = Mu * h; // Scaling by h for stabilizing contact constraints
  alpha_mu.block(p+p*d, p, p, p*d) = -E.transpose() * h;  // Scaling by h for stabilizing contact constraints

  // Prepare Beta
  sejong::Vector tau_star = A_*m_qdot + h*(m_cmd - cori_ - grav_);
  sejong::Vector beta_mu(p + p*d + p);
  beta_mu.setZero();
  // 1st Row
  beta_mu.block(0,0, p, 1) = phi/h + J_phi*A_inv*tau_star;
  // 2nd Row
  beta_mu.block(p,0, p*d, 1) = B.transpose()*A_inv*tau_star;  

  sejong::Vector fn_fd_lambda(p + p*d + p);
  fn_fd_lambda.setZero();

  // Solve LCP Problem
    // ---- START TIMER 
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();


  MobyLCPSolver l_mu;  
  bool result_mu = l_mu.lcp_lemke_regularized(alpha_mu, beta_mu, &fn_fd_lambda);

/*  std::cout << "mu LCP result " << result_mu << " (fn1, fn2, fd1, -fd1, fd2, -fd2) = " << 
                                              fn_fd_lambda[0] << ", " << 
                                              fn_fd_lambda[1] << ", " <<
                                              fn_fd_lambda[2] << ", " <<
                                              fn_fd_lambda[3] << ", " <<
                                              fn_fd_lambda[4] << ", " << std::endl;
*/
  sejong::Vector fn = fn_fd_lambda.block(0, 0, p, 1);
  sejong::Vector fd = fn_fd_lambda.block(p, 0, p*d, 1);
    // ----- END TIMER
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
/*  std::cout << "  LCP solver took " << time_span1.count()*1000.0 << "ms"<<std::endl;  */

  
  // Perform Semi Implicit Integration
  // qdot_{t+1} = qddot_t*dt + qdot_{t}
  // q_{t+1} = qdot_{t+1}*dt + q_{t}
  qddot = A_inv*(m_cmd - cori_ - grav_ + N*fn + B*fd);
  m_qdot = qddot * h + m_qdot;
  m_q = m_qdot * h+ m_q;

  sejong::pretty_print(m_q, std::cout, "q_next actual"); 
  // Without Friction Constraints ----------------------------------------
  // Prepare the LCP problem
/*  MobyLCPSolver l;
  sejong::Matrix alpha(phi.size(), phi.size());
  sejong::Vector beta(phi.size());
  sejong::Vector lambda(phi.size());
  lambda.setZero();
  alpha = J_phi*A_inv*J_phi.transpose()*m_sim_rate;
  beta = phi/m_sim_rate + J_phi*(m_qdot + m_sim_rate*A_inv*(m_cmd - cori_ - grav_));

  bool result = l.lcp_lemke(alpha, beta, &lambda);
  std::cout << "LCP result" << result << " (l1, l2) = " << lambda[0] << ", " << lambda[1] << std::endl;
  std::cout << "  Total Normal Force = " << lambda[0] + lambda[1] << std::endl;  
  // ----
  // Perform Semi Implicit Integration
  // qdot_{t+1} = qddot_t*dt + qdot_{t}
  // q_{t+1} = qdot_{t+1}*dt + q_{t}
  qddot = A_.inverse()*(m_cmd - cori_ - grav_ + J_phi.transpose()*lambda);
  m_qdot = qddot * m_sim_rate + m_qdot;
  m_q = m_qdot * m_sim_rate + m_q;*/
  // Without Friction Constraints ----------------------------------------  

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
