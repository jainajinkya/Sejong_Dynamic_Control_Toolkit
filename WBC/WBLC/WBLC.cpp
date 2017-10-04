#include "WBLC.h"
#include <stdio.h>
#include <iostream>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/utilities.h>
#include "Task.h"
#include <Configuration.h>
#include "ReactionForceCalculator.h"
#include <ContactWrenchCalculator.h>
#include "WBLC_RF_cal.h"


#if MEASURE_TIME
#include <chrono>
#endif

WBLC::WBLC():U_(NUM_ACT_JOINT, NUM_QDOT){
  U_.setZero();
  U_.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);
}

WBLC::~WBLC(){
}
///////////////////////////////////////////////////////
// WARNNING: The last task of task array must be
//           Centroidal Angular Moment Task
///////////////////////////////////////////////////////
bool  WBLC::MakeTorque(const vector<Task*> & task_array,
                       const vector<Task*> & task_array_last,
                       ReactionForceCalculator * rf_cal,
                       const sejong::Matrix & A,
                       const sejong::Matrix & Ainv,
                       const sejong::Vector & grav,
                       const sejong::Vector & coriolis,
                       Vector& gamma){
  /// Check Task
  for (int i(0); i< task_array.size(); ++i){
    if(!task_array[i]->b_settask_){
      printf("Task Setting is incompleted\n");
      exit(0);
    }
  }

  ///////////////////////////////////////////////////////////////////////
  ///////                CoM Task Calculation                     ///////
  ///////////////////////////////////////////////////////////////////////
  _PrintDebug(0);

  rf_cal->PrepareOptimization();
  sejong::Vector com_acc;
  rf_cal->getCoMAcc(com_acc);
  sejong::Matrix Null;
  sejong::Vector qddot(NUM_QDOT);
  sejong::Vector qddot_pre(NUM_QDOT);

  _PrintDebug(1);
  sejong::Matrix Jcom, Jcm;
  rf_cal->getCentroidCtrl_Jacobian(Jcm);

  // ********* Linear
  Jcom = Jcm.block(3,0, 3, NUM_QDOT);
  sejong::Matrix com_lambda_inv(Jcom * Ainv * Jcom.transpose() );
  sejong::Matrix com_lambda;
  sejong::pseudoInverse(com_lambda_inv, 1.e-8, com_lambda, 0);
  sejong::Matrix Jcombar(Ainv * Jcom.transpose() * com_lambda );
  // Jdot * qdot
  sejong::Vector Jcom_dot_qdot = Jcom * Ainv * coriolis;

  qddot = Jcombar * (com_acc -Jcom_dot_qdot);
  Null = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jcombar * Jcom;
  _PrintDebug(2);
  ///////////////////////////////////////////////////////////////////////
  ///////////             Multi Task Loop                  //////////////
  ///////////////////////////////////////////////////////////////////////
#if MEASURE_TIME
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
  sejong::Matrix Id_Mtx( sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) );

  sejong::Matrix Jbar;
  sejong::Matrix JNull;
  sejong::Matrix jac;
  sejong::Matrix lambda_inv;

  size_t ii(0);
  // Until Centroid Angular Momentum
  for (; ii < task_array.size(); ++ii) {
    qddot_pre = qddot;
    Task * task(task_array[ii]);
    jac = task->getJacobian();
    JNull = jac * Null;
    lambda_inv = JNull * Ainv * JNull.transpose();

    sejong::Matrix lambda;
    sejong::pseudoInverse(lambda_inv, 1.e-7, lambda, 0);
    Jbar = Ainv * JNull.transpose() * lambda;

    qddot += Jbar * (task->getCommand() - jac * qddot_pre);
    Null *= (Id_Mtx - Jbar * JNull);
    // printf("%d th task\n", ii);
    // sejong::pretty_print(jac, std::cout, "Jac");
    // sejong::pretty_print(JNull, std::cout, "JNull");
  }

#if MEASURE_TIME
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
#endif

  /////////// Find Reaction Force ///////////////
  sejong::Vector Jcm_dot_qdot = Jcm * Ainv * coriolis;
  // sejong::Vector cm_ang_acc = Jcm.block(0,0, 3, NUM_QDOT) * qddot + Jcm_dot_qdot.head(3);
  sejong::Vector cm_ang_acc = Jcm.block(0,0, 3, NUM_QDOT) * qddot;

  rf_cal->setCAM_Acceleration(cm_ang_acc);
  _PrintDebug(5);
  bool b_emergency = rf_cal->GetReactionForce(Fr_);
  _PrintDebug(6);
  sejong::Vector cam_acc;
  rf_cal->getCAMAcc(cam_acc);
  // qddot = qddot_pre + Jbar * (cam_acc - jac * qddot_pre);
  // Find new cam_acc
  // sejong::Matrix JangNull = Jcm.block(0,0,3, NUM_QDOT) * Jbar;
  // sejong::pretty_print(Jbar, std::cout, "Jbar");
  // sejong::pretty_print(Jcm, std::cout, "Jcm");
  // sejong::pretty_print(JangNull, std::cout, "JangNull");
  // sejong::Matrix inv_Jang = JangNull.inverse();
  // sejong::Vector new_cam_acc = inv_Jang * (cam_acc - Jcm.block(0,0,3,NUM_QDOT) * (qddot_pre - Jbar * Jcm.block(0,0,3, NUM_QDOT) * qddot_pre));
  // sejong::pretty_print(cam_acc, std::cout, "cam acc");
  // sejong::pretty_print(new_cam_acc, std::cout, "new cam acc");

  qddot = qddot_pre + Jbar * (cam_acc - jac * qddot_pre);
  // sejong::Vector cc = jac*qddot_pre;
  // sejong::Vector cc2 = Jcm.block(0,0,3, NUM_QDOT) * qddot_pre;
  // sejong::pretty_print(cc, std::cout, "cc");
  // sejong::pretty_print(cc2, std::cout, "cc2");
  // qddot = qddot_pre + Jbar * (new_cam_acc - jac * qddot_pre);

  // sejong::Vector cm_ang_acc_check = Jcm.block(0,0, 3, NUM_QDOT) * qddot + Jcm_dot_qdot.head(3);
  sejong::Vector cm_ang_acc_check = Jcm.block(0,0, 3, NUM_QDOT) * qddot;

  // sejong::pretty_print(cam_acc, std::cout, "cam acc");
  // sejong::pretty_print(cm_ang_acc_check, std::cout, "cm ang acc check");
  // sejong::Vector err = (cm_ang_acc_check - cam_acc);
  // sejong::pretty_print(err, std::cout, "error");
  
  // sejong::pretty_print(Jbar, std::cout, "Jbar");
  // printf("\n");
#if MEASURE_TIME
  std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span2 = std::chrono::duration_cast< std::chrono::duration<double> >(t3 - t2);
#endif

  ////////// Do last Tasks  //////////////////////
  for (ii = 0; ii < task_array_last.size(); ++ii) {
    qddot_pre = qddot;
    _PrintDebug(3);

    Task * task(task_array_last[ii]);
    jac = task->getJacobian();
    JNull = jac * Null;
    lambda_inv = JNull * Ainv * JNull.transpose();

    sejong::Matrix lambda;
    sejong::pseudoInverse(lambda_inv, 1.e-7, lambda, 0);
    Jbar = Ainv * JNull.transpose() * lambda;

    qddot += Jbar * (task->getCommand() - jac * qddot_pre);
    Null *= (Id_Mtx - Jbar * JNull);
    _PrintDebug(4);
    // printf("%lu th:\n", ii);
    // printf("%d th last task\n", ii);

  }
  // sejong::pretty_print(tmp,std::cout, "centroid angular command");
  sejong::Matrix Nc_comb = Null;
  // sejong::pretty_print(Nc_comb, std::cout, "nc_comb");
  ///////////////////////////////////////////////////////////////////////
  ///////////             Gamma Calculation                  ////////////
  ///////////////////////////////////////////////////////////////////////
  sejong::Matrix Jc;
  rf_cal->getContactJacobian(Jc);

  sejong::Vector full_torque = A * qddot  + grav + coriolis - Jc.transpose() * Fr_;

  sejong::Matrix Aug_Mtx(NUM_QDOT, NUM_ACT_JOINT + NUM_QDOT);
  Aug_Mtx.block(0, 0, NUM_QDOT, NUM_ACT_JOINT) = U_.transpose();
  Aug_Mtx.block(0, NUM_ACT_JOINT, NUM_QDOT, NUM_QDOT) = -A * Nc_comb;

  _PrintDebug(5);
  sejong::Matrix Aug_Mtx_inv;
  sejong::pseudoInverse(Aug_Mtx, 1.0e-8, Aug_Mtx_inv, 0);
  sejong::Vector sol = (Aug_Mtx_inv * full_torque);
  gamma = sol.head(NUM_ACT_JOINT);

#if MEASURE_TIME
  std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span3 = std::chrono::duration_cast< std::chrono::duration<double> >(t4 - t3);
#endif


  _PrintDebug(7);
#if MEASURE_TIME
  std::cout << "Time to Multi-loop: " << time_span1.count() << " seconds."<<std::endl;;
  std::cout << "Time for Optimization: " << time_span2.count() << " seconds."<<std::endl;;
  std::cout << "Time for the last: " << time_span3.count() << " seconds."<<std::endl;;
#endif
  return b_emergency;
}


void WBLC::_PrintDebug(double i) {
  // printf("[Lin RF_WBC] %f \n", i);
}

//TEST for wrench
bool  WBLC::MakeTorque(const vector<Task*> & task_array,
                       const vector<Task*> & task_array_last,
                       ContactWrenchCalculator* wr_cal,
                       const sejong::Matrix & A,
                       const sejong::Matrix & Ainv,
                       const sejong::Vector & grav,
                       const sejong::Vector & coriolis,
                       Vector& gamma){
  /// Check Task
  for (int i(0); i< task_array.size(); ++i){
    if(!task_array[i]->b_settask_){
      printf("Task Setting is incompleted\n");
      exit(0);
    }
  }

  ///////////////////////////////////////////////////////////////////////
  ///////                CoM Task Calculation                     ///////
  ///////////////////////////////////////////////////////////////////////

  wr_cal->PrepareOptimization();
  sejong::Vector com_acc;
  wr_cal->getCoMAcc(com_acc);
  sejong::Matrix Null;
  sejong::Vector qddot(NUM_QDOT);
  sejong::Vector qddot_pre(NUM_QDOT);

  sejong::Matrix Jcom, Jcm;
  wr_cal->getCentroidCtrl_Jacobian(Jcm);

  // ********* Linear
  Jcom = Jcm.block(3,0, 3, NUM_QDOT);
  sejong::Matrix com_lambda_inv(Jcom * Ainv * Jcom.transpose() );
  sejong::Matrix com_lambda;
  sejong::pseudoInverse(com_lambda_inv, 1.e-8, com_lambda, 0);
  sejong::Matrix Jcombar(Ainv * Jcom.transpose() * com_lambda );
  sejong::Vector Jcom_dot_qdot = Jcom * Ainv * coriolis;

  qddot = Jcombar * (com_acc -Jcom_dot_qdot);
  Null = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jcombar * Jcom;
  ///////////////////////////////////////////////////////////////////////
  ///////////             Multi Task Loop                  //////////////
  ///////////////////////////////////////////////////////////////////////
  sejong::Matrix Id_Mtx( sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) );

  sejong::Matrix Jbar;
  sejong::Matrix JNull;
  sejong::Matrix jac;
  sejong::Matrix lambda_inv;

  size_t ii(0);
  // Until Centroid Angular Momentum
  for (; ii < task_array.size(); ++ii) {
    qddot_pre = qddot;
    Task * task(task_array[ii]);
    jac = task->getJacobian();
    JNull = jac * Null;
    lambda_inv = JNull * Ainv * JNull.transpose();

    sejong::Matrix lambda;
    sejong::pseudoInverse(lambda_inv, 1.e-7, lambda, 0);
    Jbar = Ainv * JNull.transpose() * lambda;

    qddot += Jbar * (task->getCommand() - jac * qddot_pre);
    Null *= (Id_Mtx - Jbar * JNull);
  }

  /////////// Find Reaction Force ///////////////
  sejong::Vector Jcm_dot_qdot = Jcm * Ainv * coriolis;
  sejong::Vector cm_ang_acc = Jcm.block(0,0, 3, NUM_QDOT) * qddot;

  wr_cal->setCAM_Acceleration(cm_ang_acc);
  bool b_emergency = wr_cal->GetReactionForce(Fr_);
  sejong::Vector cam_acc;
  wr_cal->getCAMAcc(cam_acc);
    qddot = qddot_pre + Jbar * (cam_acc - jac * qddot_pre);
    sejong::Vector cm_ang_acc_check = Jcm.block(0,0, 3, NUM_QDOT) * qddot;

  ////////// Do last Tasks  //////////////////////
  for (ii = 0; ii < task_array_last.size(); ++ii) {
    qddot_pre = qddot;
    _PrintDebug(3);

    Task * task(task_array_last[ii]);
    jac = task->getJacobian();
    JNull = jac * Null;
    lambda_inv = JNull * Ainv * JNull.transpose();

    sejong::Matrix lambda;
    sejong::pseudoInverse(lambda_inv, 1.e-7, lambda, 0);
    Jbar = Ainv * JNull.transpose() * lambda;

    qddot += Jbar * (task->getCommand() - jac * qddot_pre);
    Null *= (Id_Mtx - Jbar * JNull);
  }
  sejong::Matrix Nc_comb = Null;

  ///////////////////////////////////////////////////////////////////////
  ///////////             Gamma Calculation                  ////////////
  ///////////////////////////////////////////////////////////////////////
  sejong::Matrix Jc;
  wr_cal->getContactJacobian(Jc);

  sejong::Vector full_torque = A * qddot  + grav + coriolis - Jc.transpose() * Fr_;

  sejong::Matrix Aug_Mtx(NUM_QDOT, NUM_ACT_JOINT + NUM_QDOT);
  Aug_Mtx.block(0, 0, NUM_QDOT, NUM_ACT_JOINT) = U_.transpose();
  Aug_Mtx.block(0, NUM_ACT_JOINT, NUM_QDOT, NUM_QDOT) = -A * Nc_comb;

  sejong::Matrix Aug_Mtx_inv;
  sejong::pseudoInverse(Aug_Mtx, 1.0e-8, Aug_Mtx_inv, 0);
  sejong::Vector sol = (Aug_Mtx_inv * full_torque);
  gamma = sol.head(NUM_ACT_JOINT);

  return b_emergency;
}


