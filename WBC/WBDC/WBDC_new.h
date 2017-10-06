#ifndef WBDC_2D_NEW_H
#define WBDC_2D_NEW_H
// Whole Body Dynamic Controller for 2D

#include <vector>
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Configuration.h>
#include <Optimizer/Goldfarb/QuadProg++.hh>

using namespace sejong;
using namespace std;

#define DIM_CM 3
#define DIM_FRICTION 5
template <int DIM_RF, int DIM_SELECTED_TASK, int DIM_TASK>
  class WBDC_new{
 public:
  GolDIdnani::GVect<double> x;
  // Cost
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  // Equality
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  // Inequality
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;

 WBDC_new():
  G(DIM_RF + DIM_SELECTED_TASK, DIM_RF + DIM_SELECTED_TASK ),
    g0(DIM_RF + DIM_SELECTED_TASK),
    CE(DIM_RF + DIM_SELECTED_TASK, DIM_CM),
    ce0(DIM_CM),
    CI(DIM_RF + DIM_SELECTED_TASK, DIM_FRICTION + NUM_ACT_JOINT + NUM_ACT_JOINT),
    ci0(DIM_FRICTION + NUM_ACT_JOINT + NUM_ACT_JOINT){
  }
  ~WBDC_new(){}

  void _SetCost(){
    for(int i(0); i<DIM_RF+DIM_SELECTED_TASK; ++i){
      for(int j(0); j<DIM_RF+DIM_SELECTED_TASK; ++j){
        G[i][j] = 0.;
      }
      g0[i] = 0.;
    }

    for (int i(0); i<DIM_RF; ++i){
      G[i][i] = 1.0;
    }
    G[1][1] = 0.0001;
    for (int i(DIM_RF); i<DIM_RF + DIM_SELECTED_TASK; ++i){
      G[i][i] = 1000.;
    }
    G[DIM_SELECTED_TASK + DIM_RF - 1][DIM_SELECTED_TASK + DIM_RF - 1] = 0.1;
    std::cout<<G<<std::endl;
  }

  bool MakeTorque(const Eigen::Matrix<double, NUM_QDOT,NUM_QDOT> & A,
                  const Eigen::Matrix<double, NUM_QDOT,1> & cori,
                  const Eigen::Matrix<double, NUM_QDOT,1> & grav,
                  const Eigen::Matrix<double, DIM_RF, NUM_QDOT> & Jc,
                  const Eigen::Matrix<double, 3, 3> & Icm,
                  const Eigen::Matrix<double, 3, NUM_QDOT> & Jcm,
                  const Eigen::Matrix<double, 3, 1> & JcmDotQdot,
                  const Eigen::Matrix<double, 3, 1> & grav_cm,
                  const Eigen::Matrix<double, 3, DIM_RF> & Sf,
                  const Eigen::Matrix<double, NUM_QDOT, DIM_TASK> & B,
                  const Eigen::Matrix<double, DIM_TASK, 1> & x_task,
                  const Eigen::Matrix<int, DIM_TASK, 1> & selected_task,
                  const Eigen::Matrix<double, NUM_QDOT, 1> & c,
                  const Eigen::Matrix<double, NUM_ACT_JOINT, 1> & tau_min,
                  const Eigen::Matrix<double, NUM_ACT_JOINT, 1> & tau_max,
                  Eigen::Matrix<double, NUM_ACT_JOINT, 1> & torque_cmd,
                  Eigen::Matrix<double, DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL, 1> & opt_result){

    double mu(0.3);
    double l1(0.12);
    double l2(0.08);
    _PrintDebug(0.5);

    // set cost
    _SetCost();

    // set constraint
    _PrintDebug(1.0);
    Eigen::Matrix<double, DIM_TASK, DIM_SELECTED_TASK> Sr;
    Sr.setZero();

    int num_column(0);
    for (int i(0); i<DIM_TASK; ++i){
      if(selected_task[i] == 1){
        Sr(i, num_column) = 1.;
        ++num_column;
      }
    }
    std::cout<<Sr<<std::endl;
    // CM
    Eigen::Matrix<double, DIM_CM, DIM_SELECTED_TASK> IcmJcmBSr = Icm * Jcm * B * Sr;
    Eigen::Matrix<double, DIM_CM, 1> IcmJcmBxt_c_gcm = Icm * Jcm * (B * x_task + c) + Icm *JcmDotQdot + grav_cm;

    // Multibody
    Eigen::Matrix<double, NUM_VIRTUAL + NUM_ACT_JOINT, 1> ABxt_c_b_g = A * (B * x_task + c) + cori + grav;
    Eigen::Matrix<double, NUM_VIRTUAL + NUM_ACT_JOINT, DIM_RF + DIM_SELECTED_TASK> JcT_ABSr;
    JcT_ABSr.block(0,0, NUM_QDOT, DIM_RF) = -Jc.transpose();
    JcT_ABSr.block(0,DIM_RF, NUM_QDOT, DIM_SELECTED_TASK) = A*B*Sr;

    std::cout<<Icm * Jcm - A.block(0,0, 3, NUM_QDOT)<<std::endl;

    // Friction
    Eigen::Matrix<double, DIM_FRICTION, DIM_RF> U_aug;
    U_aug.setZero();
    U_aug(0, 1) = 1.; // Fz > 0
    U_aug(1, 0) = -1.; U_aug(1,1) = mu; // -Fx + mu * Fz > 0
    U_aug(2, 0) = 1.; U_aug(2, 1) = mu; // Fx + mu * Fz > 0
    U_aug(3, 1) = 1.; U_aug(3, 2) = 1./l1; // Fz + tau_y/l1 > 0
    U_aug(4, 1) = 1.; U_aug(4, 2) = -1./l2; // Fz - tau_y/l2 > 0

    /* std::cout<<U_aug<<std::endl; */
    for (int i(0); i<DIM_CM; ++i){
      for(int j(0); j<DIM_RF + DIM_SELECTED_TASK; ++j){
        CE[j][i] = 0.;
      }
      ce0[i] = 0.;
    }
    std::cout<<CE<<std::endl;
    std::cout<<ce0<<std::endl;

    /////// Equality
    //CM
    for(int i(0); i< DIM_CM; ++i){
      for(int j(0); j<DIM_RF; ++j){
        CE[j][i] = -Sf(i, j);
      }
      for(int j(0); j<DIM_SELECTED_TASK; ++j){
        CE[j+DIM_RF][i] = IcmJcmBSr(i, j);
      }
      ce0[i] = IcmJcmBxt_c_gcm[i];
    }
    std::cout<<CE<<std::endl;
    std::cout<<ce0<<std::endl;

    //// Ineqality
    for (int i(0); i<DIM_FRICTION + 2*NUM_ACT_JOINT; ++i){
      for(int j(0); j<DIM_RF + DIM_SELECTED_TASK; ++j){
        CI[j][i] = 0.;
      }
      ci0[i] = 0.;
    }

    // friction
    for(int i(0); i< DIM_FRICTION; ++i){
      for(int j(0); j<DIM_RF; ++j){
        CI[j][i]= U_aug(i,j);
      }
      ci0[i] = 0.;
    }
    ci0[DIM_FRICTION-2] = -5.;
    ci0[DIM_FRICTION-1] = -5.;

    std::cout<<CI<<std::endl;
    std::cout<<ci0<<std::endl;

    // tau_max
    for(int i(0); i < NUM_ACT_JOINT; ++i){
      for(int j(0); j<DIM_RF + DIM_SELECTED_TASK; ++j){
        CI[j][i + DIM_FRICTION] = -JcT_ABSr(i+NUM_VIRTUAL,j);
      }
      ci0[i + DIM_FRICTION] = tau_max[i] - ABxt_c_b_g[i + NUM_VIRTUAL];
    }
    std::cout<<CI<<std::endl;
    std::cout<<ci0<<std::endl;

    // tau_min
    for(int i(0); i < NUM_ACT_JOINT; ++i){
      for(int j(0); j<DIM_RF + DIM_SELECTED_TASK; ++j){
        CI[j][i + DIM_FRICTION + NUM_ACT_JOINT] = JcT_ABSr(i+NUM_VIRTUAL,j);
      }
      ci0[i + DIM_FRICTION + NUM_ACT_JOINT] = -tau_min[i] + ABxt_c_b_g[i + NUM_VIRTUAL];
    }
    std::cout<<CI<<std::endl;
    std::cout<<ci0<<std::endl;

    _PrintDebug(2.0);
    std::cout<<g0<<std::endl;
    std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
    std::cout << "x: " << x << std::endl;
    for(int i(0); i<DIM_RF + DIM_SELECTED_TASK; ++i){
      opt_result[i] = x[i];
    }
    _PrintDebug(3.0);

    // Return torque command
    _GetSolution(A, cori, grav, Jc, Jcm,
                 B, x_task, selected_task, c, torque_cmd);
    printf("qddot:\n");
    Eigen::VectorXd qddot = B * x_task + c;
    std::cout<<qddot<<std::endl;
    printf("B:\n");
    std::cout<<B<<std::endl;
    printf("\n");

    return true;
  }

 private:
  void _PrintDebug(double i) {
    printf("[WBDC_new] %f \n", i);
  }

  void _GetSolution(const Eigen::Matrix<double, NUM_QDOT,NUM_QDOT> & A,
                    const Eigen::Matrix<double, NUM_QDOT,1> & cori,
                    const Eigen::Matrix<double, NUM_QDOT,1> & grav,
                    const Eigen::Matrix<double, DIM_RF, NUM_QDOT> & Jc,
                    const Eigen::Matrix<double, 3, NUM_QDOT> & Jcm,
                    const Eigen::Matrix<double, NUM_QDOT, DIM_TASK> & B,
                    const Eigen::Matrix<double, DIM_TASK, 1> & x_task,
                    const Eigen::Matrix<int, DIM_TASK, 1> & selected_task,
                    const Eigen::Matrix<double, NUM_QDOT, 1> & c,
                    Eigen::Matrix<double, NUM_ACT_JOINT, 1> & torque_cmd){
    // Reaction Force
    Eigen::Matrix<double, DIM_RF, 1> Fr;
    for(int i(0); i< DIM_RF; ++i){
      Fr(i, 0)= x[i];
    }

    Eigen::Matrix<double, NUM_QDOT, 1> qddot;
    Eigen::Matrix<double, DIM_TASK, 1> err;
    // Task error
    int k(0);
    for (int i(0); i<DIM_TASK; ++i){
      if (selected_task(i,0) == 0){
        err(i, 0) = 0.;
      } else{
        err(i, 0) = x[k + DIM_RF];
        ++k;
      }
    }
    sejong::pretty_print((sejong::Vector)err, std::cout, "err");
    sejong::pretty_print((sejong::Vector)Fr, std::cout, "Fr");

    // Qddot
    qddot = B * (x_task + err) + c;
    sejong::pretty_print((sejong::Vector)qddot, std::cout, "qddot solution");
    sejong::saveVector((sejong::Vector)qddot,  "qddot_solution");

    Eigen::Matrix<double, NUM_QDOT, 1> tot_tau;
    tot_tau = A* qddot + cori + grav - Jc.transpose() * Fr;
    std::cout<<"tot tau:"<<std::endl;
    std::cout<<tot_tau<<std::endl;
    torque_cmd = tot_tau.tail(NUM_ACT_JOINT);
  }

};


#endif
