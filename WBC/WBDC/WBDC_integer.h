#ifndef WBDC_2D_INTEGER_H
#define WBDC_2D_INTEGER_H

#include <vector>
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Configuration.h>
#include <Optimizer/gurobi/src/gurobi_c++.h>

using namespace sejong;
using namespace std;

#define DIM_CM 3
#define DIM_CAM 1
#define DIM_FRICTION 5

template <int DIM_RF, int DIM_NONSLIP_TASK, int DIM_MOTION_TASK, int DIM_SLIP_TASK>
  class WBDC_integer{
#define DIM_OPT_VAR (DIM_RF+DIM_MOTION_TASK+4*DIM_SLIP_TASK)
#define DIM_TASK (DIM_NONSLIP_TASK + DIM_MOTION_TASK + DIM_SLIP_TASK)
 public:
  double mu;
  double l1;
  double l2;
  GRBVar x[DIM_OPT_VAR];

 WBDC_integer(): mu(0.3), l1(0.12), l2(0.08){
  }
  ~WBDC_integer(){}

  void _SetCost(GRBModel & model, GRBQuadExpr& cost, GRBVar* x){

    /* for (int i(0); i<DIM_RF; ++i){ */
    /*   cost += 0.1 * x[i] * x[i]; */
    /* } */

    cost += 1. * x[0] * x[0];
    cost += 0.001 * x[1] * x[1];
    cost += 1. * x[2] * x[2];

    for (int i(DIM_RF); i<DIM_RF + DIM_MOTION_TASK; ++i){
      cost += 100. * x[i] * x[i];
    }
    double weight(0.1);
    for (int i(DIM_RF + DIM_MOTION_TASK); i<DIM_RF + DIM_MOTION_TASK + 2*DIM_SLIP_TASK;++i){
      cost += weight*x[i] * x[i];
    }
    model.setObjective(cost, GRB_MINIMIZE);
  }

  bool MakeTorque(
      const Eigen::Matrix<double, NUM_QDOT,NUM_QDOT> & A,
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
      const Eigen::Matrix<double, NUM_QDOT, 1> & c,
      const Eigen::Matrix<double, NUM_ACT_JOINT, 1> & tau_min,
      const Eigen::Matrix<double, NUM_ACT_JOINT, 1> & tau_max,
      Eigen::Matrix<double, NUM_ACT_JOINT, 1> & torque_cmd,
      Eigen::Matrix<double, DIM_OPT_VAR, 1> & opt_result){

    sejong::Vector qddot_tmp = B*x_task + c;
    sejong::pretty_print((sejong::Matrix)B, std::cout, "B");
    sejong::pretty_print((sejong::Vector)x_task, std::cout, "x_task");
    sejong::pretty_print(qddot_tmp, std::cout, "q ddot");

    sejong::Vector tau_left = A * qddot_tmp + cori + grav;
    sejong::pretty_print(tau_left, std::cout, "tau left");
    sejong::pretty_print((sejong::Matrix)Jc, std::cout, "Jc");

    try{
      GRBEnv env = GRBEnv();
      GRBModel model  = GRBModel(env);
      model.getEnv().set(GRB_IntParam_OutputFlag, 0);
      GRBQuadExpr cost;

      GRBVar x[DIM_OPT_VAR];

      x[0] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS); // fx
      x[1] = model.addVar(0.0, 10000.0, 0.0, GRB_CONTINUOUS); // fz
      x[2] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS); // tau_y

      x[3] = model.addVar(-0.01, 0.01, 0., GRB_CONTINUOUS); // body x
      x[4] = model.addVar(-.01, .01, 0., GRB_CONTINUOUS); // body z

      x[5] = model.addVar(0.0, 120.0, 0., GRB_CONTINUOUS); // d1 (pos)
      x[6] = model.addVar(0.0, 120.0, 0., GRB_CONTINUOUS); // d2 (neg)
      x[7] = model.addVar(0., 1., 0., GRB_BINARY); // etha1 (pos)
      x[8] = model.addVar(0., 1., 0., GRB_BINARY); // etha2 (neg)

      model.update();

      // set cost
      _SetCost(model, cost, x);

      _PrintDebug(1.0);
      // Qddot Generation
      GRBLinExpr qddot[NUM_QDOT];
      int k(0);
      for (int i(0); i<NUM_QDOT; ++i){
        for(int j(0); j< DIM_NONSLIP_TASK; ++j){
          qddot[i] += B(i,j) * (x_task[j]);
        }
        for(int j(0); j< DIM_MOTION_TASK; ++j){
          qddot[i] += B(i, DIM_NONSLIP_TASK + j) * (x_task[DIM_NONSLIP_TASK + j]  + x[DIM_RF + j]);
        }
        qddot[i] += B(i, DIM_NONSLIP_TASK + DIM_MOTION_TASK) * (x_task[DIM_NONSLIP_TASK + DIM_MOTION_TASK] + x[5] - x[6]);

        qddot[i] += c[i];
      }

      GRBLinExpr Aqddot_b_g[NUM_QDOT];
      for (int i(0); i<NUM_QDOT; ++i){
        for (int j(0); j<NUM_QDOT; ++j){
          Aqddot_b_g[i] += A(i,j) * qddot[j];
        }
        Aqddot_b_g[i] += (cori(i,0) + grav(i,0));
      }

      // Jc.transpose() * Fr
      GRBLinExpr Jc_trans_Fr[NUM_QDOT];
      for (int i(0); i<NUM_QDOT; ++i){
        for (int j(0); j<DIM_RF; ++j){
          Jc_trans_Fr[i] += Jc(j, i) * x[j];
        }
      }

      // Virtual torque = 0
      for (int i(0); i<NUM_VIRTUAL; ++i){
        model.addConstr(0., GRB_EQUAL, Aqddot_b_g[i] - Jc_trans_Fr[i]);
      }
      // Force limit
      for (int i(NUM_VIRTUAL); i<NUM_QDOT; ++i){
        GRBLinExpr torque = Aqddot_b_g[i] - Jc_trans_Fr[i];
        model.addConstr(torque, GRB_GREATER_EQUAL, tau_min[i - NUM_VIRTUAL]);
        model.addConstr(tau_max[i - NUM_VIRTUAL], GRB_GREATER_EQUAL, torque);
      }

      ////// Set Ineqaulity
      double eps(0.0001);
      double d1max(100.);
      double d2max(100.);
      double Mx(5000.); double mm(-5000.);
      // slip forward
      model.addConstr(x[7] * (d1max - eps), GRB_GREATER_EQUAL, x[5] - eps);
      model.addConstr(x[5] - eps, GRB_GREATER_EQUAL, (1 - x[7]) * ( - eps));
      model.addConstr(x[0] + mu * x[1], GRB_GREATER_EQUAL, 0.);
      model.addConstr((1-x[7])*Mx, GRB_GREATER_EQUAL, x[0] + mu * x[1]);
      // slip backward
      model.addConstr(x[8] * (d2max - eps), GRB_GREATER_EQUAL, x[6]-eps);
      model.addConstr(x[6] - eps, GRB_GREATER_EQUAL, (1-x[8]) * (-eps));
      model.addConstr(0., GRB_GREATER_EQUAL, x[0] - mu * x[1]);
      model.addConstr(x[0]- mu*x[1], GRB_GREATER_EQUAL,  (1-x[8])*mm);

      model.addConstr((1-x[7])*d2max, GRB_GREATER_EQUAL, x[6]);
      model.addConstr((1-x[8])*d1max, GRB_GREATER_EQUAL, x[5]);

      // set constraint
      // Jcm * ddot(q)
      GRBLinExpr JcmQddot[DIM_CM];

      for (int i(0); i<DIM_CM ; ++i){
        for (int j(0); j<NUM_QDOT; ++j){
          JcmQddot[i] += Jcm(i,j) * qddot[j];
        }
      }
      _PrintDebug(1.11);

      // CAM task (Fcm)
      GRBLinExpr Fcam;
      for (int j(0); j<DIM_CM; ++j){
        Fcam += Icm(2, j) * (JcmQddot[j] + JcmDotQdot[j]);
      }
      Fcam += grav_cm(2, 0);

      // Fcm = Sf * Fr
      GRBLinExpr w;
      for (int j(0); j<DIM_RF; ++j){
        w += Sf(2, j) * x[j];
      }
      /* model.addConstr(Fcam - w, GRB_EQUAL, 0); */
      _PrintDebug(1.2);

      // Friction
      model.addConstr(x[1],GRB_GREATER_EQUAL, 5.0 * (l1 + l2)/l1 - x[2]/l1); // Fz + tau_y/l1 > 0
      model.addConstr(x[1],GRB_GREATER_EQUAL, 5.0 * (l1 + l2)/l2 + x[2]/l2); // Fz - tau_y/l2 > 0

      _PrintDebug(2.0);
      model.optimize();

      _PrintDebug(3.0);
      for (int i(0); i< DIM_OPT_VAR; ++i){
        opt_result[i] = x[i].get(GRB_DoubleAttr_X);
      }
      sejong::pretty_print((sejong::Vector)opt_result, std::cout, "opt result");

      // Return torque command
      _GetSolution(opt_result, A, cori, grav, Jc, Jcm,
                   B, x_task,  c, torque_cmd);
      return true;
    }
    catch(GRBException e) {
      cout << "Error code = " << e.getErrorCode() << endl;
      cout << e.getMessage() << endl;
      exit(0);
      return false;
    }
  }

 private:
  void _PrintDebug(double i) {
    printf("[WBDC_integer] %f \n", i);
  }

  void _GetSolution(const Eigen::Matrix<double, DIM_OPT_VAR, 1> & x,
                    const Eigen::Matrix<double, NUM_QDOT,NUM_QDOT> & A,
                    const Eigen::Matrix<double, NUM_QDOT,1> & cori,
                    const Eigen::Matrix<double, NUM_QDOT,1> & grav,
                    const Eigen::Matrix<double, DIM_RF, NUM_QDOT> & Jc,
                    const Eigen::Matrix<double, 3, NUM_QDOT> & Jcm,
                    const Eigen::Matrix<double, NUM_QDOT, DIM_TASK> & B,
                    const Eigen::Matrix<double, DIM_TASK, 1> & x_task,
                    const Eigen::Matrix<double, NUM_QDOT, 1> & c,
                    Eigen::Matrix<double, NUM_ACT_JOINT, 1> & torque_cmd){
    // Reaction Force
    Eigen::Matrix<double, DIM_RF, 1> Fr;
    for(int i(0); i< DIM_RF; ++i){
      Fr(i, 0)= x[i];
    }

    Eigen::Matrix<double, NUM_QDOT, 1> qddot;
    Eigen::Matrix<double, DIM_NONSLIP_TASK + DIM_MOTION_TASK + DIM_SLIP_TASK, 1> err;
    err.setZero();
    // Task error
    int k(0);
    for (int i(0); i<DIM_MOTION_TASK; ++i){
        err[i + DIM_NONSLIP_TASK] = x[i + DIM_RF];
    }
    for (int i(0); i< DIM_SLIP_TASK; ++i){
      err[i + DIM_NONSLIP_TASK + DIM_MOTION_TASK] = x[2*i + DIM_RF + DIM_MOTION_TASK];
      err[i + DIM_NONSLIP_TASK + DIM_MOTION_TASK] -= x[2*i + 1 + DIM_RF + DIM_MOTION_TASK];
    }

    sejong::pretty_print((sejong::Vector)err, std::cout, "err");
    sejong::pretty_print((sejong::Vector)Fr, std::cout, "Fr");

    // Qddot
    qddot = B * (x_task + err) + c;
    sejong::pretty_print((sejong::Vector)qddot, std::cout, "qddot solution");
    sejong::Vector tau_left = A* qddot + cori + grav;
    sejong::pretty_print((sejong::Matrix)A, std::cout, "A");
    sejong::pretty_print((sejong::Vector)cori, std::cout, "cori");
    sejong::pretty_print((sejong::Vector)grav, std::cout, "grav");
    sejong::pretty_print(tau_left, std::cout,  "tau left_solution");

    Eigen::Matrix<double, NUM_QDOT, 1> tot_tau;
    tot_tau = A* qddot + cori + grav - Jc.transpose() * Fr;
    std::cout<<"tot tau:"<<std::endl;
    std::cout<<tot_tau<<std::endl;
    torque_cmd = tot_tau.tail(NUM_ACT_JOINT);
  }

};


#endif
