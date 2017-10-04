#ifndef WBDC_H
#define WBDC_H
// Whole Body Dynamic Controller

#include <vector>
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Optimizer/gurobi/src/gurobi_c++.h>
#include <Configuration.h>

using namespace sejong;
using namespace std;

template <int DIM_RF, int DIM_CM, int DIM_SELECTED_TASK, int DIM_TASK>
class WBDC{
public:
  WBDC(){}
  ~WBDC(){}

  void _SetCost(GRBModel & model, GRBQuadExpr& cost, GRBVar* x){
    double weight(1.);
    for (int i(0); i<DIM_RF; ++i){
      cost += x[i] * x[i];
    }
    for (int i(DIM_RF); i<DIM_RF + DIM_SELECTED_TASK; ++i){
      cost += 10000. * x[i] * x[i];
    }

    for (int i(DIM_RF + DIM_SELECTED_TASK); i<DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL; ++i){
      cost += weight*x[i] * x[i];
    }

    model.setObjective(cost, GRB_MINIMIZE);
  }
  void _SetModel(GRBModel & model, GRBVar * x){
    // Reaction Force 2D
    x[0] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS); // X
    x[1] = model.addVar(5.0, 10000.0, 0.0, GRB_CONTINUOUS); // Z
    x[2] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS); // Ry
    for(int i(0); i<DIM_SELECTED_TASK; ++i){
      x[DIM_RF + i] = model.addVar(-1000.0, 1000.0, 0.0, GRB_CONTINUOUS);
    }
    for(int i(0); i<NUM_VIRTUAL; ++i){
      x[DIM_RF + DIM_SELECTED_TASK + i] = model.addVar(-1000.0, 1000.0, 0.0, GRB_CONTINUOUS);
    }

    model.update();
  }
  void _GetSolution(GRBVar* x,
                    const Eigen::Matrix<double, NUM_QDOT,NUM_QDOT> & A,
                    const Eigen::Matrix<double, NUM_QDOT,1> & cori,
                    const Eigen::Matrix<double, NUM_QDOT,1> & grav,
                    const Eigen::Matrix<double, DIM_RF, NUM_QDOT> & Jc,
                    const Eigen::Matrix<double, DIM_CM, NUM_QDOT> & Jcm,
                    const Eigen::Matrix<double, NUM_QDOT, DIM_TASK> & B,
                    const Eigen::Matrix<double, DIM_TASK, 1> & x_task,
                    const Eigen::Matrix<int, DIM_TASK, 1> & selected_task,
                    const Eigen::Matrix<double, NUM_QDOT, 1> & c,
                    Eigen::Matrix<double, NUM_ACT_JOINT, 1> & torque_cmd){
    // Reaction Force
    Eigen::Matrix<double, DIM_RF, 1> Fr;
    for(int i(0); i< DIM_RF; ++i){
      Fr(i, 0)= x[i].get(GRB_DoubleAttr_X);
    }

    Eigen::Matrix<double, NUM_QDOT, 1> qddot;
    Eigen::Matrix<double, DIM_TASK, 1> err;
    // Task error
    int k(0);
    for (int i(0); i<DIM_TASK; ++i){
      if (selected_task(i,0) == 0){
        err(i, 0) = 0.;
      } else{
        err(i, 0) = x[k + DIM_RF].get(GRB_DoubleAttr_X);
        ++k;
      }
    }
    /* sejong::pretty_print((sejong::Vector)err, std::cout, "err"); */
    // Qddot
    qddot = B * (x_task + err) + c;
    sejong::pretty_print((sejong::Vector)qddot, std::cout, "qddot solution");
    Eigen::Matrix<double, NUM_QDOT, 1> tot_tau;
    tot_tau = A* qddot + cori + grav - Jc.transpose() * Fr;
    std::cout<<"tot tau:"<<std::endl;
    std::cout<<tot_tau<<std::endl;
    torque_cmd = tot_tau.tail(NUM_ACT_JOINT);
  }
  bool MakeTorque(const Eigen::Matrix<double, NUM_QDOT,NUM_QDOT> & A,
                  const Eigen::Matrix<double, NUM_QDOT,1> & cori,
                  const Eigen::Matrix<double, NUM_QDOT,1> & grav,
                  const Eigen::Matrix<double, DIM_RF, NUM_QDOT> & Jc,
                  const Eigen::Matrix<double, DIM_CM, DIM_CM> & Icm,
                  const Eigen::Matrix<double, DIM_CM, NUM_QDOT> & Jcm,
                  const Eigen::Matrix<double, DIM_CM, 1> & JcmDotQdot,
                  const Eigen::Matrix<double, DIM_CM, 1> & grav_cm,
                  const Eigen::Matrix<double, DIM_CM, DIM_RF> & Sf,
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
    try {
      GRBEnv env = GRBEnv();
      GRBModel model  = GRBModel(env);
      model.getEnv().set(GRB_IntParam_OutputFlag, 0);
      GRBVar x[DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL];
      /* GRBVar x[DIM_RF + DIM_SELECTED_TASK]; */
      GRBQuadExpr cost;

      // set model
      /* _SetModel(model, x); */

      double variable_ub(100.0);
      double variable_lb(-100.0);

      // Reaction Force 2D
      x[0] = model.addVar(variable_lb, variable_ub, 0.0, GRB_CONTINUOUS); // X
      x[1] = model.addVar(5.0, 10000.0, 0.0, GRB_CONTINUOUS); // Z
      x[2] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS); // Ry

      for(int i(0); i<DIM_SELECTED_TASK; ++i){
        x[DIM_RF + i] = model.addVar(-1000.0, 1000.0, 0.0, GRB_CONTINUOUS);
      }
      for(int i(0); i<NUM_VIRTUAL; ++i){
        x[DIM_RF + DIM_SELECTED_TASK + i] = model.addVar(-1000.0, 1000.0, 0.0, GRB_CONTINUOUS);
      }
      model.update();

      // set cost
      _SetCost(model, cost, x);

      // set constraint
      _PrintDebug(1.0);
      // Qddot Generation
      GRBLinExpr qddot[NUM_QDOT];
      int k(0);
      for (int i(0); i<NUM_QDOT; ++i){
        k = 0;
        for(int j(0); j< DIM_TASK; ++j){
          if(selected_task[j] == 1){ // selection
            qddot[i] += B(i,j) * (x_task(j, 0)  + x[DIM_RF + k]);
            ++k;
          } else {
            qddot[i] += B(i,j) * x_task(j, 0);
          }
        }
        qddot[i] += c(i, 0);
      }
      // Jcm * ddot(q)
      GRBLinExpr JcmQddot[DIM_CM];

      for (int i(0); i<DIM_CM ; ++i){
        for (int j(0); j<NUM_QDOT; ++j){
          JcmQddot[i] += Jcm(i,j) * qddot[j];
        }
      }
      std::cout<<Jcm<<std::endl;
      _PrintDebug(1.11);

      // CM task (Fcm)
      GRBLinExpr Fcm[DIM_CM];
      for (int i(0); i<DIM_CM; ++i){
        for (int j(0); j<DIM_CM; ++j){
          Fcm[i] += Icm(i, j) * (JcmQddot[j] - JcmDotQdot(j,0));
        }
        Fcm[i] += grav_cm(i, 0);
      }
      _PrintDebug(1.12);
      std::cout<<Sf<<std::endl;
      // Fcm = Sf * Fr
      for (int i(0); i<DIM_CM; ++i){
        GRBLinExpr w;
        for (int j(0); j<DIM_RF; ++j){
          w += Sf(i, j) * x[j];
        }
        model.addConstr(Fcm[i] - w, GRB_EQUAL, 0);
      }
      _PrintDebug(1.2);

      // A * qddot + b + g
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
      _PrintDebug(1.3);

      GRBLinExpr tot_tau[NUM_QDOT - NUM_VIRTUAL];
      int act_jidx(0);
      for (int i(0); i<NUM_VIRTUAL; ++i){
        model.addConstr(x[i + DIM_RF + DIM_SELECTED_TASK], GRB_EQUAL, Aqddot_b_g[i] - Jc_trans_Fr[i]);
        /* model.addConstr(0., GRB_EQUAL, Aqddot_b_g[i] - Jc_trans_Fr[i]); */
      }

      ///// Force limit
      for (int i(NUM_VIRTUAL); i<NUM_QDOT; ++i){
        GRBLinExpr torque = Aqddot_b_g[i] - Jc_trans_Fr[i];
        model.addConstr(torque, GRB_GREATER_EQUAL, tau_min[i - NUM_VIRTUAL]);
        model.addConstr(tau_max[i - NUM_VIRTUAL], GRB_GREATER_EQUAL, torque);
      }

      // Friction
      model.addConstr(mu*x[1],GRB_GREATER_EQUAL,x[0]);
      model.addConstr(x[0],GRB_GREATER_EQUAL,-mu*x[1]);
      model.addConstr(x[1],GRB_GREATER_EQUAL, 5.0 * (l1 + l2)/l1 - x[2]/l1); // Fz + tau_y/l1 > 0
      model.addConstr(x[1],GRB_GREATER_EQUAL, 5.0 * (l1 + l2)/l2 + x[2]/l2); // Fz - tau_y/l2 > 0

      for(int i(0);i<DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL; ++i){
        x[i].set(GRB_DoubleAttr_Start, 0.0);
      }
      _PrintDebug(2.0);
      model.optimize();

      _PrintDebug(3.0);
      for (int i(0); i<DIM_RF+DIM_SELECTED_TASK + NUM_VIRTUAL; ++i){
        opt_result[i] = x[i].get(GRB_DoubleAttr_X);
      }

      // Return torque command
      _GetSolution(x, A, cori, grav, Jc, Jcm,
                   B, x_task, selected_task, c, torque_cmd);
      return true;
    }
    catch(GRBException e) {
      cout << "Error code = " << e.getErrorCode() << endl;
      cout << e.getMessage() << endl;
      /* exit(0); */
      return false;
    }
  }

private:
  void _PrintDebug(double i) {
    printf("[WBDC] %f \n", i);
  }

};


#endif
