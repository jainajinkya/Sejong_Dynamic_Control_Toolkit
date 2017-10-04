#ifndef WBDC_3D_H
#define WBDC_3D_H
// Whole Body Dynamic Controller 3D 

#include <vector>
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Optimizer/gurobi/src/gurobi_c++.h>
#include <Configuration.h>
#include <chrono>

#define TIME_MEASURE_WBDC_3D

using namespace sejong;
using namespace std;
#define DIM_CM 6
template <int DIM_RF, int DIM_SELECTED_TASK, int DIM_TASK>
class WBDC_3D{
public:
  WBDC_3D(){}
  ~WBDC_3D(){}

  void _SetCost(GRBModel & model, GRBQuadExpr& cost, GRBVar* x){
    double weight(1.);
    for (int i(0); i<DIM_RF; ++i){
      cost += x[i] * x[i];
    }
    for (int i(DIM_RF); i<DIM_RF + DIM_SELECTED_TASK; ++i){
      cost += 1000. * x[i] * x[i];
    }

    for (int i(DIM_RF + DIM_SELECTED_TASK); i<DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL; ++i){
      cost += weight*x[i] * x[i];
    }
    model.setObjective(cost, GRB_MINIMIZE);
  }

  void _SetModel(GRBModel & model, GRBVar * x){
    // Reaction Force 3D Double contact (tau_x, tau_y, tau_z, fx, fy, fz)
    // Right
    double F_hori_lim(500.);
    for (int i(0); i<5; ++i){
      x[i] = model.addVar(-F_hori_lim, F_hori_lim, 0.0, GRB_CONTINUOUS);
    }
    x[5] = model.addVar(5.0, 2000.0, 0.0, GRB_CONTINUOUS); // Z
    // Left
    for (int i(6); i<11; ++i){
      x[i] = model.addVar(-F_hori_lim, F_hori_lim, 0.0, GRB_CONTINUOUS);
    }
    x[11] = model.addVar(5.0, 2000.0, 0.0, GRB_CONTINUOUS); // Z

    for(int i(0); i<DIM_SELECTED_TASK; ++i){
      x[DIM_RF + i] = model.addVar(-50.0, 50.0, 0.0, GRB_CONTINUOUS);
    }
    for(int i(0); i<NUM_VIRTUAL; ++i){
      x[DIM_RF + DIM_SELECTED_TASK + i] = model.addVar(-350.0, 350.0, 0.0, GRB_CONTINUOUS);
    }
    model.update();
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
                  const Eigen::Matrix<double, 16, 6> & UcR_right,
                  const Eigen::Matrix<double, 16, 6> & UcR_left,
                  const Eigen::Matrix<double, NUM_ACT_JOINT, 1> & tau_min,
                  const Eigen::Matrix<double, NUM_ACT_JOINT, 1> & tau_max,
                  Eigen::Matrix<double, NUM_ACT_JOINT, 1> & torque_cmd,
                  Eigen::Matrix<double, DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL, 1> & opt_result){

    try {
      GRBEnv env = GRBEnv();
      GRBModel model  = GRBModel(env);
      model.getEnv().set(GRB_IntParam_OutputFlag, 0);
      GRBVar x[DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL];
      GRBQuadExpr cost;

      // set model
      _SetModel(model, x);

      // set cost
      _SetCost(model, cost, x);

      // set constraint


      // Qddot Generation
      GRBLinExpr qddot[NUM_QDOT];
      int k(0);
      for (int i(0); i<NUM_QDOT; ++i){
        k = 0;
        for(int j(0); j< DIM_TASK; ++j){
          if(selected_task[j] == 0){ // No selection
            qddot[i] += B(i,j) * x_task(j, 0);
          } else {
            qddot[i] += B(i,j) * (x_task(j, 0) + x[DIM_RF + k]);
            ++k;
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

      // CM task (Fcm)
      GRBLinExpr Fcm[DIM_CM];
      for (int i(0); i<DIM_CM; ++i){
        for (int j(0); j<DIM_CM; ++j){
          Fcm[i] += Icm(i, j) * (JcmQddot[j] - JcmDotQdot(j,0));
        }
        Fcm[i] += grav_cm(i, 0);
      }
      // Fcm = Sf * Fr
      for (int i(0); i<DIM_CM; ++i){
        GRBLinExpr w;
        for (int j(0); j<DIM_RF; ++j){
          w += Sf(i, j) * x[j];
        }
        model.addConstr(Fcm[i] - w, GRB_EQUAL, 0);
      }

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

      GRBLinExpr tot_tau[NUM_QDOT - NUM_VIRTUAL];
      int act_jidx(0);
      for (int i(0); i<NUM_VIRTUAL; ++i){
        model.addConstr(x[i + DIM_RF + DIM_SELECTED_TASK], GRB_EQUAL, Aqddot_b_g[i] - Jc_trans_Fr[i]);
      }

      ///// Force limit
      for (int i(NUM_VIRTUAL); i<NUM_QDOT; ++i){
        GRBLinExpr torque = Aqddot_b_g[i] - Jc_trans_Fr[i];
        model.addConstr(torque, GRB_GREATER_EQUAL, tau_min[i - NUM_VIRTUAL]);
        model.addConstr(tau_max[i - NUM_VIRTUAL], GRB_GREATER_EQUAL, torque);
      }

      ///// Friction
      // Right
      for (int i(0); i<16; ++i){
        GRBLinExpr UrFr;
        for(int j(0); j<6; ++j){
          UrFr += UcR_right(i, j) * x[j];
        }
        model.addConstr(0., GRB_GREATER_EQUAL, UrFr);
      }
      // Left
      for (int i(0); i<16; ++i){
        GRBLinExpr UrFr;
        for(int j(0); j<6; ++j){
          UrFr += UcR_left(i, j) * x[j + 6];
        }
        model.addConstr(0., GRB_GREATER_EQUAL, UrFr);
      }

      /* Optimization Solving */
      for(int i(0);i<DIM_RF + DIM_SELECTED_TASK + NUM_VIRTUAL; ++i){
        x[i].set(GRB_DoubleAttr_Start, 0.0);
      }


#ifdef TIME_MEASURE_WBDC_3D
      static int count(0);
      static double time_sum(0.);
      std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
      // Optimization Solving
      model.optimize();

#ifdef TIME_MEASURE_WBDC_3D

      std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
      time_sum += (time_span1.count()*1000.0);
      ++count;
      if (count % 100 == 99){
        double ave_time = time_sum/((double)count);
        std::cout << "Optimization took " << ave_time << "ms."<<std::endl;
        count = 0;
        time_sum = 0.;
      }
#endif

      for (int i(0); i<DIM_RF+DIM_SELECTED_TASK + NUM_VIRTUAL; ++i){
        /* std::cout<<x[i].get(GRB_DoubleAttr_X)<<std::endl; */
        opt_result[i] = x[i].get(GRB_DoubleAttr_X);
      }
      /* printf("\n"); */
      /// Return torque command
      // Reaction Force
      Eigen::Matrix<double, DIM_RF, 1> Fr;
      for(int i(0); i< DIM_RF; ++i){
        Fr(i, 0)= x[i].get(GRB_DoubleAttr_X);
      }

      Eigen::Matrix<double, NUM_QDOT, 1> qddot_result;
      Eigen::Matrix<double, DIM_TASK, 1> err;

      // Task error
      int relaxed_task_idx(0);
      for (int i(0); i<DIM_TASK; ++i){
        if (selected_task(i,0) == 0){
          err(i, 0) = 0.;
        } else{
          err(i, 0) = x[relaxed_task_idx + DIM_RF].get(GRB_DoubleAttr_X);
          ++relaxed_task_idx;
        }
      }
      // Qddot
      qddot_result = B * (x_task + err) + c;
      Eigen::Matrix<double, NUM_QDOT, 1> total_tau_result;
      total_tau_result = A* qddot_result + cori + grav - Jc.transpose() * Fr;
      /* sejong::pretty_print(total_tau_result, std::cout, "total tau"); */
      torque_cmd = total_tau_result.tail(NUM_ACT_JOINT);

      return true;
    }
    catch(GRBException e) {
      cout << "Error code = " << e.getErrorCode() << endl;
      cout << e.getMessage() << endl;
    }
  }

private:
  void _PrintDebug(double i) {
    /* printf("[WBDC_3D] %f \n", i); */
  }

};


#endif
