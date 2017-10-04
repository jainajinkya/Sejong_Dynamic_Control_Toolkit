#ifndef WBLC_REACTION_WRENCH_CALCULATOR_GUROBI
#define WBLC_REACTION_WRENCH_CALCULATOR_GUROBI

#include <utils/wrap_eigen.hpp>
#include <utils/utilities.h>

#include "ContactWrenchCalculator.h"

#include <gurobi/src/gurobi_c++.h>
#include <Configuration.h>
#include <utils/pseudo_inverse.hpp>

using namespace std;

template <int NUM_OPT_VAR, int NUM_CONTACT>
class WBLC_Wrench_Cal : public ContactWrenchCalculator{
    public:
        WBLC_Wrench_Cal(WBLC_Model* model, ContactHandler* contacthandler):ContactWrenchCalculator(model, contacthandler), g(9.81){
            W_ = sejong::Matrix::Zero(6, NUM_OPT_VAR);
            mu_ = sejong::Vector::Zero(NUM_CONTACT);
            for (int i = 0; i < NUM_CONTACT; ++i) {
                mu_[i] = contact_handler_->contact_list[i]->GetStatFricCoef();
            }
        }

        virtual ~WBLC_Wrench_Cal(){ }

        virtual void PrintName(){  }

        virtual void PrepareOptimization(){
            _SetMatrixW();
            _SetCoMAcc();
            _SetContactJacobian();

            b_prepared_ = true;
        }

    protected:
        sejong::Vector F_cm_;

        void _SetF_Centroid(){
            sejong::Matrix Jcm_ang, Jcm, Ainv;
            sejong::Vector coriolis;
            robot_model_->getCentroidJacobian(Jcm);
            Jcm_ang = Jcm.block(0,0,3, NUM_QDOT);
            robot_model_->getInverseMassInertia(Ainv);
            robot_model_->getCoriolis(coriolis);

            sejong::Vector Jcm_ang_dot_qdot = Jcm_ang * Ainv * coriolis;

            // Centroidal Acceleration Setting
            cm_acc_.head(3) = CAM_acc_ + Jcm_ang_dot_qdot;
            cm_acc_.tail(3) = CoM_acc_;
            cm_acc_[5] += g;

            sejong::Matrix Icm;
            robot_model_->getCentroidInertia(Icm);

            F_cm_ = Icm * cm_acc_;
        }

        bool _Optimization_Gurobi(sejong::Vector & Wr){
            _SetF_Centroid();
            try {
                GRBEnv env = GRBEnv();
                GRBModel model  = GRBModel(env);
                model.getEnv().set(GRB_IntParam_OutputFlag, 0);
                GRBVar x[NUM_OPT_VAR];
                GRBQuadExpr cost;

                _SetModel(lb, ub, model, x, NUM_OPT_VAR);
                _SetCost(model, cost, x, NUM_OPT_VAR);
                _SetConstraint(model, x, mu_, NUM_OPT_VAR);

                for(int i(0);i<NUM_OPT_VAR; ++i){
                    x[i].set(GRB_DoubleAttr_Start, 0.0);
                }

                sejong::Vector g;
                robot_model_->getGravity(g);
                double num_cp = (NUM_OPT_VAR)/3.;

                for(int i(0);i<NUM_OPT_VAR; ++i){
                    if((i)%3 == 2){
                        x[i].set(GRB_DoubleAttr_Start, g[2]/num_cp);
                    }
                    x[i].set(GRB_DoubleAttr_Start, 0.0);
                }
                model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
                model.optimize();

                _GetSolution(x, Wr, NUM_OPT_VAR);
                return false; // Normal Situation
            }
            catch(GRBException e) {
                printf("[Reaction Force Cal] Emergency Calculation\n");
                try {
                    GRBEnv env_emergency = GRBEnv();
                    GRBModel model_emergency = GRBModel(env_emergency);
                    model_emergency.getEnv().set(GRB_IntParam_OutputFlag, 0);
                    GRBVar x_emergency[NUM_OPT_VAR];
                    GRBQuadExpr cost_emergency;

                    _SetModel(lb, ub, model_emergency, x_emergency, NUM_OPT_VAR);
                    _SetCost(model_emergency, cost_emergency, x_emergency, NUM_OPT_VAR);
                    _SetConstraint(model_emergency, x_emergency, mu_emergency, NUM_OPT_VAR);
                    for(int i(0);i<NUM_OPT_VAR; ++i){
                        x_emergency[i].set(GRB_DoubleAttr_Start, 0.0);
                    }
                    model_emergency.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
                    model_emergency.optimize();

                    _GetSolution(x_emergency, Wr, NUM_OPT_VAR);
                    return true; // Emergency
                }
                catch(GRBException e) {
                    cout << "Error code = " << e.getErrorCode() << endl;
                    cout << e.getMessage() << endl;
                    exit(0);
                }
            }

        }
        virtual void _SetModel(double* lb, double * ub, GRBModel& model, GRBVar* x, int num_opt) = 0;
        virtual void _SetCost(GRBModel& model, GRBQuadExpr& cost, GRBVar* x, int num_opt) = 0;
        virtual void _SetConstraint(GRBModel& model, GRBVar* x, sejong::Vector mu_, int num_opt) = 0;
        virtual bool _CalculateReactionForce(sejong::Vector & Wr){
            Wr = sejong::Vector::Zero(NUM_OPT_VAR);
            return _Optimization_Gurobi(Wr);
        }
        sejong::Vector mu_;
        //double mu_emergency;
        sejong::Vector mu_emergency;
        double g;
        double lb[NUM_OPT_VAR];
        double ub[NUM_OPT_VAR];

        void _SetCoMAcc(){
            sejong::Vector dummy_Q(NUM_Q);
            sejong::Vector dummy_Qdot(NUM_QDOT);

            sejong::Vect3 com_pos, com_vel;
            robot_model_->getCoMPosition(dummy_Q, com_pos);
            robot_model_->getCoMVelocity(dummy_Q, dummy_Qdot, com_vel);

            // Linear Only
            for(int i(0); i<3; ++i){
                CoM_acc_[i] = des_ff_[i] + Kp_[i] *(des_conf_[i] - com_pos[i]) + Kv_[i] * (des_vel_[i] - com_vel[i]);
            }
        }

        void _NormalCostSetting(GRBVar* x, GRBModel& model, GRBQuadExpr& cost, int num_opt ){
            for (int i(0); i<num_opt; ++i){
                cost += 1.0 * ( x[i]) * ( x[i] );
            }
            // Angular Constraint is cost Change Constraint Setting too
            for (int i(0); i < 3; ++i) {
                GRBLinExpr w;
                for(int j(0); j< num_opt ; ++j){
                    w += W_(i,j)*x[j];
                }
                cost += 5000.0 * (F_cm_[i] - w) * (F_cm_[i] - w);
            }

            model.setObjective(cost, GRB_MINIMIZE);
        }

        void _NormalConstraintSetting(GRBModel& model, GRBVar* x, sejong::Vector mu_, int num_opt){
            //Constraint for linear momentum
            for (int i(3); i < 6; ++i) {
                GRBLinExpr w;
                for(int j(0); j< num_opt ; ++j){
                    w += W_(i,j)*x[j];
                }
                model.addConstr(F_cm_[i] - w, GRB_EQUAL, 0);
            }

            // Wrench Cone
            sejong::Matrix H_rep;
            for (int i = 0; i < NUM_CONTACT; ++i) {
                H_rep = contact_handler_->contact_list[i]->GetWrenchFace();
                for (int j = 0; j < H_rep.rows(); ++j) {
                    GRBLinExpr w;
                    for (int k = 0; k < H_rep.cols(); ++k) {
                        w += H_rep(j, k)*x[6*i+k];
                    }
                    model.addConstr(w <= 0);
                }
            }
        }

        void _NormalModelSetting(double * lb, double * ub,
                GRBModel& model, GRBVar* x,  int num_opt){
            for(int i(0); i<num_opt; ++i){
                lb[i] = -1000.0;
                ub[i] = 1000.0;
            }

            for(int i(0); i<num_opt; ++i){
                x[i]=model.addVar(lb[i], ub[i], 0.0, GRB_CONTINUOUS);
            }
            model.update();
        }

        void _GetSolution(GRBVar* x, sejong::Vector& Wr, int num_opt){
            for(int i(0); i<num_opt; ++i){
                Wr[i]= x[i].get(GRB_DoubleAttr_X);
            }

            sejong::Vector F_cm = W_ * Wr;
            sejong::Matrix Icm;
            robot_model_->getCentroidInertia(Icm);

            cm_acc_ = Icm.inverse() * F_cm;
            cm_acc_[5] -= 9.81;

            sejong::Matrix Jcm;
            robot_model_->getCentroidJacobian(Jcm);
            sejong::Vector cori;
            robot_model_->getCoriolis(cori);
            sejong::Matrix Ainv;
            robot_model_->getInverseMassInertia(Ainv);
            sejong::Vector Jdot_qdot = Jcm * Ainv * cori;
            cm_acc_ -= Jdot_qdot;
        }
};

#endif
