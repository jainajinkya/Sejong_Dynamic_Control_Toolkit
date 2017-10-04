#ifndef STATE_ESTIMATOR_ 
#define STATE_ESTIMATOR_ 

#include <utils/wrap_eigen.hpp>

#include <vector>
#include "Configuration.h"

#include "Hume_Model/Hume_Model.h"

#include "utils/utilities.h"
#include "StateProvider.h"
#include "Integrator.h"

#include <utils/DataManager.h>

template <int NUM_OBS, int NUM_STATE, int NUM_STATE_DOT, int NUM_NOISE>
class StateEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateEstimator():
        torque_save_(NUM_ACT_JOINT),
        obs_save_(NUM_OBS),
        z_save_(NUM_OBS),
        jpos_save_(NUM_ACT_JOINT),
        est_save_(NUM_STATE),
        pred_save_(NUM_STATE)
        {
            obs_save_.setZero();
            z_save_.setZero();
        
            DataManager::GetDataManager()->RegisterData(&obs_save_, SJ_VEC, "observation", NUM_OBS);
            DataManager::GetDataManager()->RegisterData(&z_save_, SJ_VEC, "prediction", NUM_OBS);
            DataManager::GetDataManager()->RegisterData(&est_save_, SJ_VEC, "estimation", NUM_STATE);
            DataManager::GetDataManager()->RegisterData(&pred_save_, SJ_VEC, "pred_state", NUM_STATE);

            DataManager::GetDataManager()->RegisterData(&torque_save_, SJ_VEC, "torque", NUM_ACT_JOINT);
            DataManager::GetDataManager()->RegisterData(&jpos_save_, SJ_VEC, "jpos", NUM_ACT_JOINT);
    
            model_ = HumeModel::GetHumeModel();
            state_provider_ = StateProvider::GetStateProvider();
            integrator_ = new Integrator(SERVO_RATE);

            P_1_1_.setIdentity();
            P_2_1_.setIdentity();
    
    
            U_ = sejong::Matrix::Zero(NUM_ACT_JOINT, NUM_QDOT);
            U_.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(model_->NAJ(), model_->NAJ());
        }
    
    virtual ~StateEstimator(){}

    void UpdateState(_DEF_SENSOR_DATA_){
        for (int i(0); i < NUM_ACT_JOINT; ++i){
            jpos_save_[i] = jpos[i];
            torque_save_[i] = torque[i];
        }
        Step(_VAR_SENSOR_DATA_);
        _UpdateStateProvider(_VAR_SENSOR_DATA_);
        est_save_ = x_est_;
    }

    void Prediction(const sejong::Vector & torque,
                    const sejong::Matrix & Jc){
        torque_ = torque;
        Jc_ = Jc;
        
        _GeneratePredictionModel();
        integrator_->ForwardDynamics(virtual_configuration_,
                                     virtual_vel_,
                                     torque, Jc);
        
        Eigen::Matrix<double, NUM_STATE_DOT, NUM_STATE_DOT> Q;
        Q = F_*L_*Qc_*L_.transpose()*F_.transpose()*SERVO_RATE;
        
        P_2_1_ =  F_ * P_1_1_ * F_.transpose() + Q;

        pred_save_ = x_pred_;
    }
    const Eigen::Matrix<double, NUM_Q, 1> & getConfiguration(){
        return virtual_configuration_;
    }
    
    virtual void Initialize_Estimator(_DEF_SENSOR_DATA_) =0;

protected:

    // Update Observation Vector
    virtual void _GenerateObservationVector(_DEF_SENSOR_DATA_) = 0;
    // Update z_, H_, R_
    virtual void _GenerateObservationModel() = 0;
    
    // Update F_, Qc_, L_ (and x_pred_)
    virtual void _GeneratePredictionModel() = 0;

    // Update State based on Error (update x_est_)
    virtual void _IntegrateState() = 0;

    // Update Model & Provider based on estimated values
    virtual void _UpdateStateProvider(_DEF_SENSOR_DATA_) = 0;
    
    HumeModel* model_;

    sejong::Matrix Jc_;

    sejong::Vector est_save_;
    sejong::Vector pred_save_;

    sejong::Vector obs_save_;
    sejong::Vector z_save_;
    sejong::Vector torque_save_;
    sejong::Vector jpos_save_;

    sejong::Matrix U_;
    Eigen::Matrix<double, NUM_QDOT,NUM_QDOT> Nc_;

    StateProvider* state_provider_;

    Eigen::Matrix<double, NUM_Q, 1>         virtual_configuration_;
    Eigen::Matrix<double, NUM_QDOT, 1>      virtual_vel_;
    
    Integrator* integrator_;

    Eigen::Matrix<double, NUM_ACT_JOINT, 1> torque_;
    
    Eigen::Matrix<double, NUM_STATE,1> x_est_;
    Eigen::Matrix<double, NUM_STATE,1> x_pred_;    

    Eigen::Matrix<double, NUM_OBS, 1> obs_;

    Eigen::Matrix<double, NUM_OBS, 1> z_;
    Eigen::Matrix<double, NUM_OBS, NUM_STATE_DOT> H_;
    Eigen::Matrix<double, NUM_OBS, NUM_OBS>  R_;

    Eigen::Matrix<double, NUM_STATE_DOT, NUM_STATE_DOT>  F_;
    Eigen::Matrix<double, NUM_NOISE, NUM_NOISE>  Qc_;
    Eigen::Matrix<double, NUM_STATE_DOT, NUM_NOISE>  L_;

    Eigen::Matrix<double, NUM_STATE_DOT, NUM_STATE_DOT> P_1_1_;
    Eigen::Matrix<double, NUM_STATE_DOT, NUM_STATE_DOT> P_2_1_;

    Eigen::Matrix<double, NUM_STATE_DOT, 1> err_;
    void _observation_debuging(){
        sejong::Matrix disp(NUM_OBS, 3);
        disp.block(0, 0, NUM_OBS, 1) = obs_;
        disp.block(0, 1, NUM_OBS, 1) = z_;
        disp.block(0, 2, NUM_OBS, 1) = obs_ - z_;

        // sejong::pretty_print(disp, std::cout, "obs, pred, err","");
    }
    void Step(_DEF_SENSOR_DATA_){
        _GenerateObservationVector(_VAR_SENSOR_DATA_);
        _GenerateObservationModel();

        _observation_debuging();
        // static int count(0);
        // ++count;
        // if(count > 3){
        //     exit(0);
        // }
        // Kalman Filtering
        Eigen::Matrix<double, NUM_OBS, NUM_OBS> S (H_ * P_2_1_ * H_.transpose() + R_);
        Eigen::Matrix<double, NUM_STATE_DOT, NUM_OBS> Kalman_gain ( P_2_1_ * H_.transpose() * S.inverse());
        err_ = Kalman_gain * (obs_ - z_);
        P_1_1_ = (sejong::Matrix::Identity(NUM_STATE_DOT, NUM_STATE_DOT) - Kalman_gain * H_) * P_2_1_;

        _IntegrateState();

        x_pred_ = x_est_;

        // Save for Data Saving
        obs_save_ = obs_;
        z_save_ = z_;

        // std::cout<<"S:\n"<<S<<std::endl;
        // std::cout<<"P_2_1:\n"<<P_2_1_<<std::endl;
        // std::cout<<"Sinv: \n"<<S.inverse()<<std::endl;
        // std::cout<<"Kalman gain: \n"<<Kalman_gain<<std::endl;
        // std::cout<<"H: \n"<<H<<std::endl;
        // std::cout<<"obs\n"<<obs<<std::endl;
        // std::cout<<"predict\n"<<z<<std::endl;
        // std::cout<<"err: \n"<<err<<std::endl;
    }
};

#endif
