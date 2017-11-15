#ifndef NO_ACCELERATION_STATE_ESTIMATOR
#define NO_ACCELERATION_STATE_ESTIMATOR

#include "OriEstimator.hpp"
#define DIM_STATE_NO_ACC_STATE 3*2
#define DIM_OBSER_NO_ACC_STATE 3*2

class NoAccState:public OriEstimator{
public:
  NoAccState();
  virtual ~NoAccState();

  virtual void EstimatorInitialization(const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel);

protected:
  Eigen::Matrix<double, DIM_STATE_NO_ACC_STATE, DIM_STATE_NO_ACC_STATE> F_;
  Eigen::Matrix<double, DIM_OBSER_NO_ACC_STATE, DIM_STATE_NO_ACC_STATE> H_;

  Eigen::Matrix<double, DIM_STATE_NO_ACC_STATE, DIM_STATE_NO_ACC_STATE> P_;
  Eigen::Matrix<double, DIM_STATE_NO_ACC_STATE, DIM_STATE_NO_ACC_STATE> P_pred_;

  Eigen::Matrix<double, DIM_STATE_NO_ACC_STATE, DIM_STATE_NO_ACC_STATE> Q_;
  Eigen::Matrix<double, DIM_OBSER_NO_ACC_STATE, DIM_OBSER_NO_ACC_STATE> R_;
  // vel(3),  ...[orientation (3*)]
  sejong::Vector x_;
  sejong::Vector x_pred_;
  //Observation
  Eigen::Matrix<double, DIM_OBSER_NO_ACC_STATE, 1> y_;
  Eigen::Matrix<double, DIM_OBSER_NO_ACC_STATE, 1> s_;
  Eigen::Matrix<double, DIM_OBSER_NO_ACC_STATE, 1> h_;

  // ori prime
  sejong::Quaternion ori_pred_;


  void _SetGlobalAngularVelocity(const std::vector<double> & ang_vel);

};


#endif
