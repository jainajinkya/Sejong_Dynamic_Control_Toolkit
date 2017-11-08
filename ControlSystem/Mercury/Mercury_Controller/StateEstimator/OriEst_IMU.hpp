#ifndef ORIENTATION_ESTIMATOR_IMU
#define ORIENTATION_ESTIMATOR_IMU

#include <Utils/wrap_eigen.hpp>

class OriEst_IMU{
public:
  OriEst_IMU();
  ~OriEst_IMU();

  void EstimatorInitialization(const std::vector<double> & acc,
                               const std::vector<double> & ang_vel);
  void setSensorData(const std::vector<double> & acc,
                     const std::vector<double> & ang_vel);
  void getEstimatedState(sejong::Quaternion & ori,
                         sejong::Vect3 & global_ang_vel);

private:
  sejong::Quaternion ori_imu_;
  sejong::Vect3 global_ang_vel_;
};

#endif
