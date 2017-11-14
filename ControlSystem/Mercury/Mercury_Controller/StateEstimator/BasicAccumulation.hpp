#ifndef ORIENTATION_ESTIMATOR_IMU
#define ORIENTATION_ESTIMATOR_IMU

#include "OriEstimator.hpp"

class BasicAccumulation:public OriEstimator{
public:
  BasicAccumulation();
  virtual ~BasicAccumulation();

  virtual void EstimatorInitialization(const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & ang_vel,
                             const std::vector<double> & ang_vel_inc);
};

#endif
