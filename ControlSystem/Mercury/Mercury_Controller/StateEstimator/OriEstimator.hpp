#ifndef ORIENTATION_ESTIMATOR
#define ORIENTATION_ESTIMATOR

#include <Utils/wrap_eigen.hpp>

class OriEstimator{
public:
  OriEstimator(){}
  virtual ~OriEstimator(){}

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel) = 0;

  virtual void EstimatorInitialization(const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel) = 0;


  void getEstimatedState(sejong::Quaternion & ori,
                         sejong::Vect3 & global_ang_vel){
    ori = global_ori_;
    global_ang_vel = global_ang_vel_;
  }

protected:
  sejong::Quaternion global_ori_;
  sejong::Vect3 global_ang_vel_;
};

#endif
