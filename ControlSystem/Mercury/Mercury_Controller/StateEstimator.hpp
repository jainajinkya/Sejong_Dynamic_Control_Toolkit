#ifndef STATE_ESTIMATOR_MERCURY
#define STATE_ESTIMATOR_MERCURY

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class StateProvider;
class RobotModel;
class filter;
class OriEstimator;

class StateEstimator{
public:
  StateEstimator();
  ~StateEstimator();

  void Initialization(_DEF_SENSOR_DATA_);
  void Update(_DEF_SENSOR_DATA_);

protected:
  double initial_height_;
  SJLinkID fixed_foot_;
  sejong::Vect3 foot_pos_;
  StateProvider* sp_;
  RobotModel* robot_model_;

  OriEstimator* ori_est_;

  std::vector<filter*> jvel_filter_;
};

#endif
