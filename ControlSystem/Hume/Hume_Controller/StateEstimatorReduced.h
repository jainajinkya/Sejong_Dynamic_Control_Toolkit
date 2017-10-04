#ifndef STATE_ESTIMATOR_REDUCED 
#define STATE_ESTIMATOR_REDUCED 

#include "StateEstimator.h"
#include "MoCap_Receiver.h"

#define NUM_LED_REDUCE 5

// (LED Position)
#define NUM_OBS_REDUCE (3*NUM_LED_REDUCE)
// (X, Xdot, Q, bias_f, bias_w)
#define NUM_STATE_DOT_REDUCE  (3 + 3 + 3 + 3 + 3)
#define NUM_STATE_REDUCE  (3 + 3 + 4 + 3 + 3)
// (w_vel, w_f, w_w, w_{bf}, w_{bw})
#define NUM_NOISE_REDUCE (3 + 3 + 3 + 3 + 3)
class HumeModel;
class StateProvider;
class Integrator;
class MoCapReceiver;

using namespace sejong;

class StateEstimatorReduced : public StateEstimator<NUM_OBS_REDUCE, NUM_STATE_REDUCE, NUM_STATE_DOT_REDUCE, NUM_NOISE_REDUCE>{
public:
    StateEstimatorReduced();
    virtual ~StateEstimatorReduced();

    virtual void Initialize_Estimator(_DEF_SENSOR_DATA_);

protected:
    Eigen::Matrix<double, 3,1> omega_;
    Eigen::Matrix<double, 3,1> acc_;

    virtual void _IntegrateState();
    virtual void _GenerateObservationVector(_DEF_SENSOR_DATA_);
    virtual void _GenerateObservationModel();
    virtual void _GeneratePredictionModel();
    virtual void _UpdateStateProvider(_DEF_SENSOR_DATA_);

    bool LED_visibility_[NUM_LED_OBS];
    MoCapReceiver* MoCap_receiver_;
};

#endif
