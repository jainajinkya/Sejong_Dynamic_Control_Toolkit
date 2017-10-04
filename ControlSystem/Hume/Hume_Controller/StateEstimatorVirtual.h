#ifndef STATE_ESTIMATOR_VIRTUAL
#define STATE_ESTIMATOR_VIRTUAL

#include "StateEstimator.h"
#include "MoCap_Receiver.h"

#define NUM_LED_REDUCE 5

// (Ang_Vel, Lin_Acc, LED Position)
#define NUM_OBS_VIRTUAL (3 + 3 + 3*NUM_LED_REDUCE)
// (X, Xdot, F_ext)
#define NUM_STATE_DOT_VIRTUAL  (6 + 6 + 6)
#define NUM_STATE_VIRTUAL  (7 + 6 + 6)

#define NUM_NOISE_VIRTUAL NUM_STATE_DOT_VIRTUAL

class MoCapReceiver;

using namespace sejong;

class StateEstimatorVirtual : public StateEstimator<NUM_OBS_VIRTUAL, NUM_STATE_VIRTUAL, NUM_STATE_DOT_VIRTUAL, NUM_NOISE_VIRTUAL>{
public:
    StateEstimatorVirtual();
    virtual ~StateEstimatorVirtual();

    virtual void Initialize_Estimator(_DEF_SENSOR_DATA_);

protected:
    sejong::Matrix J_;
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
