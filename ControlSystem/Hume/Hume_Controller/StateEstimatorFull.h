#ifndef STATE_ESTIMATOR_FULL 
#define STATE_ESTIMATOR_FULL 

#include "MoCap_Receiver.h"
#include "StateEstimator.h"

// (Joint Position) +
// (Joint Torque) +
// (IMU Ang Vel) +
// (IMU Acc) + 
// (LED Position)
#define NUM_OBS_FULL (NUM_ACT_JOINT + NUM_ACT_JOINT + 3 + 3+ 3*NUM_LED_OBS)
// (Q, Qdot, Torque, Ext_Force, 1)
#define NUM_STATE_DOT_FULL  (NUM_QDOT + NUM_QDOT + NUM_ACT_JOINT + 1)
#define NUM_STATE_FULL  (NUM_Q + NUM_QDOT + NUM_ACT_JOINT + 1)

#define NUM_NOISE_FULL NUM_STATE_DOT_FULL
class HumeModel;
class StateProvider;
class Integrator;

using namespace sejong;

class StateEstimatorFull : public StateEstimator<NUM_OBS_FULL, NUM_STATE_FULL, NUM_STATE_DOT_FULL, NUM_NOISE_FULL>{
public:
    StateEstimatorFull();
    virtual ~StateEstimatorFull();

    virtual void Initialize_Estimator(_DEF_SENSOR_DATA_);

protected:
    virtual void _IntegrateState();
    virtual void _GenerateObservationVector(_DEF_SENSOR_DATA_);
    virtual void _GenerateObservationModel();
    virtual void _GeneratePredictionModel();
    virtual void _UpdateStateProvider(_DEF_SENSOR_DATA_);

    bool LED_visibility_[NUM_LED_OBS];

    sejong::Vector jvel_save_;
    sejong::Vector torque_est_save_;

    MoCapReceiver* MoCap_receiver_;

    bool impulse_;

    bool pre_lcontact_;
    bool pre_rcontact_;
};

#endif
