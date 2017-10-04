#ifndef MOCAP_PRECEIVER_H
#define MOCAP_PRECEIVER_H

#include "MoCap_udp.h"

#include <vector>

#include <utils/Sejong_Thread.h>
#include <utils/wrap_eigen.hpp>

#define NUM_LED_OBS (NUM_LED)

class MoCapReceiver: public Sejong_Thread
{
public:
    MoCapReceiver();
    virtual ~MoCapReceiver();
    void getMoCapData(Eigen::Matrix<double, 3*NUM_LED_OBS,1> & MoCapData, bool * LED_visibility);
    
    int count;
    int socketInteger;
    int loopIndex;
    bool MoCap_Visibility_[NUM_LED_OBS];    
protected:
    message mostRecentMessage;


    
    virtual void handleUpdate();
    void _ProcessRawData(Eigen::Matrix<double, 3*NUM_LED_OBS,1> & MoCap_Data);

    void run();
};

#endif
