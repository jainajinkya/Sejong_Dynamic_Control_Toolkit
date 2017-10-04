#include "MoCap_Receiver.h"
#include <stdio.h>
#include <utils/comm_udp.h>

MoCapReceiver::MoCapReceiver():Sejong_Thread(),
                               count(0),socketInteger(0){
    
}
MoCapReceiver::~MoCapReceiver(){
}

void MoCapReceiver::handleUpdate()
{
    Int32BitField* bitfield = (Int32BitField*) &mostRecentMessage.validBits;
    message* msg = &mostRecentMessage;
    count++;
    // if (count % 1500 == 0)
    //     fprintf(stderr,
    //             "Received Data: {%d,%d,%d,%d} {%d,%d,%d,%d} {%d,%d,%d} \n\t(%f %f %f)(%f %f %f)(%f %f %f)\n\t(%f %f %f)(%f %f %f)(%f %f %f)(%f %f %f)\n",
    //             bitfield->b0, bitfield->b1, bitfield->b2, bitfield->b3, bitfield->b4, bitfield->b5, bitfield->b6,
    //             bitfield->b7, bitfield->b8, bitfield->b9, bitfield->b10,
    //             msg->x[0], msg->y[0], msg->z[0], msg->x[1], msg->y[1], msg->z[1], msg->x[2], msg->y[2],
    //             msg->z[2], msg->x[3], msg->y[3], msg->z[3], msg->x[4], msg->y[4], msg->z[4], msg->x[5],
    //             msg->y[5], msg->z[5], msg->x[6], msg->y[6], msg->z[6]);
}

void MoCapReceiver::run() {

    while(true){
        COMM::receive_data(socketInteger, POS_PORT, &mostRecentMessage, sizeof(message), IP_ADDR_MYSELF);
        // if (mostRecentMessage.index != (loopIndex + 1))
        //     fprintf(stderr, "Missing %d packets (%08x->%08x)\n", mostRecentMessage.index - loopIndex, mostRecentMessage.index, loopIndex);
        
        loopIndex = mostRecentMessage.index;
        handleUpdate();
    }
}

void MoCapReceiver::_ProcessRawData(Eigen::Matrix<double, 3*NUM_LED_OBS,1> & MoCap_Data){
    for(int i(0); i<NUM_LED_OBS; ++i){
        MoCap_Data[3*i] = -mostRecentMessage.z[i] * 0.001;
        MoCap_Data[3*i+1] = -mostRecentMessage.x[i] * 0.001;
        MoCap_Data[3*i+2] = mostRecentMessage.y[i] * 0.001;
    }

    Int32BitField* bitfield = (Int32BitField*) &mostRecentMessage.validBits;

    // Body LED
    MoCap_Visibility_[0] = (bool)bitfield->b0;
    MoCap_Visibility_[1] = (bool)bitfield->b1;
    MoCap_Visibility_[2] = (bool)bitfield->b2;
    MoCap_Visibility_[3] = (bool)bitfield->b3;

    // Leg 1
    MoCap_Visibility_[4] = (bool)bitfield->b4;
    MoCap_Visibility_[5] = (bool)bitfield->b5;
    MoCap_Visibility_[6] = (bool)bitfield->b6;

    // Leg 2
    MoCap_Visibility_[7] = (bool)bitfield->b7;
    MoCap_Visibility_[8] = (bool)bitfield->b8;
}

void MoCapReceiver::getMoCapData(Eigen::Matrix<double, 3*NUM_LED_OBS,1> & MoCapData, bool * LED_visibility){
    _ProcessRawData(MoCapData);
    for (int i(0); i<NUM_LED_OBS; ++i){
        LED_visibility[i] = MoCap_Visibility_[i];
    }
}
