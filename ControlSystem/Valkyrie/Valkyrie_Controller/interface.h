#ifndef INTERFACE_VALKYRIE
#define INTERFACE_VALKYRIE

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include "StateEstimator.h"

#include <map>
#include <string>

class ValkyrieSystem;

class Interface{
public:
    Interface();
    ~Interface();

public:
    void GetCommand(_DEF_SENSOR_DATA_,
                    std::vector<double> & command);
    void GetReactionForce(std::vector<sejong::Vect3> & reaction_force );
    /* std::map<std::string, int> joint_idx_map; */
    
private:
    bool _Initialization(_DEF_SENSOR_DATA_);

    ValkyrieSystem* valkyrie_sys_;
    
    sejong::Vector virtual_sensor_;
    sejong::Vector torque_command_;
    sejong::Vector sensed_torque_;
    int count_;
    double running_time_;

    sejong::Vector initial_jpos_;
    StateEstimator state_estimator_;
};

#endif
