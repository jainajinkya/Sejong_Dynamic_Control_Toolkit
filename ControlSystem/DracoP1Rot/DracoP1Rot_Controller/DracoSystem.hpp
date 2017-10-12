#ifndef DRACO_SYSTEM
#define DRACO_SYSTEM

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class DracoController;

class DracoSystem {
public:
    DracoSystem();
    ~DracoSystem();

    void getTorqueInput(sejong::Vector & torque_command);

    DracoController* controller_;
};


#endif
