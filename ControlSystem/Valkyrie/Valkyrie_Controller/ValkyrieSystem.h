#ifndef VALKYRIE_SYSTEM
#define VALKYRIE_SYSTEM

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class ValkyrieController;

class ValkyrieSystem {
public:
    ValkyrieSystem();
    ~ValkyrieSystem();

    void getTorqueInput(sejong::Vector & torque_command);

    ValkyrieController* controller_;
};


#endif
