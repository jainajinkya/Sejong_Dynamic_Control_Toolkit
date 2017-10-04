#include "ValkyrieSystem.h"
#include "ControlSet/Test_Ctrl.h"

ValkyrieSystem::ValkyrieSystem(){
  controller_ = new Test_Ctrl();

  // Add more controllers here.
  // Uncomment the line with your own controller to use it
  // and comment out the other controller(s) like I did above.

  printf("[Valkyrie System] Constructed \n");
}

ValkyrieSystem::~ValkyrieSystem(){
    printf("[Valkyrie System] Destruction \n");
}

void ValkyrieSystem::getTorqueInput(sejong::Vector & torque_command){
    controller_->ComputeTorqueCommand(torque_command);
}
