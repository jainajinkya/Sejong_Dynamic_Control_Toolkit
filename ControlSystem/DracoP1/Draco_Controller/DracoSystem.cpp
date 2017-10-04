#include "DracoSystem.hpp"
#include "ControlSet/WBDC_Ctrl.hpp"

DracoSystem::DracoSystem(){
  printf("System\n");
  controller_ = new WBDC_Ctrl();

  // Add more controllers here.
  // Uncomment the line with your own controller to use it
  // and comment out the other controller(s) like I did above.

  printf("[Draco System] Constructed \n");
}

DracoSystem::~DracoSystem(){
    printf("[Draco System] Destruction \n");
}

void DracoSystem::getTorqueInput(sejong::Vector & torque_command){
    controller_->ComputeTorqueCommand(torque_command);
}
