#include "OC3System.hpp"
#include "ControlSet/Basic_ctrl.hpp"

OC3System::OC3System(){
  printf("System\n");
  controller_ = new Basic_ctrl();

  // Add more controllers here.
  // Uncomment the line with your own controller to use it
  // and comment out the other controller(s) like I did above.

  printf("[OC3 System] Constructed \n");
}

OC3System::~OC3System(){
    printf("[OC3 System] Destruction \n");
}

void OC3System::getTorqueInput(sejong::Vector & torque_command){
    controller_->ComputeTorqueCommand(torque_command);
}
void OC3System::Initialization(){
  controller_->Initialization();
}
