#include "OC2System.hpp"
//#include "ControlSet/Basic_ctrl.hpp"
#include "ControlSet/No_ctrl.hpp"

OC2System::OC2System(){
  printf("System\n");
  //controller_ = new Basic_ctrl();
  controller_ = new No_ctrl();  
  // Add more controllers here.
  // Uncomment the line with your own controller to use it
  // and comment out the other controller(s) like I did above.

  printf("[OC2 System] Constructed \n");
}

OC2System::~OC2System(){
    printf("[OC2 System] Destruction \n");
}

void OC2System::getTorqueInput(sejong::Vector & torque_command){
    controller_->ComputeTorqueCommand(torque_command);
}
void OC2System::Initialization(){
  controller_->Initialization();
}
