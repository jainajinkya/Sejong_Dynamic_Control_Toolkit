#include "Walker2D_System.hpp"
#include "ControlSet/Basic_ctrl.hpp"

Walker2D_System::Walker2D_System(){
  printf("System\n");
  controller_ = new Basic_ctrl();

  // Add more controllers here.
  // Uncomment the line with your own controller to use it
  // and comment out the other controller(s) like I did above.

  printf("[Walker2D_ System] Constructed \n");
}

Walker2D_System::~Walker2D_System(){
    printf("[Walker2D System] Destruction \n");
}

void Walker2D_System::getTorqueInput(sejong::Vector & torque_command){
    controller_->ComputeTorqueCommand(torque_command);
}
void Walker2D_System::Initialization(){
  controller_->Initialization();
}
