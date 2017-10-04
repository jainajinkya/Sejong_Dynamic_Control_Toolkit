#include "Task.h"
#include <stdio.h>
#include <iostream>
#include <Utils/utilities.h>

Task::Task(): b_settask_(false),
              num_control_DOF_(0) {
}

void Task::_PostProcess_Task(){
    b_settask_ = false;
}
void Task::SetTask(const sejong::Vector & des, const sejong::Vector & vel_des,
                   const sejong::Vector & act, const sejong::Vector & vel_act) {
    des_ = des;
    vel_des_ = vel_des;
    act_ = act;
    vel_act_ = vel_act;
    feedforward_.setZero();
    b_settask_ = true;
}

void Task::SetTask(const sejong::Vector & des, const sejong::Vector & vel_des,
                   const sejong::Vector & act, const sejong::Vector & vel_act,
                   const sejong::Vector & feedforward){
    des_ = des;
    vel_des_ = vel_des;
    act_ = act;
    vel_act_ = vel_act;
    feedforward_ = feedforward;
    b_settask_ = true;
}

double Task::crop_value(double value, double min, double max, std::string source){
    if(value> max){
        // printf("%s: Hit The MAX\t", source.c_str());
        // printf("Original Value: %f\n", value);
        return max;
    }
    else if(value < min){
        // printf("%s: Hit The MIN\t", source.c_str());
        // printf("Original Value: %f\n", value);
        return min;
    }
    else{
        return value;
    }
}

void Task::_debug_show_task_setup(){
    printf("\n");
    sejong::pretty_print(des_, std::cout, "Des", "");
    sejong::pretty_print(act_, std::cout, "Act", "");
    printf("\n");
    sejong::pretty_print(vel_des_, std::cout, "Des Vel", "");
    sejong::pretty_print(vel_act_, std::cout, "Act Vel", "");
    printf("\n \n");
}
void Task::_debug_show_task_setup(const sejong::Vector & input){
    printf("\n");
    sejong::pretty_print(des_, std::cout, "Des", "");
    sejong::pretty_print(act_, std::cout, "Act", "");
    printf("\n");
    sejong::pretty_print(vel_des_, std::cout, "Des Vel", "");
    sejong::pretty_print(vel_act_, std::cout, "Act Vel", "");
    printf("\n \n");
    sejong::pretty_print(input, std::cout, "Input", "");
    printf("\n \n");

}
