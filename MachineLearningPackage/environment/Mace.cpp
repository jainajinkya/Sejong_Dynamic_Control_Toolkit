#include "Mace.h"
#include <Utils/utilities.h>
#include <iostream>
#include <algorithm>



Env_Mace::Env_Mace():Environment(), life_cost_(-5.){
  // _Small_Mace_Setup();
  _Midium_Mace_Setup();
}
Env_Mace::~Env_Mace(){}

void Env_Mace::_Midium_Mace_Setup(){
  obstacle_list_.resize(20);
  // Left Wall
  for (int i(0); i<6; ++i){
    obstacle_list_[i] = sejong::Vector::Zero(2);

    obstacle_list_[i][0] = i;
    obstacle_list_[i][1] = -1.;
  }
  // Bottom 
  obstacle_list_[6] = sejong::Vector::Zero(2);
  obstacle_list_[6][0] = 6;
  for (int i(0); i<3; ++i){
    obstacle_list_[7+i] = sejong::Vector::Zero(2);

    obstacle_list_[7+i][0] = 6;
    obstacle_list_[7+i][1] = i + 2;
  }

  // Right
  obstacle_list_[10] = sejong::Vector::Zero(2);
  obstacle_list_[10][0] = 5;
  obstacle_list_[10][1] = 5;

  for (int i(0); i<4; ++i){
    obstacle_list_[11+i] = sejong::Vector::Zero(2);

    obstacle_list_[11+i][0] = 3-i;
    obstacle_list_[11+i][1] = 5;
  }

  // Upper
  for (int i(0); i<3; ++i){
    obstacle_list_[15+i] = sejong::Vector::Zero(2);

    obstacle_list_[15+i][0] = -1;
    obstacle_list_[15+i][1] = 3-i;
  }
  // Hole 1
  obstacle_list_[18] = sejong::Vector::Zero(2);
  obstacle_list_[18][0] = 1;
  obstacle_list_[18][1] = 1;

  // Hole 2
  obstacle_list_[19] = sejong::Vector::Zero(2);
  obstacle_list_[19][0] = 2;
  obstacle_list_[19][1] = 4;

  // Hole 3
  // obstacle_list_[20] = sejong::Vector::Zero(2);
  // obstacle_list_[20][0] = 3;
  // obstacle_list_[20][1] = 3;


  std::vector<int> location(2);
  location[0] = -1;  location[1] = 0;
  terminal_list_.insert(std::make_pair(location, 100.0));

  location[0] = 6;  location[1] = 1;
  terminal_list_.insert(std::make_pair(location, 10.0));

  location[0] = 4;  location[1] = 5;
  terminal_list_.insert(std::make_pair(location, 5.0));

  location[0] = -1;  location[1] = 4;
  terminal_list_.insert(std::make_pair(location, -10.0));
}


void Env_Mace::_Small_Mace_Setup(){
  obstacle_list_.resize(4);

  obstacle_list_[0] = sejong::Vector::Zero(2);
  obstacle_list_[0][0] = -1.;
  
  obstacle_list_[1] = sejong::Vector::Zero(2);
  obstacle_list_[1][1] = -1.;

  obstacle_list_[2] = sejong::Vector::Zero(2);
  obstacle_list_[2][0] = 1.;

  obstacle_list_[3] = sejong::Vector::Zero(2);
  obstacle_list_[3][0] = 1.;
  obstacle_list_[3][1] = 1.;

  std::vector<int> location(2);
  location[0] = -1;  location[1] = 1;
  terminal_list_.insert(std::make_pair(location, 10.0));

  location[0] = 0;  location[1] = 2;
  terminal_list_.insert(std::make_pair(location, -5.0));
}

bool Env_Mace::Transition(const sejong::Vector & state,
                          const sejong::Vector & action,
                          double & reward,
                          sejong::Vector & nx_state){
  reward = life_cost_;
  nx_state = state;
  nx_state += action;
  // sejong::pretty_print(state, std::cout, "curr_state");
  // sejong::pretty_print(action, std::cout, "action");

  // Obstacle Check
  std::vector<sejong::Vector>::iterator iter;
  if( (iter = std::find(obstacle_list_.begin(), obstacle_list_.end(), nx_state) ) != obstacle_list_.end()){

    // sejong::pretty_print((*iter), std::cout, "obstacle");
    nx_state = state;
  }
  else {
    // Terminal Check
    std::map< std::vector<int>, double >::iterator iter_map;
    std::vector<int> nx_state_vec(DIM_STATE);
    for(int i(0); i<DIM_STATE; ++i)  nx_state_vec[i] = nx_state[i];

    if( (iter_map = terminal_list_.find(nx_state_vec) ) != terminal_list_.end()){
      // printf("terminal value: %f \n", iter_map->second);
      reward = iter_map->second;
      return true;
    }
  }

  return false;
}

void Env_Mace::getTerminalPlace(std::vector< std::vector<int> > & terminal_loc_list){
  terminal_loc_list.clear();

  std::map< std::vector<int>, double >::iterator iter_map;

  for(iter_map = terminal_list_.begin(); iter_map != terminal_list_.end(); ++ iter_map){
    terminal_loc_list.push_back(iter_map->first);
  }
}
