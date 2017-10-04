#include "DataSave.h"

#include <iostream>
#include <utils/comm_udp.h>
#include <utils/utilities.h>
#include <utils/DataManager.h>

DataSave::DataSave():Sejong_Thread(), socket1_(0), socket2_(0){}
DataSave::~DataSave(){}

void DataSave::run(void ) {
    DATA_Protocol::DATA_SETUP data_setup;


    COMM::receive_data(socket1_, PORT_DATA_SETUP, &data_setup, sizeof(DATA_Protocol::DATA_SETUP), IP_ADDR);
    _ShowDataSetup(data_setup);
    
    double* data = new double[data_setup.tot_num_array_data];
    long long iter(0);
    while (true){
        COMM::receive_data(socket2_, PORT_DATA_RECEIVE, data, data_setup.tot_num_array_data*sizeof(double), IP_ADDR);

        int st_idx(0);
        int display_freq(30);
        for (int i(0); i < data_setup.num_data; ++i){
            if(iter % display_freq == 0){
                printf("%s : ", data_setup.data_name[i]);
                for(int j(0); j < data_setup.num_array_data[i]; ++j){
                    std::cout<< data[st_idx + j]<<", ";
                }
                printf("\n");
            }
            sejong::saveVector(&data[st_idx],
                               data_setup.data_name[i],
                               data_setup.num_array_data[i]);
            st_idx += data_setup.num_array_data[i];
        }
        ++iter;
    }

    delete [] data;
}


void DataSave::_ShowDataSetup(const DATA_Protocol::DATA_SETUP & data_setup){
    printf("Number of Data: %i \n", data_setup.num_data);
    printf("Total number of values: %i \n", data_setup.tot_num_array_data);
    for(int i(0); i<data_setup.num_data; ++i){
        printf("%i th -  data name: %s, number of value: %i \n", i, data_setup.data_name[i], data_setup.num_array_data[i]);
    }
}
