#ifndef DATA_SAVE
#define DATA_SAVE

#include <Utils/Sejong_Thread.hpp>
#include "data_protocol.h"

class DataSave: public Sejong_Thread{
public:
    DataSave();
    virtual ~DataSave();
    virtual void run (void );

private:
    void _ShowDataSetup(const DATA_Protocol::DATA_SETUP & data_setup);
    int socket1_;
    int socket2_;
};

#endif
