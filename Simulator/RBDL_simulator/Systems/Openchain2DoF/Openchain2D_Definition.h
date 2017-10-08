#ifndef DEFINITION_OPENCHAIN_2D
#define DEFINITION_OPENCHAIN_2D

#include <stdio.h>
#include <string>

#define NUM_LINK 2
#define NUM_LINES 2
#define NUM_VALUE_PYTHON NUM_LINES * 4 + 1

enum SJLinkID{
    m1,
    m2,
    m3,
    NUM_LINKS
};

inline SJLinkID FindLinkID(std::string const & str_id){
    if(str_id == "m1")  return m1;
    if(str_id == "m2")  return m2;
    if(str_id == "m3")  return m3;
}

#endif
