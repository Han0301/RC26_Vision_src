#ifndef __LOGGER_CPP_
#define __LOGGER_CPP_


#include "logger.h"


namespace Ten
{
    std::once_flag Ten_logger::logger_flag_;
    std::string Ten_logger::directory_ = std::to_string('.');
    int Ten_logger::flag_ = 0;
}





#endif

