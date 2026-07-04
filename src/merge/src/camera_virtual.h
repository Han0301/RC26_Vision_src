#ifndef __CAMERA_VIRTUAL_H_
#define __CAMERA_VIRTUAL_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include "threadpool.h"

namespace Ten
{

    class camera_virtual
    {
    public:
        virtual cv::Mat camera_read() = 0;

    };

}

#endif
