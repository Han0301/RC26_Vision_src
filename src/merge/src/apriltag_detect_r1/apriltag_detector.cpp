#ifndef __APRILTAG_DETECTOR_CPP_
#define __APRILTAG_DETECTOR_CPP_
#include "apriltag_detector.h"
#include "apriltag_detector_area.h"

namespace Ten::apriltag_detect
{
    inline int is_r1(Ten::camera_virtual& camera)
    {
        static Ten::apriltag_detect::AprilTagDetector det;
        std::cout << "set_r1ing" << std::endl;
        det.detR1(camera);
        int result = det.get_is_r1();
        std::cout << "end_r1ing" << std::endl;
        return result;
    }

    inline int is_r1_area(Ten::camera_virtual& camera)
    {
        static Ten::apriltag_detect::AprilTagDetector_AREA det;
        std::cout << "set_r1ing" << std::endl;
        det.detR1(camera);
        int result = det.get_is_r1();
        std::cout << "end_r1ing" << std::endl;
        return result;
    }
}
#endif