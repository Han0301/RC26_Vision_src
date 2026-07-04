#include "super.h"

namespace Ten
{

    void nine_vision()
    {
        Ten::mouse m;
        Ten::segmentation s;
        Ten::detector d;
        m.calibration();
        Ten::Ten_usb_cam& usbcam = Ten::Ten_usb_cam::GetInstance(Ten::_usb_device_num3_,640,480,30);
        while(Ten::_TREADPOOL_FLAG_.read_flag())
        {
            cv::Mat img = usbcam.camera_read();
            if(img.empty())
            {
                cv::waitKey(1000 / 30);
                continue;
            }
            std::vector<cv::Mat> grids = s.getroi(img);
            cv::Mat roi_debug = s.debugAssemble3x3Grid(grids);
            cv::imshow("global", img);
            cv::imshow("roi_debug", roi_debug);
            int key = cv::waitKey(1000 / 30);
            if(key == 'q')
            {
                break;
            }
            else if(key == 's')
            {
                d.saveimg(grids);
            }
        }

    }

}