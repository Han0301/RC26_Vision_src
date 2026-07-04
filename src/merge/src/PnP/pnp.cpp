#ifndef __PNP_CPP_H_
#define __PNP_CPP_H_
#include "./pnp_main.h"
#include "./../superstratum/super.h"


namespace Ten
{

    void test_pnp()
    {
        urcu_memb_register_thread();

        Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
        Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        float arr[1] = {0};
        camera.reset_camera_depth(640, 480, 30);
        rs2_intrinsics color_intr = camera.get_color_intrinsics();
        Ten::KFS::kfsLocator pnp_hander(color_intr);

        while (Ten::_CAMERA_KFS_FLAG_.read_flag())
        {
            Ten::camera_frame frame = camera.camera_read_depth();
            arr[0] = pnp_hander.processOneFrame(frame.bgr_image,frame.depth_image);
            serial.serial_send(arr, 7, sizeof(arr)); 
            std::cout << "bias: " << arr[0] << std::endl;
        }

        urcu_memb_unregister_thread();
    }

}

#endif
