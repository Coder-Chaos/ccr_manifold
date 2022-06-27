#include "vision.hpp"
#include "log.h"
#include "uvc.hpp"
// #define MULTI_THREAD

// #define SAVE_PIC

int main(int argc, char **argv)
{
    common::GlobalInit(&argc, &argv);

    // set log level
    log_set_level(1);

    // init udp client
    common::Net udp_client{common::protocal_type::UDP, common::type::CLIENT, "10.60.2.97", 7000};

    // init image processor
    quick_inspection::image_process image_processor{std::move(udp_client)};

    // uvc camera
    uvc uvc_cam{640, 480, "/dev/video0"};

    quick_inspection::vision vision_inspect{std::move(image_processor), std::move(uvc_cam)};

    // init vision system
    vision_inspect.Init();

    // start udp transmission
    // vision_inspect.StartUDPTransmission();

    // start do3think camera stream
    vision_inspect.StartStream();

    // start uvc camera
    vision_inspect.StartUVC();

    // press enter to stop
    PAUSE();

    vision_inspect.StopStream();

    // stop uvc camera
    vision_inspect.StopUVC();

    // vision_inspect.StopUDPTransmission();

    // stop udp transmission

    Camera::fini();

    return 0;
}