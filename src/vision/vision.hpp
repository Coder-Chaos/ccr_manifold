#pragma once

#include <stdio.h>
#include "DVPCamera.h"
#include "utility.hpp"
#include "camera.hpp"
#include "common.hpp"
#include "data_structure.hpp"
#include "image_process.hpp"
#include "net.hpp"
#include <queue>
#include "log.h"
#include "uvc.hpp"

namespace quick_inspection
{

// frame info
#define FRAME_WIDTH 1224
#define FRAME_HEIGHT 1024

#define RESIZE_WIDTH 416
#define RESIZE_HEIGHT 416

#define IN_MMP_SIZE FRAME_WIDTH *FRAME_HEIGHT * 3
#define OUT_MMP_SIZE RESIZE_WIDTH *RESIZE_HEIGHT * 3

// camera number
#define CAM_NUM 4

#define QUEUE_LEN 4

    typedef struct
    {
        dvpByte frame_data[4 * 1024 * 1024];
    } frame_4MB;

    class vision
    {

    private:
        // maximum number of do3think cameras in the system
        static const int kMaxCamNum = 4;

        // do3think camera number
        int do3think_cam_cnt_;

        // stop signal
        bool stop_signal_ = false;

        // get frame interval,defualt 40ms, 25ms to take the frame
        int frame_interval_ = 40;

        // stream thread
        std::thread stream_thread_;

        // uvc thread
        std::thread uvc_thread_;

        // udp stream thread
        std::thread udp_transmission_thread_;

        // queue length
        static const int kQueueLength = 4;

        // create frame info queue
        // common::Queue<visual_buffer_info_t> udp_queue_[kMaxCamNum] = {kQueueLength, kQueueLength, kQueueLength, kQueueLength};
        // std::queue<visual_buffer_info_t> udp_queue_[kMaxCamNum];

        // image processer
        image_process image_proc_;

        // uvc cam
        uvc uvc_cam_;

        // udp mutex
        std::mutex udp_mutex_;

    public:
        // init
        int Init(void);

        // Start stream video
        void StartStream(void);

        // stop stream video
        void StopStream(void);

        // Start save picture
        void StartSavePicture(void);

        // stop save picture
        void StopSavePicture(void);

        // start udp transmission
        void StartUDPTransmission(void);
        void StartUDPSend(void);

        // stop udp transmission
        void StopUDPTransmission(void);
        void StopUDPSend(void);

        // start frame compression
        void StartFrameCompression(void);

        // stop frame compression
        void StopFrameCompression(void);

        // start uvc camera
        void StartUVC();

        // stop uvc camera
        void StopUVC();

        vision() {}
        vision(const image_process &&image_proc) : image_proc_(image_proc) {}
        vision(const image_process &&image_proc, const uvc &&uvc_cam) : image_proc_(image_proc), uvc_cam_(uvc_cam)
        {
            uvc_cam_.Init();
        }

        ~vision()
        {
        }

    private:
        // cam start stream
        void GetFrame(void);

        // udp transmission
        void UDPTransmission(void);

        // udp send thread with camera no.
        void UDPTransmission(int i);
    };
} // namespace quick_inspection