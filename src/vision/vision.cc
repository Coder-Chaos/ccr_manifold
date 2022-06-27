#include "vision.hpp"
#include "udmabuf.hpp"
#include "hw_resize.hpp"
#include <array>
#include <atomic>

#define LOCAL_DEBUG 0
#define UDP_DECOUPLE 0
#define SAVE_PIC 1
#define USE_HW_ACCELERATOR 1
#define DEBUG

// one frame delay time ms
#define DELAY 0
#define TIME_DELAY_MS(N) (N * 1000)

namespace quick_inspection
{
    // static var
    static visual_buffer_info_t visual_buf[CAM_NUM];
    // create frame data queue statically
    static common::Queue<visual_buffer_info_t> frame_queue[CAM_NUM] = {QUEUE_LEN, QUEUE_LEN, QUEUE_LEN, QUEUE_LEN};

    static visual_fragment_t visual_fragment_array[CAM_NUM][MAX_VISUAL_FRAME_FRAGMENT_SIZE];

    static common::udmabuf udmabuf_in[CAM_NUM] = {{"udmabuf0", IN_MMP_SIZE}, {"udmabuf1", IN_MMP_SIZE}, {"udmabuf2", IN_MMP_SIZE}, {"udmabuf3", IN_MMP_SIZE}};
    static common::udmabuf udmabuf_out[CAM_NUM] = {{"udmabuf4", OUT_MMP_SIZE}, {"udmabuf5", OUT_MMP_SIZE}, {"udmabuf6", OUT_MMP_SIZE}, {"udmabuf7", OUT_MMP_SIZE}};

    static hw_resize resize_accel[CAM_NUM] = {{0, FRAME_WIDTH, FRAME_HEIGHT, RESIZE_HEIGHT, RESIZE_WIDTH, (u64)udmabuf_in[0].GetPhyAddr(), (u64)udmabuf_out[0].GetPhyAddr()},
                                              {1, FRAME_WIDTH, FRAME_HEIGHT, RESIZE_HEIGHT, RESIZE_WIDTH, (u64)udmabuf_in[1].GetPhyAddr(), (u64)udmabuf_out[1].GetPhyAddr()},
                                              {2, FRAME_WIDTH, FRAME_HEIGHT, RESIZE_HEIGHT, RESIZE_WIDTH, (u64)udmabuf_in[2].GetPhyAddr(), (u64)udmabuf_out[2].GetPhyAddr()},
                                              {3, FRAME_WIDTH, FRAME_HEIGHT, RESIZE_HEIGHT, RESIZE_WIDTH, (u64)udmabuf_in[3].GetPhyAddr(), (u64)udmabuf_out[3].GetPhyAddr()}};

    // init vision system
    int vision::Init(void)
    {
        int state;
        // init camera
        state = Camera::init(do3think_cam_cnt_);
        if (state != 0)
        {
            // TRACE("Error at Camera::init");
            log_error("\r do3think cameras init error.\n");
            exit(-1);
        }
        else
            return 0;
    }

    static uint8_t pBuffer[4][16 * 1024 * 1024];
    void vision::GetFrame(void)
    {
        int i = 0;
        int ret;

        // get frame time
        utility::GetTime get_frame_time[do3think_cam_cnt_];
        // save png time
        utility::GetTime save_pic_time[do3think_cam_cnt_];

        // compress time
        utility::GetTime compress_time[do3think_cam_cnt_];

        // timer
        long long cam_timer[do3think_cam_cnt_] = {0, 0, 0, 0};

        // compose visual frame fragment time
        utility::GetTime compose_time;
        // udp send time
        utility::GetTime udp_time[do3think_cam_cnt_];

        visual_fragment_info_t visual_fragment_info[do3think_cam_cnt_];

        long total_time[do3think_cam_cnt_];
        long sum = 0;

        dvpFrame frame_info_buf[do3think_cam_cnt_];
        long long cnt[do3think_cam_cnt_] = {0};

        while (stop_signal_ == false)
        {
#pragma omp parallel for
            for (i = 0; i < do3think_cam_cnt_; i++)
            {
#if 0
                // // delay wait time capture time reached,ms.
                // while (frame_time[i].Process() < 100 && stop_signal_ == false)
                // {
                //     usleep(1000);
                // }

                // // refresh time
                // frame_time[i].Start();

                // // check queue state
                // while (frame_queue[i].QueueISFull() && stop_signal_ == false)
                // {
                //     // printf("\rudp queue[%d] full !\n", i);
                //     // exit(-1);
                //     // continue;
                //     usleep(1000);
                // }
#endif
                // get frame
                printf("Camera::TakePicture %d\n", i);
                get_frame_time[i].Start();
                Camera::TakePicture(i, frame_info_buf + i, pBuffer[i], 100);
                get_frame_time[i].Stop();
                printf("Camera::TakePicture %d\n", i);
                log_debug("camera[%d] get one frame time:%ld ms", i, get_frame_time[i].Process());

#if SAVE_PIC
                // save  picture
                save_pic_time[i].Start();
                Camera::SavePicture(i, frame_info_buf + i, pBuffer[i]);
                save_pic_time[i].Stop();
                log_debug("camera[%d] save one picture time:%ld ms", i, save_pic_time[i].Process());
#endif

                // compress frame
                compress_time[i].Start();

#if USE_HW_ACCELERATOR

                // copy data to accelerator
                memcpy(udmabuf_in[i].GetVirtualAddr(), (char *)pBuffer[i], IN_MMP_SIZE);
                // set information
                visual_buf[i].position = i;
                visual_buf[i].time_stamp = cnt[i];
                visual_buf[i].data_length = OUT_MMP_SIZE + sizeof(photo_info_t);

                photo_info_t *visual_info = (photo_info_t *)(visual_buf[i].data);
                visual_info->position = i;
                visual_info->group = 0;
                visual_info->sequence = 0;
                visual_info->time_stamp = cnt[i];
                visual_info->width = RESIZE_WIDTH;
                visual_info->height = RESIZE_HEIGHT;
                visual_info->bytes_per_pixel = 3;

                // start accelerator
                resize_accel[i].Start();

                // copy data back from accelerator
                memcpy(visual_buf[i].data + sizeof(photo_info_t), udmabuf_out[i].GetVirtualAddr(), OUT_MMP_SIZE);
#else
                image_proc_.compose_visual_buffer_info(i, (dvpByte *)pBuffer[i], FRAME_WIDTH, FRAME_HEIGHT, cnt[i], visual_buf[i]);
#endif
                compress_time[i].Stop();
                log_debug("camera[%d] compress one frame time:%ld ms", i, compress_time[i].Process());
            }

            // enqueue to compress
            // frame_queue[i].EnQueue(visual_buf[i]);

            log_debug("----------------------------------------------");

#if UDP_DECOUPLE
            compose_time.Start();
            // compose visual frame fragment
#pragma omp parallel for
            for (i = 0; i < do3think_cam_cnt_; i++)
            {
                visual_fragment_info[i] = image_proc_.compose_visual_frame_fragment(visual_buf[i], visual_fragment_array[i]);
            }
            compose_time.Stop();
            log_debug("cameras compose visual frame fragment time:%ld ms", i, compose_time.Process());

            // udp send
            for (i = 0; i < do3think_cam_cnt_; i++)
            {
                udp_time[i].Start();
                image_proc_.send_visual(visual_fragment_array[i], visual_fragment_info[i]);
                udp_time[i].Stop();
                log_debug("camera[%d] udp send one frame time:%ld ms", i, udp_time[i].Process());
            }

#else
            // udp send, parallel?
            for (i = 0; i < do3think_cam_cnt_; i++)
            {
                /* code */
                udp_time[i].Start();
                ret = image_proc_.send_visual(visual_buf[i]);
                udp_time[i].Stop();
                log_debug("camera[%d] udp send one frame time:%ld ms", i, udp_time[i].Process());
            }
#endif
            // delay
            usleep(TIME_DELAY_MS(DELAY));
            log_debug("----------------------------------------------");
            // one frame total time
            for (i = 0; i < do3think_cam_cnt_; i++)
            {

                /* code */
#if UDP_DECOUPLE
                total_time[i] = (int)(get_frame_time[i].Process() + save_pic_time[i].Process() + compress_time[i].Process() + udp_time[i].Process() + compose_time.Process() + DELAY);

#else
                total_time[i] = (int)(get_frame_time[i].Process() + save_pic_time[i].Process() + compress_time[i].Process() + udp_time[i].Process() + DELAY);

#endif
                log_info("camera[%d] one frame total time:%d ms", i, total_time[i]);
                sum += total_time[i];
            }
            log_info("average one frame total time:%d ms", sum / do3think_cam_cnt_);
            sum = 0;
            log_info("----------------------------------------------");
            log_info(" ");
            cnt[i]++;
        }
    }

    // start stream
    void vision::StartStream()
    {
        // create work thread, use lambda
        printf("\rcamera start streaming.\n");
        stream_thread_ = std::thread([&]() { GetFrame(); });
    }

    // start uvc
    void vision::StartUVC()
    {
        visual_buffer_info_t visual_buf;
        // create uvc thread, use lambda
        printf("\rstart uvc camera.\n");
        uvc_thread_ = std::thread([&]() {
            while (stop_signal_ == false)
            {
                uvc_cam_.Stream();
                /* udp send */
#if 1
                photo_info_t *photo_info = (photo_info_t *)visual_buf.data;

                photo_info->position = 4;
                photo_info->group = 0;
                photo_info->sequence = 0;
                photo_info->time_stamp = uvc_cam_.frame_cnt_;
                photo_info->width = uvc_cam_.img_width_;
                photo_info->height = uvc_cam_.img_height_;
                photo_info->bytes_per_pixel = 3;
                photo_info->reserved = 0;

                memcpy((char *)visual_buf.data + sizeof(photo_info_t), (char *)uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].start, uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length);

                visual_buf.data_length = uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length + sizeof(photo_info_t);
                visual_buf.position = 4;
                visual_buf.time_stamp = uvc_cam_.frame_cnt_;

                // add mutex ！
                std::lock_guard<std::mutex> guard(udp_mutex_);

                // udp send
                image_proc_.send_visual(visual_buf);
                uvc_cam_.QBuf();
#endif
                usleep(100000);
            }
        });
    }

    // stop uvc
    void vision::StopUVC()
    {
        stop_signal_ = true;
        uvc_thread_.join();

        printf("\rstop uvc camera.\n");
        // stop uvc
        uvc_cam_.DeInit();
    }

    // stop stream
    void vision::StopStream()
    {
        stop_signal_ = true;
        stream_thread_.join();

        printf("\rcamera stop streaming.\n");
    }

    void
    vision::UDPTransmission(void)
    {
        int i = 0, ret = 0;
        utility::GetTime udp_send_time;

        while (stop_signal_ == false)
        {
            for (i = 0; i < do3think_cam_cnt_; i++)
            {
                // wait until data arrive.
                if (frame_queue[i].QueueISEmpty())
                {
                    printf("\rudp queue[%d] empty !\n", i);
                    continue;
                }
                udp_send_time.Start();
                // get frame from queue and send
                ret = image_proc_.send_visual(*(visual_buffer_info_t *)frame_queue[i].DeQueue());
                udp_send_time.Stop();
                if (ret == 0)
                {
                    printf("\r-------------------------------------------------------\n");
                    printf("\rcamera[%d] udp send visual time:%ld ms\n", i, udp_send_time.Process());
                }
            }
            // delay 1ms
            usleep(1000);
        }
    }

    // start udp transmission
    void vision::StartUDPTransmission(void)
    {
        printf("\rstart udp transmission.\n");
        // create work thread, use lambda
        udp_transmission_thread_ = std::thread([&]() { UDPTransmission(); });
        // udp_transmission_thread_ = std::thread(this->UDPTransmission());
    }

    // stop udp transmission
    void vision::StopUDPTransmission(void)
    {
        stop_signal_ = true;
        udp_transmission_thread_.join();

        printf("\rstop udp transmmsion.\n");
    }

} // namespace quick_inspection