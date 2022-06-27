#pragma once
// #include "common.hpp"
#include "net.hpp"



#define VISUAL_BUFFER_BLOCK_SIZE (4 * 1024 * 1024)

#define VISUAL_FRAME_FRAGMENT_DATA_SIZE (4 * 1024)
#define VISUAL_FRAME_FRAGMENT_BUFFER_SIZE (8 * 1024)

#define MAX_VISUAL_FRAME_FRAGMENT_SIZE 100

// 长、宽都右移
#define COMPRESS_RATIO 2

    struct visual_buffer_info_t
    {
        uint64_t state;
        uint64_t position;
        uint64_t time_stamp;
        uint64_t data_length;
        uint8_t data[VISUAL_BUFFER_BLOCK_SIZE];
    };

    struct photo_info_t
    {
        uint64_t position;
        uint64_t group;
        uint64_t sequence;
        uint64_t time_stamp;
        uint64_t width;
        uint64_t height;
        uint64_t bytes_per_pixel;
        uint64_t reserved;
    };

    struct req_head_t
    {
        uint32_t command;
        uint32_t sequence;
    };

    struct rsp_head_t
    {
        uint32_t command;
        uint32_t sequence;
        uint32_t result;
    };

    struct visual_frame_fragment_head_t
    {
        uint64_t position;
        uint64_t time_stamp;
        uint64_t fragment_count;
        uint64_t fragment_index;
    };

    struct visual_fragment_info_t{
        uint32_t fragment_count;
        uint32_t remain;
    };

        struct visual_fragment_t
    {
        visual_frame_fragment_head_t visual_frame_fragment_head;
        uint8_t visual_frame_fragment_data[VISUAL_FRAME_FRAGMENT_BUFFER_SIZE - sizeof(visual_frame_fragment_head_t)];
    };



        int32_t send_visual(const visual_buffer_info_t &visual_buffer_info, common::Net &udp_client);
    
     
