#include "image_process.hpp"
#include "utility.hpp"
#include "log.h"
namespace quick_inspection
{
    image_process::image_process(const common::Net &&udp_client) : udp_client_(udp_client)
    {
    }

    int image_process::Init(void)
    {
        int ret = 0;

        // init udp
        // common::Net udp_client_{UDP, client_ip_addr_, kClientPort};

        return ret;
    }

    int32_t image_process::send_visual(const visual_buffer_info_t &visual_buffer_info)
    {

        utility::GetTime udp_send_time;
        const uint32_t count = (uint32_t)(visual_buffer_info.data_length / VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        const uint32_t remain = (uint32_t)(visual_buffer_info.data_length % VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        const uint32_t fragment_count = count + (remain != 0 ? 1 : 0);
        uint8_t visual_frame_fragment[VISUAL_FRAME_FRAGMENT_BUFFER_SIZE];
        visual_frame_fragment_head_t *visual_frame_fragment_head = (visual_frame_fragment_head_t *)visual_frame_fragment;
        visual_frame_fragment_head->position = visual_buffer_info.position;
        visual_frame_fragment_head->time_stamp = visual_buffer_info.time_stamp;
        visual_frame_fragment_head->fragment_count = fragment_count;
        uint8_t *visual_frame_fragment_data = visual_frame_fragment + sizeof(visual_frame_fragment_head_t);
        log_trace("count：%d", count);

        // compose visual frame fragment
        for (uint32_t i = 0; i < count; ++i)
        {
            visual_frame_fragment_head->fragment_index = i;
            const uint8_t *visual_data = visual_buffer_info.data + i * VISUAL_FRAME_FRAGMENT_DATA_SIZE;
            // copy  data to fragment
            for (uint64_t j = 0; j < VISUAL_FRAME_FRAGMENT_DATA_SIZE; ++j)
            {
                visual_frame_fragment_data[j] = visual_data[j];
            }
            udp_send_time.Start();
            udp_client_.UdpSendtoServer((const char *)visual_frame_fragment, sizeof(visual_frame_fragment_head_t) + VISUAL_FRAME_FRAGMENT_DATA_SIZE);
            udp_send_time.Stop();
            log_trace("udp send one fragment time:%ld ms", udp_send_time.Process());
        }

        if (remain != 0)
        {
            visual_frame_fragment_head->fragment_index = count;
            const uint8_t *visual_data = visual_buffer_info.data + count * VISUAL_FRAME_FRAGMENT_DATA_SIZE;
            for (uint64_t j = 0; j < remain; ++j)
            {
                visual_frame_fragment_data[j] = visual_data[j];
            }
            udp_send_time.Start();
            udp_client_.UdpSendtoServer((const char *)visual_frame_fragment, sizeof(visual_frame_fragment_head_t) + remain);
            udp_send_time.Stop();
            log_trace("udp send one fragment time:%ld ms", udp_send_time.Process());
        }

        return 0;
    }



    visual_fragment_info_t image_process::compose_visual_frame_fragment(const visual_buffer_info_t &visual_buffer_info, visual_fragment_t *visual_fragment_array)
    {
        // utility::GetTime udp_send_time;
        const uint32_t count = (uint32_t)(visual_buffer_info.data_length / VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        const uint32_t remain = (uint32_t)(visual_buffer_info.data_length % VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        const uint32_t fragment_count = count + (remain != 0 ? 1 : 0);
        visual_fragment_info_t fragment_info;

        // set fragment head information
        for (int i = 0; i < count; i++)
        {
            visual_fragment_array[i].visual_frame_fragment_head.position = visual_buffer_info.position;
            visual_fragment_array[i].visual_frame_fragment_head.time_stamp = visual_buffer_info.time_stamp;
            visual_fragment_array[i].visual_frame_fragment_head.fragment_count = fragment_count;
            visual_fragment_array[i].visual_frame_fragment_head.fragment_index = i;
            memcpy(visual_fragment_array[i].visual_frame_fragment_data, visual_buffer_info.data + i * VISUAL_FRAME_FRAGMENT_DATA_SIZE, VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        }

        if (remain != 0)
        {
            visual_fragment_array[count].visual_frame_fragment_head.position = visual_buffer_info.position;
            visual_fragment_array[count].visual_frame_fragment_head.time_stamp = visual_buffer_info.time_stamp;
            visual_fragment_array[count].visual_frame_fragment_head.fragment_count = fragment_count;
            visual_fragment_array[count].visual_frame_fragment_head.fragment_index = count;
            memcpy(visual_fragment_array[count].visual_frame_fragment_data, visual_buffer_info.data + remain, remain);
        }

        fragment_info.fragment_count = fragment_count;
        fragment_info.remain = remain;

        return fragment_info;
    }

    int32_t image_process::send_visual(visual_fragment_t *visual_fragment_array, const visual_fragment_info_t &fragment_info)
    {
        int i;
        for (i = 0; i < fragment_info.fragment_count; i++)
        {
            udp_client_.UdpSendtoServer((char *)&visual_fragment_array[i], sizeof(visual_frame_fragment_head_t) + VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        }

        if (fragment_info.remain != 0)
        {
            udp_client_.UdpSendtoServer((char *)&visual_fragment_array[i], sizeof(visual_frame_fragment_head_t) + fragment_info.remain);
        }

        return 0;
    }

    void image_process::compose_visual_buffer_info(uint64_t position, const dvpFrameBuffer &frame, visual_buffer_info_t &visual_buffer_info)
    {
        const uint64_t photo_stride = frame.frame.iWidth * 3;
        const uint64_t visual_width = (frame.frame.iWidth >> COMPRESS_RATIO);
        const uint64_t visual_height = (frame.frame.iHeight >> COMPRESS_RATIO);
        const uint64_t visual_stride = (photo_stride >> COMPRESS_RATIO);
        visual_buffer_info.position = position;
        visual_buffer_info.time_stamp = frame.frame.uFrameID;
        visual_buffer_info.data_length = visual_height * visual_stride + sizeof(photo_info_t);
        photo_info_t *visual_info = (photo_info_t *)(visual_buffer_info.data);
        visual_info->position = position;
        visual_info->group = 0;
        visual_info->sequence = 0;
        visual_info->time_stamp = frame.frame.uFrameID;
        visual_info->width = visual_width;
        visual_info->height = visual_height;
        visual_info->bytes_per_pixel = 3;

        uint8_t *visual_data = visual_buffer_info.data + sizeof(photo_info_t);
        for (uint64_t i = 0; i < visual_height; ++i)
        {
            for (uint64_t j = 0; j < visual_stride; j += 3)
            {
                visual_data[i * visual_stride + j + 0] = frame.pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 0];
                visual_data[i * visual_stride + j + 1] = frame.pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 1];
                visual_data[i * visual_stride + j + 2] = frame.pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 2];
            }
        }
    }

    void image_process::compose_visual_buffer_info(uint64_t position, const dvpFrame *frame_info, const dvpByte *pBuffer, visual_buffer_info_t &visual_buffer_info)
    {
        const uint64_t photo_stride = frame_info->iWidth * 3;
        const uint64_t visual_width = (frame_info->iWidth >> COMPRESS_RATIO);
        const uint64_t visual_height = (frame_info->iHeight >> COMPRESS_RATIO);
        const uint64_t visual_stride = (photo_stride >> COMPRESS_RATIO);
        visual_buffer_info.position = position;
        visual_buffer_info.time_stamp = frame_info->uFrameID;
        // printf("\rframe ID:%d\n", frame_info->uFrameID);
        visual_buffer_info.data_length = visual_height * visual_stride + sizeof(photo_info_t);
        photo_info_t *visual_info = (photo_info_t *)(visual_buffer_info.data);
        visual_info->position = position;
        visual_info->group = 0;
        visual_info->sequence = 0;
        visual_info->time_stamp = frame_info->uFrameID;
        visual_info->width = visual_width;
        visual_info->height = visual_height;
        visual_info->bytes_per_pixel = 3;

        uint8_t *visual_data = visual_buffer_info.data + sizeof(photo_info_t);
        for (uint64_t i = 0; i < visual_height; ++i)
        {
            for (uint64_t j = 0; j < visual_stride; j += 3)
            {
                visual_data[i * visual_stride + j + 0] = pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 0];
                visual_data[i * visual_stride + j + 1] = pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 1];
                visual_data[i * visual_stride + j + 2] = pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 2];
            }
        }
    }

    void image_process::compose_visual_buffer_info(uint64_t position, const dvpByte *pBuffer, int frame_width, int frame_height, long long frame_cnt, visual_buffer_info_t &visual_buffer_info)
    {
        const uint64_t photo_stride = frame_width * 3;
        const uint64_t visual_width = (frame_width >> COMPRESS_RATIO);
        const uint64_t visual_height = (frame_height >> COMPRESS_RATIO);
        const uint64_t visual_stride = (photo_stride >> COMPRESS_RATIO);
        visual_buffer_info.position = position;
        visual_buffer_info.time_stamp = frame_cnt;

        visual_buffer_info.data_length = visual_height * visual_stride + sizeof(photo_info_t);
        photo_info_t *visual_info = (photo_info_t *)(visual_buffer_info.data);
        visual_info->position = position;
        visual_info->group = 0;
        visual_info->sequence = 0;
        visual_info->time_stamp = frame_cnt;
        visual_info->width = visual_width;
        visual_info->height = visual_height;
        visual_info->bytes_per_pixel = 3;

        uint8_t *visual_data = visual_buffer_info.data + sizeof(photo_info_t);
#pragma omp parallel for
        for (uint64_t i = 0; i < visual_height; ++i)
        {
#pragma omp parallel for
            for (uint64_t j = 0; j < visual_stride; j += 3)
            {
                visual_data[i * visual_stride + j + 0] = pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 0];
                visual_data[i * visual_stride + j + 1] = pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 1];
                visual_data[i * visual_stride + j + 2] = pBuffer[(i << COMPRESS_RATIO) * photo_stride + (j << COMPRESS_RATIO) + 2];
            }
        }
    }

} // namespace quick_inspection