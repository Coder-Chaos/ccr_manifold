#include "udp_send.hpp"

int32_t send_visual(const visual_buffer_info_t &visual_buffer_info, common::Net &udp_client)
    {

        // utility::GetTime udp_send_time;
        const uint32_t count = (uint32_t)(visual_buffer_info.data_length / VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        const uint32_t remain = (uint32_t)(visual_buffer_info.data_length % VISUAL_FRAME_FRAGMENT_DATA_SIZE);
        const uint32_t fragment_count = count + (remain != 0 ? 1 : 0);
        uint8_t visual_frame_fragment[VISUAL_FRAME_FRAGMENT_BUFFER_SIZE];
        visual_frame_fragment_head_t *visual_frame_fragment_head = (visual_frame_fragment_head_t *)visual_frame_fragment;
        visual_frame_fragment_head->position = visual_buffer_info.position;
        visual_frame_fragment_head->time_stamp = visual_buffer_info.time_stamp;
        visual_frame_fragment_head->fragment_count = fragment_count;
        uint8_t *visual_frame_fragment_data = visual_frame_fragment + sizeof(visual_frame_fragment_head_t);
        // log_trace("count：%d", count);

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
            // udp_send_time.Start();
            udp_client.UdpSendtoServer((const char *)visual_frame_fragment, sizeof(visual_frame_fragment_head_t) + VISUAL_FRAME_FRAGMENT_DATA_SIZE);
            // udp_send_time.Stop();
            // log_trace("udp send one fragment time:%ld ms", udp_send_time.Process());
        }

        if (remain != 0)
        {
            visual_frame_fragment_head->fragment_index = count;
            const uint8_t *visual_data = visual_buffer_info.data + count * VISUAL_FRAME_FRAGMENT_DATA_SIZE;
            for (uint64_t j = 0; j < remain; ++j)
            {
                visual_frame_fragment_data[j] = visual_data[j];
            }
            // udp_send_time.Start();
            udp_client.UdpSendtoServer((const char *)visual_frame_fragment, sizeof(visual_frame_fragment_head_t) + remain);
            // udp_send_time.Stop();
            // log_trace("udp send one fragment time:%ld ms", udp_send_time.Process());
        }

        return 0;
    }

