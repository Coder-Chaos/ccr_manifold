#pragma once
#include "DVPCamera.h"
namespace Camera
{
    int32_t init(int &cnt);
    int32_t init(int &cnt, dvpStreamFormat format);
    int32_t fini(void);
    // int32_t TakePicture(uint64_t position, dvpFrameBuffer &raw, dvpFrameBuffer &out);
    // int32_t TakePicture(uint64_t position, dvpFrame *pFrame, void **pBuffer, dvpUint32 timeout);
    int32_t TakePicture(uint64_t position, dvpFrame *pFrame, void *pBuffer, dvpUint32 timeout);

    int32_t SavePicture(uint64_t group, uint64_t sequence, uint64_t position, const dvpFrameBuffer &raw, const dvpFrameBuffer &out);
    int32_t SavePicture(uint64_t position, const dvpFrame *pFrame, const void *pBuffer);
} // namespace Camera
