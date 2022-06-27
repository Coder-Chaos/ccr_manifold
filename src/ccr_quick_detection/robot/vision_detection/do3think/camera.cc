#include "camera.hpp"

#include <stdio.h>
#include <memory.h>

#include "DVPCamera.h"
#include <iostream>
#include <mutex>
#include <string>

#define MAX_CAMERA_COUNT 16

static std::mutex cam_mutex;

namespace {
static dvpUint32 CameraCount;
static dvpCameraInfo CameraInfo[MAX_CAMERA_COUNT];
static dvpHandle CameraHandle[MAX_CAMERA_COUNT];

} // namespace

int32_t Camera::init(int &cnt) {
  dvpStatus status = dvpRefresh(&CameraCount);
  if (status != DVP_STATUS_OK) {
    perror("Error at dvpRefresh\n");
    return -1;
  }

  printf("CameraCount:%d\n", CameraCount);
  cnt = CameraCount;
  if (CameraCount < 1) {
    perror("Error, CameraCount < 1\n");
    return -2;
  }

  for (dvpUint32 i = 0; i < CameraCount; ++i) {
    status = dvpEnum(i, CameraInfo + i);
    if (status != DVP_STATUS_OK) {
      // std::cout << "Error at dvpEnum: " << status << std::endl;
      printf("Error at dvpEnum: %d\n", status);
      continue;
    }

    // std::cout << "FriendlyName: " << CameraInfo[i].FriendlyName << std::endl;
    printf("FriendlyName: %s\n", CameraInfo[i].FriendlyName);
    // std::cout << "SerialNumber: " << CameraInfo[i].SerialNumber << std::endl;
    printf("SerialNumber: %s\n", CameraInfo[i].SerialNumber);

    status = dvpOpen(i, OPEN_NORMAL, CameraHandle + i);
    if (status != DVP_STATUS_OK) {
      // std::cout << "Error at " << CameraInfo[i].FriendlyName << " dvpOpen: "
      // << status << std::endl;
      printf("Error at:%s dvpOpen:%d\n", CameraInfo[i].FriendlyName, status);
      continue;
    }
    {
      dvpSetAutoDefectFixState(CameraHandle[i], false);
      dvpSetBlackLevelState(CameraHandle[i], false);
      dvpSetColorTemperatureState(CameraHandle[i], false);
      dvpSetContrastState(CameraHandle[i], false);
      dvpSetDefectFixState(CameraHandle[i], false);
      dvpSetFlatFieldState(CameraHandle[i], false);
      dvpSetFlipHorizontalState(CameraHandle[i], false);
      dvpSetFlipVerticalState(CameraHandle[i], false);
      dvpSetGammaState(CameraHandle[i], false);
      dvpSetInverseState(CameraHandle[i], false);
      dvpSetMonoState(CameraHandle[i], false);
      dvpSetNoiseReduct2dState(CameraHandle[i], false);
      dvpSetNoiseReduct3dState(CameraHandle[i], false);
      dvpSetRgbGainState(CameraHandle[i], false);
      dvpSetRoiState(CameraHandle[i], false);
      dvpSetRotateState(CameraHandle[i], false);
      dvpSetSaturationState(CameraHandle[i], false);
      dvpSetSharpnessState(CameraHandle[i], false);
      dvpSetSoftTriggerLoopState(CameraHandle[i], false);

      dvpSetTriggerState(CameraHandle[i], false);
      dvpSetTriggerSource(CameraHandle[i], TRIGGER_SOURCE_SOFTWARE);
      dvpSetExposure(CameraHandle[i], 10000.0);
      dvpSetQuickRoiSel(CameraHandle[i], 0);

      // set resolution
      dvpSetResolutionModeSel(CameraHandle[i], 1);

      // set format, 0-RAW8, 21-YUV422
      dvpSetSourceFormatSel(CameraHandle[i], 0);
      dvpSetTargetFormatSel(CameraHandle[i], 0);
    }

    status = dvpStart(CameraHandle[i]);
    if (status != DVP_STATUS_OK) {
      printf("Error at:%s dvpStart:%d\n", CameraInfo[i].FriendlyName, status);
    }
  }

  return 0;
}

int32_t Camera::init(int &cnt, dvpStreamFormat format) {
  dvpStatus status = dvpRefresh(&CameraCount);
  if (status != DVP_STATUS_OK) {
    perror("Error at dvpRefresh\n");
    return -1;
  }

  printf("CameraCount:%d\n", CameraCount);
  cnt = CameraCount;
  if (CameraCount < 1) {
    perror("Error, CameraCount < 1\n");
    return -2;
  }

  for (dvpUint32 i = 0; i < CameraCount; ++i) {
    status = dvpEnum(i, CameraInfo + i);
    if (status != DVP_STATUS_OK) {
      // std::cout << "Error at dvpEnum: " << status << std::endl;
      printf("Error at dvpEnum: %d\n", status);
      continue;
    }

    // std::cout << "FriendlyName: " << CameraInfo[i].FriendlyName << std::endl;
    printf("FriendlyName: %s\n", CameraInfo[i].FriendlyName);
    // std::cout << "SerialNumber: " << CameraInfo[i].SerialNumber << std::endl;
    printf("SerialNumber: %s\n", CameraInfo[i].SerialNumber);

    status = dvpOpen(i, OPEN_NORMAL, CameraHandle + i);
    if (status != DVP_STATUS_OK) {
      // std::cout << "Error at " << CameraInfo[i].FriendlyName << " dvpOpen: "
      // << status << std::endl;
      printf("Error at:%s dvpOpen:%d\n", CameraInfo[i].FriendlyName, status);
      continue;
    }
    {
      #if 0
      dvpSetAutoDefectFixState(CameraHandle[i], false);
      dvpSetBlackLevelState(CameraHandle[i], false);
      dvpSetColorTemperatureState(CameraHandle[i], false);
      dvpSetContrastState(CameraHandle[i], false);
      dvpSetDefectFixState(CameraHandle[i], false);
      dvpSetFlatFieldState(CameraHandle[i], false);
      dvpSetFlipHorizontalState(CameraHandle[i], false);
      dvpSetFlipVerticalState(CameraHandle[i], false);
      dvpSetGammaState(CameraHandle[i], false);
      dvpSetInverseState(CameraHandle[i], false);
      dvpSetMonoState(CameraHandle[i], false);
      dvpSetNoiseReduct2dState(CameraHandle[i], false);
      dvpSetNoiseReduct3dState(CameraHandle[i], false);
      // dvpSetRgbGainState(CameraHandle[i], false);
      dvpSetRoiState(CameraHandle[i], false);
      dvpSetRotateState(CameraHandle[i], false);
      dvpSetSaturationState(CameraHandle[i], false);
      dvpSetSharpnessState(CameraHandle[i], false);
      dvpSetSoftTriggerLoopState(CameraHandle[i], false);

      dvpSetTriggerState(CameraHandle[i], false);
      dvpSetTriggerSource(CameraHandle[i], TRIGGER_SOURCE_SOFTWARE);
      // dvpSetExposure(CameraHandle[i], 10000.0);
      dvpSetQuickRoiSel(CameraHandle[i], 0);

      // dvpSetRgbGainState(CameraHandle[i], true);
      // dvpSetRgbGain(CameraHandle[i], 2.0, 1.0, 2.0);

      // set resolution
      dvpSetResolutionModeSel(CameraHandle[i], 0);

      // set format, 0-RAW8, 21-YUV422
      dvpSetSourceFormatSel(CameraHandle[i], format);
      dvpSetTargetFormatSel(CameraHandle[i], format);
      #endif
      //  dvpSetPixelRateSel(CameraHandle[i], 1);
      if(i==1||i==3){
        dvpSetAeTarget(CameraHandle[i],100);
      }
    }

    status = dvpStart(CameraHandle[i]);
    if (status != DVP_STATUS_OK) {
      printf("Error at:%s dvpStart:%d\n", CameraInfo[i].FriendlyName, status);
    }
  }

  return 0;
}

int32_t Camera::fini(void) {
  for (dvpUint32 i = 0; i < CameraCount; ++i) {
    dvpStatus status = dvpStop(CameraHandle[i]);
    if (status != DVP_STATUS_OK) {
      // std::cout << "Error at " << CameraInfo[i].FriendlyName << " dvpStop: "
      // << status << std::endl;
      printf("Error at:%s dvpStop:%d\n", CameraInfo[i].FriendlyName, status);
    }

    status = dvpClose(CameraHandle[i]);
    if (status != DVP_STATUS_OK) {
      // std::cout << "Error at " << CameraInfo[i].FriendlyName << " dvpClose: "
      // << status << std::endl;
      printf("Error at:%s dvpClose:%d\n", CameraInfo[i].FriendlyName, status);
    }
  }

  return 0;
}
#if 0
int32_t Camera::TakePicture(uint64_t position, dvpFrameBuffer &raw,
                            dvpFrameBuffer &out) {
  dvpStatus status =
      dvpGetFrameBuffer(CameraHandle[position], &raw, &out, 1000);
  if (status != DVP_STATUS_OK) {
    // std::cout << "Error at " << CameraInfo[position].FriendlyName << "
    // dvpGetFrameBuffer: " << status << std::endl;
    printf("Error at:%s dvpGetFrameBuffer:%d\n",
           CameraInfo[position].FriendlyName, status);
    return -1;
  }

  return 0;
}

int32_t Camera::TakePicture(uint64_t position, dvpFrame *pFrame, void **pBuffer,
                            dvpUint32 timeout) {
  std::lock_guard<std::mutex> lock(cam_mutex);
  dvpStatus status =
      dvpGetFrame(CameraHandle[position], pFrame, pBuffer, timeout);

  if (status != DVP_STATUS_OK) {
    printf("\rcamera[%d], get frame failed\n", position);
  }

// print frame info
#if 0
    dvpFrameCount framecount;
    status = dvpGetFrameCount(CameraHandle[position], &framecount);
    if (status != DVP_STATUS_OK)
    {
        printf("\rcamera[%d], get framecount failed\n", position);
    }
    else
    {
        printf("\rcamera[%d], framecount: %d, framerate: %f\n", position, framecount.uFrameCount, framecount.fFrameRate);
        printf("\rframeformat: %d, framebytes:%d bytes, framewidth:%d, frameheight:%d\n", pFrame->format, pFrame->uBytes, pFrame->iWidth, pFrame->iHeight);
    }
#endif
  return status;
}
#endif

int32_t Camera::TakePicture(uint64_t position, dvpFrame *pFrame, void *pBuffer, dvpUint32 timeout) {
  std::lock_guard<std::mutex> lock(cam_mutex);
  void *_;
  dvpStatus status = dvpGetFrame(CameraHandle[position], pFrame, &_, timeout);

  if (status != DVP_STATUS_OK) {
    printf("\rcamera[%d], get frame failed\n", position);
  return status;
  }

  memcpy(pBuffer, _, pFrame->uBytes);

  return status;
}

int32_t Camera::SavePicture(uint64_t group, uint64_t sequence,
                            uint64_t position, const dvpFrameBuffer &raw,
                            const dvpFrameBuffer &out) {
  std::string file_path = std::to_string(group) + '_' +
                          std::to_string(sequence) + '_' +
                          std::to_string(position) + ".png";
  std::cout << file_path << std::endl;
#if 0
    dvpStatus status = dvpSavePicture(&(out.frame), out.pBuffer, file_path.data(), 100);
    if (status != DVP_STATUS_OK)
    {
        std::cout << "Error at " << CameraInfo[position].FriendlyName << " dvpSavePicture: " << status << std::endl;
        return -1;
    }

#endif
  return 0;
}

int32_t Camera::SavePicture(uint64_t position, const dvpFrame *pFrame,
                            const void *pBuffer) {
  char PicName[64];
  dvpStatus status;
  // sprintf(PicName, "cam", "%d_pic_%d.jpg", position, cnt);
  // sprintf(PicName, "./saved_pics/%s_%d_pic_%d.jpg", "cam",position, cnt);
  sprintf(PicName, "%s_%d_pic_%d.dat", "cam", position, pFrame->uFrameID);
  status = dvpSavePicture(pFrame, pBuffer, PicName, 90);
  if (status == DVP_STATUS_OK) {
    printf("Save to %s OK\r\n", PicName);
  }
  return status;
}
