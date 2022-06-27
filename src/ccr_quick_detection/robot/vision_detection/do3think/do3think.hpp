#pragma once
#include "DVPCamera.h"
#include "common.hpp"

class Do3Think {
 private:
  //   cam id
  int id_;

  dvpUint32 CameraCount_;
  dvpCameraInfo CameraInfo_;
  dvpHandle CameraHandle_;

 public:
  dvpFrame pFrame_;
  uint8_t *pBuffer_ = nullptr;

 public:
  Do3Think(int id, uint8_t *pBuffer);
  Do3Think(int id);

  Do3Think(/* args */) {}
  ~Do3Think() {
    /* deinit */
    dvpStatus status = dvpStop(CameraHandle_);
    if (status != DVP_STATUS_OK) {
      printf("Error at:%s dvpStop:%d\n", CameraInfo_.FriendlyName, status);
    }

    status = dvpClose(CameraHandle_);
    if (status != DVP_STATUS_OK) {
      printf("Error at:%s dvpClose:%d\n", CameraInfo_.FriendlyName, status);
    }
  }

  // take picture
  int32_t TakePicture();

  //   save picture
};
