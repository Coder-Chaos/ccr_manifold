#include "do3think.hpp"

#include "log.h"
Do3Think::Do3Think(int id) : id_(id) {
  /* init */

  dvpStatus status = dvpRefresh(&CameraCount_);
  if (status != DVP_STATUS_OK) {
    log_error("Error at dvpRefresh");
    // return -1;
    exit(-1);
  }

  log_info(" %d cameras in total,current id:%d", CameraCount_, id_);

  //   enum
  status = dvpEnum(id_, &CameraInfo_);
  if (status != DVP_STATUS_OK) {
    log_error("Error at dvpEnum: %d", status);
  }

  // print camera info
  log_info("FriendlyName: %s", CameraInfo_.FriendlyName);
  log_info("SerialNumber: %s", CameraInfo_.SerialNumber);

  //   open camera
  status = dvpOpen(id_, OPEN_NORMAL, &CameraHandle_);
  if (status != DVP_STATUS_OK) {
    log_error("Error at:%s dvpOpen:%d", CameraInfo_.FriendlyName, status);
  }
  {
    dvpSetAutoDefectFixState(CameraHandle_, false);
    dvpSetBlackLevelState(CameraHandle_, false);
    dvpSetColorTemperatureState(CameraHandle_, false);
    dvpSetContrastState(CameraHandle_, false);
    dvpSetDefectFixState(CameraHandle_, false);
    dvpSetFlatFieldState(CameraHandle_, false);
    dvpSetFlipHorizontalState(CameraHandle_, false);
    dvpSetFlipVerticalState(CameraHandle_, false);
    dvpSetGammaState(CameraHandle_, false);
    dvpSetInverseState(CameraHandle_, false);
    dvpSetMonoState(CameraHandle_, false);
    dvpSetNoiseReduct2dState(CameraHandle_, false);
    dvpSetNoiseReduct3dState(CameraHandle_, false);
    dvpSetRoiState(CameraHandle_, false);
    dvpSetRotateState(CameraHandle_, false);
    dvpSetSaturationState(CameraHandle_, false);
    dvpSetSharpnessState(CameraHandle_, false);
    dvpSetSoftTriggerLoopState(CameraHandle_, false);

    dvpSetTriggerState(CameraHandle_, false);
    dvpSetTriggerSource(CameraHandle_, TRIGGER_SOURCE_SOFTWARE);
    dvpSetExposure(CameraHandle_, 10000.0);
    dvpSetQuickRoiSel(CameraHandle_, 0);

    dvpSetAwbOperation(CameraHandle_, AWB_OP_CONTINUOUS);
    // dvpSetRgbGainState(CameraHandle_, true);
    // dvpSetRgbGain(CameraHandle_, 2.0, 1.0, 2.0);

    // set resolution
    dvpSetResolutionModeSel(CameraHandle_, 1);

    // set format, 0-RAW8, 21-YUV422
    dvpSetSourceFormatSel(CameraHandle_, 0);
    dvpSetTargetFormatSel(CameraHandle_, 10);
  }

  // start camera
  status = dvpStart(CameraHandle_);
  if (status != DVP_STATUS_OK) {
    log_error("Error at:%s dvpStart:%d", CameraInfo_.FriendlyName, status);
  }
}

// take picture
int32_t Do3Think::TakePicture() {
  dvpStatus status =
      dvpGetFrame(CameraHandle_, &pFrame_, (void **)&pBuffer_, 100);

  if (status != DVP_STATUS_OK) {
    log_error("\rcamera[%d], get frame failed\n", id_);
    return status;
  }
}
