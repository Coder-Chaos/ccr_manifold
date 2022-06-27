#include "robot.hpp"
#include "uvc.hpp"
#define TEST_MOTION 1
#define TEST_UVC 1
#include <iostream>

using namespace std;

int main(int argc, char **argv) {

#if DEBUG_ON
  // set log level
  log_set_level(1);
#else
  log_set_level(2);
#endif
  ccr_quick_detection::Robot ccr;

  // connect host
  ccr.ConnectHost();

  // start guard thread
  ccr.Guard();

  //ccr.StartVisionDetection();

  // start tfmini thread
  //ccr.TFminiRecv();

#if TEST_MOTION
  // robot run
  ccr.Run();
#endif

#if TEST_UVC
  ccr.StartUVC();
#endif

  // wait to exit
  PAUSE();

#if TEST_MOTION
  //   wait thread complete
  ccr.Stop();
#endif

  //ccr.TFminiStop();

#if TEST_UVC
  ccr.StopUVC();
#endif

  //ccr.StopVisionDetection();

  ccr.DisConnectHost();

  ccr.ShutDown();

  return 0;
}
