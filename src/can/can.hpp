#pragma once
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "log.h"
#include <string>

#include "log.h"

typedef __u8 uint8_t;

class can {
 private:
  /* data */
  int s_;
  char *can_dev_;

  // CAN filter number, 8 canopen devices, each has 4 TxPDO.
  static const __u8 kNodeNum = 6;

  static const int kCanFilterNum = 24;

 public:
  can(char *can_dev);
  can() {}
  ~can();

  void close_socketCAN(void);
  int send(const struct can_frame *send_frame);
  int receive(can_frame *recv_frame);
};
