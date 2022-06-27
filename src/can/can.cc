#include "can.hpp"

can::can(char *can_dev) : can_dev_(can_dev)
{
  //  int i, j;
  struct sockaddr_can addr;
  struct ifreq ifr;
  int ret;

  // the stack will overflow if not add 1?
  struct can_filter rfilter[kCanFilterNum + 1];

  // create socket
  s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(ifr.ifr_name, can_dev_);

  ioctl(s_, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // bind can0
  ret = bind(s_, (struct sockaddr *)&addr, sizeof(addr));
  if (ret == 0)
  {
    log_info("%s bind socket success.", can_dev_);
  }
  else
  {
    log_error("%s bind socket failed.", can_dev_);
    exit(-1);
  }

  // set the socket fileter options
  setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
}

can::~can() {}

int can::send(const struct can_frame *send_frame)
{
  return write(s_, send_frame, sizeof(*send_frame));
}

int can::receive(can_frame *recv_frame)
{
  return read(s_, recv_frame, sizeof(*recv_frame));
}
