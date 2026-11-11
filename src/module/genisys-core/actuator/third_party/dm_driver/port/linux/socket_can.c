#include "socket_can.h"
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

int SocketCAN_Connect(const char* can_port, bool fd_mode) {
  int sock_fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd == -1) {
    return -1;
  }

  if (fd_mode) {
    int enable_canfd = 1;
    if (setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) == -1) {
      close(sock_fd);
      return -1;
    }
  }

  // 允许接收本 socket 发出的帧（vcan 回环测试必需）
  int recv_own = 1;
  if (setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own)) == -1) {
    close(sock_fd);
    return -1;
  }

  // 知道can接口
  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, can_port, IFNAMSIZ - 1);
  if (ioctl(sock_fd, SIOCGIFFLAGS, &ifr) == -1) {
    close(sock_fd);
    return -1;
  }

  if (!(ifr.ifr_flags & IFF_UP)) {
    close(sock_fd);
    return -1;
  }

  // ifr_flags 与 ifr_ifindex 共用同一个 union，必须在查完 flags 之后再取 index
  if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) == -1) {
    close(sock_fd);
    return -1;
  }

  // 绑定socket到can接口
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
    close(sock_fd);
    return -1;
  }

  // 接收超时10ms（对read生效）
  struct timeval rcv_timeout;
  rcv_timeout.tv_sec = 0;
  rcv_timeout.tv_usec = 10000;
  if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout)) == -1) {
    close(sock_fd);
    return -1;
  }

  // 发送超时2ms（对write生效）
  struct timeval snd_timeout;
  snd_timeout.tv_sec = 0;
  snd_timeout.tv_usec = 2000;
  if (setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout)) == -1) {
    close(sock_fd);
    return -1;
  }

  return sock_fd;
}

int SocketCAN_SetFilters(int sock_fd, const uint32_t* ids, int count) {
  if (sock_fd == -1 || ids == NULL || count <= 0) {
    return -1;
  }

  struct can_filter filters[count];
  for (int i = 0; i < count; i++) {
    filters[i].can_id = ids[i] & CAN_SFF_MASK;
    filters[i].can_mask = CAN_SFF_MASK;
  }

  return setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                    filters, sizeof(filters));
}

static int SocketCAN_SendFrame(int sock_fd, const void* frame, size_t len) {
  for (int retry = 0; retry < 3; ++retry) {
    ssize_t n = send(sock_fd, frame, len, MSG_DONTWAIT);
    if (n >= 0) {
      return (int)n;
    }
    if (errno != EINTR) {
      return -1;
    }
  }
  return -1;
}

int SocketCAN_Send(int sock_fd, uint32_t id, uint8_t dlc, const uint8_t* data) {
  if (sock_fd == -1 || dlc > CAN_MAX_DLC) {
    return -1;
  }
  struct can_frame frame;
  frame.can_id = id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);

  int n = SocketCAN_SendFrame(sock_fd, &frame, sizeof(frame));
  return (n == (int)sizeof(frame)) ? n : -1;
}

int SocketCAN_Receive(int sock_fd, uint32_t* id, uint8_t* dlc, uint8_t* data) {
  if (sock_fd == -1) {
    return -1;
  }

  struct can_frame frame;
  int nbytes = recv(sock_fd, &frame, sizeof(struct can_frame), MSG_DONTWAIT);
  if (nbytes < 0) {
    return -1;
  }

  *id = frame.can_id;
  *dlc = frame.can_dlc;
  memcpy(data, frame.data, frame.can_dlc);

  return nbytes;
}

int SocketCANFD_Send(int sock_fd, uint32_t id, uint8_t dlc, bool brs, const uint8_t* data) {
  if (sock_fd == -1 || dlc > CANFD_MAX_DLEN) {
    return -1;
  }

  struct canfd_frame frame;
  frame.can_id = id;
  frame.len = dlc;
  frame.flags = brs ? CANFD_BRS : 0;
  memcpy(frame.data, data, dlc);

  int n = SocketCAN_SendFrame(sock_fd, &frame, sizeof(frame));
  return (n == (int)sizeof(frame)) ? n : -1;
}

int SocketCANFD_Receive(int sock_fd, uint32_t* id, uint8_t* dlc, uint8_t* data) {
  if (sock_fd == -1) {
    return -1;
  }

  struct canfd_frame frame;
  int nbytes = recv(sock_fd, &frame, sizeof(struct can_frame), MSG_DONTWAIT);
  if (nbytes < 0) {
    return -1;
  }

  *id = frame.can_id;
  *dlc = frame.len;
  memcpy(data, frame.data, frame.len);

  return nbytes;
}

void SocketCAN_Close(int sock_fd) {
  if (sock_fd != -1) {
    close(sock_fd);
  }
}
