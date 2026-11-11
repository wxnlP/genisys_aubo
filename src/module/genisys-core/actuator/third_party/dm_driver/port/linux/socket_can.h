#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

int SocketCAN_Connect(const char* can_port, bool fd_mode);
int SocketCAN_Send(int sock_fd, uint32_t id, uint8_t dlc, const uint8_t* data);
int SocketCAN_Receive(int sock_fd, uint32_t* id, uint8_t* dlc, uint8_t* data);
int SocketCANFD_Send(int sock_fd, uint32_t id, uint8_t dlc, bool brs, const uint8_t* data);
int SocketCANFD_Receive(int sock_fd, uint32_t* id, uint8_t* dlc, uint8_t* data);
int SocketCAN_SetFilters(int sock_fd, const uint32_t* ids, int count);
void SocketCAN_Close(int sock_fd);

#ifdef __cplusplus
}
#endif