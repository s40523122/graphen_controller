#include "can_driver/socketcan_interface.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/can/raw.h>

namespace can_driver
{

SocketCanInterface::SocketCanInterface(const std::string & channel)
    : channel_(channel),
      sock_fd_(-1) {
}

SocketCanInterface::~SocketCanInterface() {
    close();
}

bool SocketCanInterface::open() {
    if (sock_fd_ >= 0) {
        return true;
    }

    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        std::perror("SocketCanInterface: socket");
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", channel_.c_str());

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        std::perror("SocketCanInterface: SIOCGIFINDEX");
        ::close(fd);
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::perror("SocketCanInterface: bind");
        ::close(fd);
        return false;
    }

    sock_fd_ = fd;
    return true;
}

void SocketCanInterface::close() {
    if (sock_fd_ < 0) {
        return;
    }
    ::close(sock_fd_);
    sock_fd_ = -1;
}

bool SocketCanInterface::isOpen() const noexcept {
    return sock_fd_ >= 0;
}

bool SocketCanInterface::sendFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc) {
    if (!isOpen() || !data || dlc > 8) {
        return false;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    return sendFrame(frame);
}

bool SocketCanInterface::sendFrame(const struct can_frame & frame) {
    if (!isOpen()) {
        return false;
    }

    ssize_t nbytes = ::write(sock_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        std::perror("SocketCanInterface: write");
        return false;
    }
    return true;
}

bool SocketCanInterface::receiveFrame(struct can_frame & frame, int timeout_ms) {
    if (timeout_ms < 0) {
        return waitAndRead(frame, nullptr);
    }
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    return waitAndRead(frame, &tv);
}

bool SocketCanInterface::receiveFrame(struct can_frame & frame, std::chrono::microseconds timeout) {
    const long long us = std::max<long long>(timeout.count(), 0);
    struct timeval tv;
    tv.tv_sec = static_cast<time_t>(us / 1000000);
    tv.tv_usec = static_cast<suseconds_t>(us % 1000000);
    return waitAndRead(frame, &tv);
}

bool SocketCanInterface::waitAndRead(struct can_frame & frame, struct timeval * timeout) {
    if (!isOpen()) {
        return false;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock_fd_, &readfds);

    int ret = ::select(sock_fd_ + 1, &readfds, nullptr, nullptr, timeout);
    if (ret < 0) {
        std::perror("SocketCanInterface: select");
        return false;
    }
    if (ret == 0) {
        // timeout
        return false;
    }

    ssize_t nbytes = ::read(sock_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        std::perror("SocketCanInterface: read");
        return false;
    }

    return true;
}

}  // namespace can_driver

