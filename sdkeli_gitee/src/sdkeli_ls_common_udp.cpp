#include "sdkeli_ls_common_udp.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <iterator>

namespace sdkeli_ls_udp
{

CSDKeliLsCommonUdp::CSDKeliLsCommonUdp(
  const std::string & hostname,
  int port,
  int & timelimit,
  CParserBase * parser,
  const rclcpp::Node::SharedPtr & node)
: CSDKeliLsCommon(parser, node),
  host_name_(hostname),
  mPort(port),
  time_limit_(timelimit)
{
}

CSDKeliLsCommonUdp::~CSDKeliLsCommonUdp()
{
  StopScanner();
  CloseDevice();
}

int CSDKeliLsCommonUdp::InitDevice()
{
  int opt = 1;
  struct sockaddr_in local_addr {};
  struct hostent * h = nullptr;

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Create UDP socket failed");
    return ExitError;
  }

  h = gethostbyname(host_name_.c_str());
  if (h == nullptr)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Unknown host '%s'", host_name_.c_str());
    return ExitError;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Sending data to '%s' (IP: %s) (PORT: %s)",
    h->h_name,
    inet_ntoa(*(struct in_addr *)h->h_addr_list[0]),
    std::to_string(mPort).c_str());

  remote_addr_.sin_family = h->h_addrtype;
  std::memcpy(
    &remote_addr_.sin_addr.s_addr,
    h->h_addr_list[0],
    h->h_length);

  remote_addr_.sin_port = htons(mPort);

  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = 0;  // let OS choose ephemeral port

  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  if (bind(socket_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Bind socket failed");
    close(socket_fd_);
    socket_fd_ = -1;
    return ExitError;
  }

  ClearConnectFlag();

  RCLCPP_INFO(node_->get_logger(), "UDP connection ready");

  // ✅ AUTO START
  send_start();
  connect_flag_ = true;

  return ExitSuccess;
}

int CSDKeliLsCommonUdp::CloseDevice()
{
  if (socket_fd_ != -1)
  {
    close(socket_fd_);
    socket_fd_ = -1;
    RCLCPP_INFO(node_->get_logger(), "UDP socket closed");
  }
  return ExitSuccess;
}

int CSDKeliLsCommonUdp::SendUdpData2Device(char * buf, int length)
{
  if (socket_fd_ < 0)
  {
    return -1;
  }

  return sendto(
    socket_fd_,
    buf,
    length,
    0,
    (struct sockaddr *)&remote_addr_,
    sizeof(remote_addr_));
}

void CSDKeliLsCommonUdp::send_start()
{
  const uint8_t cmd[] = {0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01};
  sendto(socket_fd_, cmd, sizeof(cmd), 0,
         (sockaddr*)&remote_addr_, sizeof(remote_addr_));

  RCLCPP_INFO(node_->get_logger(), "START command sent to LiDAR");
}

int CSDKeliLsCommonUdp::SendDeviceReq(
  const uint8_t * req,
  size_t len)
{
  if (socket_fd_ == -1)
  {
    RCLCPP_ERROR(node_->get_logger(), "SendDeviceReq: socket NOT open");
    return ExitError;
  }

  if (SendUdpData2Device(
        reinterpret_cast<char *>(const_cast<uint8_t *>(req)),
        len) != static_cast<int>(len))
  {
    RCLCPP_ERROR(node_->get_logger(), "Write error for request command");
    return ExitError;
  }

  return ExitSuccess;
}

int CSDKeliLsCommonUdp::GetDataGram(
  uint8_t * buffer,
  int buffer_size,
  int * length)
{
  struct timeval tv {};
  fd_set rfds;
  socklen_t addrlen;
  struct sockaddr_in recv_addr {};

  if (socket_fd_ == -1)
  {
    RCLCPP_ERROR(node_->get_logger(), "GetDataGram: socket NOT open");
    return ExitError;
  }

  addrlen = sizeof(recv_addr);

  tv.tv_sec = time_limit_;
  tv.tv_usec = 0;

  FD_ZERO(&rfds);
  FD_SET(socket_fd_, &rfds);

  if (select(socket_fd_ + 1, &rfds, nullptr, nullptr, &tv) > 0)
  {
    *length = recvfrom(
      socket_fd_,
      buffer,
      buffer_size,
      0,
      (struct sockaddr *)&recv_addr,
      &addrlen);

    return (*length > 0) ? ExitSuccess : ExitError;
  }
  else
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "GetDataGram timeout after %d seconds",
      time_limit_);
    return ExitError;
  }
}

}  // namespace sdkeli_ls_udp
