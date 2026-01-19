#ifndef SDKELI_LS_COMMON_UDP_HPP_
#define SDKELI_LS_COMMON_UDP_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <netinet/in.h>

#include <rclcpp/rclcpp.hpp>
#include "sdkeli_ls_common.h"

namespace sdkeli_ls_udp
{

class CSDKeliLsCommonUdp : public CSDKeliLsCommon
{
public:
  CSDKeliLsCommonUdp(
    const std::string & hostname,
    int port,
    int & timelimit,
    CParserBase * parser,
    const rclcpp::Node::SharedPtr & node);

  virtual ~CSDKeliLsCommonUdp();

protected:
  int InitDevice() override;
  int CloseDevice() override;

  int SendDeviceReq(
    const uint8_t * req,
    size_t len) override;

  int GetDataGram(
    uint8_t * buffer,
    int buffer_size,
    int * length) override;

  int SendUdpData2Device(char * buf, int length);

  void send_start();
  void send_stop();

private:
  int socket_fd_ {-1};

  std::string host_name_;
  int mPort;
  int         time_limit_;

  struct sockaddr_in remote_addr_ {};
};

}  // namespace sdkeli_ls_udp

#endif  // SDKELI_LS_COMMON_UDP_HPP_
