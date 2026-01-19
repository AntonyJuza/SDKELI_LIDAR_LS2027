#ifndef SDKELI_LS_COMMON_HPP_
#define SDKELI_LS_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include "parser_base.h"
#include "sdkeli_ls_config.h"
#include "sdkeli_ls_constants.h"

namespace sdkeli_ls_udp
{

class CSDKeliLsCommon
{
protected:
  struct dataSaveSt
  {
    unsigned int totaIndexlCount;
    unsigned char subPkgNum;
    unsigned char subPkgIndex;
    unsigned int rawDataLen;
    unsigned char sens_data[1500];
  };

public:
  explicit CSDKeliLsCommon(
    CParserBase * parser,
    const rclcpp::Node::SharedPtr & node);

  virtual ~CSDKeliLsCommon();

  int Init();
  int LoopOnce();
  int StopScanner();
  bool RebootDevice();

  void UpdateConfig(const SDKeliLsConfig & config);

protected:
  virtual int InitDevice() = 0;
  virtual int InitScanner();
  virtual int CloseDevice() = 0;

  virtual int SendDeviceReq(
    const uint8_t * req,
    size_t len) = 0;

  virtual int GetDataGram(
    uint8_t * buffer,
    int buffer_size,
    int * length) = 0;

  void ClearConnectFlag();

protected:
  rclcpp::Node::SharedPtr node_;
  CParserBase * parser_;
  SDKeliLsConfig config_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_publisher_;

  bool publish_data_{false};
  bool connect_flag_{false};

  uint8_t recv_buffer_[65536];
  uint8_t store_buffer_[65536];
  dataSaveSt data_save_[4];
};

}  // namespace sdkeli_ls_udp

#endif
