#include "sdkeli_ls_common.h"

#include <cstring>
#include <cstdio>

namespace sdkeli_ls_udp
{

CSDKeliLsCommon::CSDKeliLsCommon(
  CParserBase * parser,
  const rclcpp::Node::SharedPtr & node)
: node_(node),
  parser_(parser)
{
  std::memset(recv_buffer_, 0, 65536);

  scan_publisher_ =
    node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  node_->get_parameter("publish_datagram", publish_data_);

  if (publish_data_)
  {
    data_publisher_ =
      node_->create_publisher<std_msgs::msg::String>("datagram", 10);
  }

  connect_flag_ = false;
}

CSDKeliLsCommon::~CSDKeliLsCommon()
{
  RCLCPP_INFO(node_->get_logger(), "sdkeli_ls_udp driver exiting");
}

int CSDKeliLsCommon::Init()
{
  int result = InitDevice();
  if (result != ExitSuccess)
  {
    RCLCPP_FATAL(
      node_->get_logger(),
      "Failed to init device: %d", result);
    return result;
  }

  result = InitScanner();
  if (result != ExitSuccess)
  {
    RCLCPP_FATAL(
      node_->get_logger(),
      "Failed to init scanner: %d", result);
  }

  return result;
}

int CSDKeliLsCommon::InitScanner()
{
  SendDeviceReq(CMD_START_STREAM_DATA, sizeof(CMD_START_STREAM_DATA));
  return ExitSuccess;
}

int CSDKeliLsCommon::StopScanner()
{
  return SendDeviceReq(CMD_STOP_STREAM_DATA, sizeof(CMD_STOP_STREAM_DATA));
}

bool CSDKeliLsCommon::RebootDevice()
{
  return true;
}

int CSDKeliLsCommon::LoopOnce()
{
  static unsigned int last_frame_index = 0xFFFFFFFF;
  static unsigned int iteration_count = 0;

  unsigned int total_data_len = 0;
  int recv_len = 0;

  int result = GetDataGram(recv_buffer_, 65536, &recv_len);
  if (result != ExitSuccess)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to receive datagram");
    return ExitError;
  }

  if (!connect_flag_)
  {
    connect_flag_ = true;
    RCLCPP_INFO(node_->get_logger(), "SDKELI_LS connected");
  }

  unsigned char header[4] = {0xFA, 0x5A, 0xA5, 0xAA};

  if (std::memcmp(recv_buffer_, header, sizeof(header)) != 0)
  {
    std::memset(recv_buffer_, 0, RECV_BUFFER_SIZE);
    return ExitSuccess;
  }

  int raw_len =
    ((recv_buffer_[CMD_FRAME_HEADER_LENGTH_H] << 8) |
     recv_buffer_[CMD_FRAME_HEADER_LENGTH_L]) -
    (CMD_FRAME_DATA_START - CMD_FRAME_HEADER_CHECK_SUM);

  unsigned int frame_index =
    (recv_buffer_[CMD_FRAME_HEADER_TOTAL_INDEX_H] << 8) |
     recv_buffer_[CMD_FRAME_HEADER_TOTAL_INDEX_L];

  unsigned char sub_pkg_num =
    recv_buffer_[CMD_FRAME_HEADER_SUB_PKG_NUM];
  unsigned char sub_pkg_idx =
    recv_buffer_[CMD_FRAME_HEADER_SUB_INDEX];

  unsigned char checksum = 0;
  for (int i = 0;
       i < raw_len + (CMD_FRAME_DATA_START - CMD_FRAME_HEADER_TYPE);
       ++i)
  {
    checksum += recv_buffer_[CMD_FRAME_HEADER_TYPE + i];
  }

  if (checksum != recv_buffer_[CMD_FRAME_HEADER_CHECK_SUM])
  {
    return ExitSuccess;
  }

  if (sub_pkg_num < CMD_FRAME_MIN_SUB_PKG_NUM ||
      sub_pkg_num > CMD_FRAME_MAX_SUB_PKG_NUM)
  {
    return ExitSuccess;
  }

  data_save_[sub_pkg_idx].totaIndexlCount = frame_index;
  data_save_[sub_pkg_idx].subPkgNum = sub_pkg_num;
  data_save_[sub_pkg_idx].subPkgIndex = sub_pkg_idx;
  data_save_[sub_pkg_idx].rawDataLen = raw_len;

  std::memcpy(
    data_save_[sub_pkg_idx].sens_data,
    recv_buffer_ + CMD_FRAME_DATA_START,
    raw_len);

  bool incomplete = false;
  for (unsigned int i = 0; i < sub_pkg_num - 1; ++i)
  {
    if (data_save_[i].totaIndexlCount !=
          data_save_[i + 1].totaIndexlCount ||
        data_save_[i].subPkgIndex + 1 !=
          data_save_[i + 1].subPkgIndex)
    {
      incomplete = true;
      break;
    }
  }

  if (incomplete)
  {
    return ExitSuccess;
  }

  for (unsigned int i = 0; i < sub_pkg_num; ++i)
  {
    std::memcpy(
      store_buffer_ + total_data_len,
      data_save_[i].sens_data,
      data_save_[i].rawDataLen);
    total_data_len += data_save_[i].rawDataLen;
  }

  if (frame_index != last_frame_index + 1 && frame_index != 0)
  {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Frame out of order: %u (last %u)",
      frame_index, last_frame_index);
  }

  last_frame_index = frame_index;

  if (iteration_count++ % (config_.skip + 1) != 0)
  {
    return ExitSuccess;
  }

  sensor_msgs::msg::LaserScan msg;

  int success =
    parser_->Parse(
      store_buffer_,
      total_data_len,
      config_,
      msg);

  if (success == ExitSuccess)
  {
    scan_publisher_->publish(msg);
  }

  std::memset(store_buffer_, 0, 65536);
  return ExitSuccess;
}

void CSDKeliLsCommon::UpdateConfig(const SDKeliLsConfig & new_config)
{
  config_ = new_config;
}

void CSDKeliLsCommon::ClearConnectFlag()
{
  connect_flag_ = false;
}

}  // namespace sdkeli_ls_udp
