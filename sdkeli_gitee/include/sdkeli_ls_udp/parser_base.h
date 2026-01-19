#ifndef PARSER_BASE_HPP_
#define PARSER_BASE_HPP_

#include <cstddef>

#include "sdkeli_ls_config.h"
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sdkeli_ls_udp
{

enum ExitCode
{
  ExitSuccess = 0,
  ExitError   = 1,
  ExitFatal   = 2
};

class CParserBase
{
public:
  CParserBase() = default;
  virtual ~CParserBase() = default;

  virtual int Parse(
    const uint8_t * data,
    size_t data_length,
    SDKeliLsConfig & config,
    sensor_msgs::msg::LaserScan & msg) = 0;
};

}  // namespace sdkeli_ls_udp

#endif  // PARSER_BASE_HPP_
