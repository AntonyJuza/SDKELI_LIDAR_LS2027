#ifndef SDKELI_LS_CONFIG_HPP_
#define SDKELI_LS_CONFIG_HPP_

namespace sdkeli_ls_udp
{

struct SDKeliLsConfig
{
  double angle_min = -2.35619;
  double angle_max = 2.35619;
  double range_min = 0.05;
  double range_max = 10.0;
  double scan_frequency = 23.0;
  double scan_time = 0.0;
  double time_offset = 0.0;

  bool intensity = false;
  bool inverted = false;
  bool debug_mode = false;

  int skip = 0;
};

}  // namespace sdkeli_ls_udp

#endif
