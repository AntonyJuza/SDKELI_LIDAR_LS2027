#ifndef SDKELI_LS1207DE_PARSER_HPP_
#define SDKELI_LS1207DE_PARSER_HPP_

#include <string>

#include "parser_base.h"

namespace sdkeli_ls_udp
{

class CSDKeliLs1207DEParser : public CParserBase
{
public:
  CSDKeliLs1207DEParser();
  ~CSDKeliLs1207DEParser() override;

  int Parse(
    const uint8_t * data,
    size_t data_length,
    SDKeliLsConfig & config,
    sensor_msgs::msg::LaserScan & msg) override;

  void SetRangeMin(float min_range);
  void SetRangeMax(float max_range);
  void SetTimeIncrement(float time);
  void SetFrameId(const std::string & frame_id);

private:
  float range_min_ {0.0f};
  float range_max_ {0.0f};
  float time_increment_ {0.0f};
  std::string frame_id_;
};

}  // namespace sdkeli_ls_udp

#endif  // SDKELI_LS1207DE_PARSER_HPP_
