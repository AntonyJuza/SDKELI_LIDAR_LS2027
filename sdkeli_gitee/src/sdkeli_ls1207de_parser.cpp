#include "sdkeli_ls1207de_parser.h"
#include "sdkeli_ls_sensor_frame.h"

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstdio>
#include <limits>

namespace sdkeli_ls_udp
{

CSDKeliLs1207DEParser::CSDKeliLs1207DEParser()
: CParserBase(),
  range_min_(0.05f),
  range_max_(10.0f),
  time_increment_(0.0f),
  frame_id_("laser")
{
}

CSDKeliLs1207DEParser::~CSDKeliLs1207DEParser() = default;

int CSDKeliLs1207DEParser::Parse(
  const uint8_t * data,
  size_t data_length,
  SDKeliLsConfig & config,
  sensor_msgs::msg::LaserScan & msg)
{
  CSDKeliLsSensFrame sens_frame;

  if (!sens_frame.InitFromSensBuff(
        reinterpret_cast<const char *>(data),
        data_length))
  {
    return ExitSuccess;
  }

  const int data_count = sens_frame.GetSensDataCount();

  /* Header */
  msg.header.frame_id = frame_id_;

  /* 1: Scan time */
  const uint16_t scanning_freq = 1000 / 43 * 100;
  msg.scan_time = 1.0 / (scanning_freq / 100.0);

  /* 2: Time increment */
  time_increment_ = 0.000040;
  msg.time_increment = time_increment_;

  /* 3: Angle min */
  const int starting_angle = 0xFFF92230;
  msg.angle_min =
    (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;

  /* 4: Angle increment */
  const uint16_t angular_step_width = 0xD05;
  msg.angle_increment =
    (angular_step_width / 10000.0) / 180.0 * M_PI;

  /* 5: Angle max */
  msg.angle_max =
    msg.angle_min + (data_count - 1) * msg.angle_increment;

  /* Adjust index_min using config angle limits */
  int index_min = 0;
  while (msg.angle_min + msg.angle_increment < config.angle_min)
  {
    msg.angle_min += msg.angle_increment;
    index_min++;
  }

  /* Adjust index_max using config angle limits */
  int index_max = data_count - 1;
  while (msg.angle_max - msg.angle_increment > config.angle_max)
  {
    msg.angle_max -= msg.angle_increment;
    index_max--;
  }

  /* Fill ranges */
  const int output_size = index_max - index_min + 1;
  msg.ranges.assign(
    output_size,
    std::numeric_limits<float>::infinity());

  for (int j = index_min; j <= index_max; ++j)
  {
    const uint16_t range =
      sens_frame.GetSensDataOfIndex(j);

    const float meter_value = range / 100.0f;

    if (meter_value > range_min_ &&
        meter_value < range_max_)
    {
      msg.ranges[j - index_min] = meter_value;
    }
  }

  /* Intensities */
  if (config.intensity &&
      data_length == static_cast<size_t>(data_count * 4))
  {
    msg.intensities.resize(output_size);

    for (int j = index_min; j <= index_max; ++j)
    {
      uint16_t intensity =
        sens_frame.GetSensIntensityOfIndex(j);

      if (intensity > 55000)
      {
        intensity = 600;
      }
      else if (intensity > 5000)
      {
        intensity = 200 + (intensity - 5000) / 1200;
      }
      else
      {
        intensity = intensity / 25;
      }

      msg.intensities[j - index_min] = intensity;
    }
  }

  msg.range_min = range_min_;
  msg.range_max = range_max_;

  /* Timestamp handling (ROS2) */
  const double scan_duration =
    static_cast<double>(data_count) * msg.time_increment;

  rclcpp::Time stamp = rclcpp::Clock().now();
  stamp = stamp - rclcpp::Duration::from_seconds(scan_duration);
  stamp = stamp +
          rclcpp::Duration::from_seconds(
            static_cast<double>(index_min) * msg.time_increment);
  stamp = stamp +
          rclcpp::Duration::from_seconds(config.time_offset);

  msg.header.stamp = stamp;

  /* Consistency check */
  const double expected_time_increment =
    msg.scan_time * msg.angle_increment / (2.0 * M_PI);

  if (std::fabs(expected_time_increment - msg.time_increment) > 1e-5)
  {
    // Throttled logging must be done in node layer in ROS2
  }

  return ExitSuccess;
}

void CSDKeliLs1207DEParser::SetRangeMin(float min_range)
{
  range_min_ = min_range;
}

void CSDKeliLs1207DEParser::SetRangeMax(float max_range)
{
  range_max_ = max_range;
}

void CSDKeliLs1207DEParser::SetTimeIncrement(float time)
{
  time_increment_ = time;
}

void CSDKeliLs1207DEParser::SetFrameId(const std::string & frame_id)
{
  frame_id_ = frame_id;
}

}  // namespace sdkeli_ls_udp
