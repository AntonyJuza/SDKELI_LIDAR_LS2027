#include "rclcpp/rclcpp.hpp"

#include "sdkeli_ls_common_udp.h"
#include "sdkeli_ls_config.h"
#include "sdkeli_ls1207de_parser.h"

using namespace sdkeli_ls_udp;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("sdkeli_ls1207de");

  /* Declare parameters */
  node->declare_parameter<std::string>("hostname", "");
  node->declare_parameter<int>("port", 2112);
  node->declare_parameter<int>("timelimit", 5);
  node->declare_parameter<bool>("subscribe_datagram", false);
  node->declare_parameter<int>("device_number", 0);

  node->declare_parameter<double>("range_min", 0.05);
  node->declare_parameter<double>("range_max", 10.0);
  node->declare_parameter<double>("angle_min", -2.35619);
  node->declare_parameter<double>("angle_max", 2.35619);
  node->declare_parameter<double>("scan_frequency", 23.0);
  node->declare_parameter<double>("scan_time", 0.0);
  node->declare_parameter<double>("time_increment", -1.0);
  node->declare_parameter<double>("time_offset", 0.0);
  node->declare_parameter<std::string>("frame_id", "laser");
  node->declare_parameter<bool>("publish_intensity", true);
  node->declare_parameter<bool>("inverted", false);
  node->declare_parameter<int>("skip", 0);

  /* Read parameters */
  std::string hostname;
  int port;
  int time_limit;
  int device_number;

  node->get_parameter("hostname", hostname);
  node->get_parameter("port", port);
  node->get_parameter("timelimit", time_limit);
  node->get_parameter("device_number", device_number);

  /* Create parser */
  auto parser = std::make_unique<CSDKeliLs1207DEParser>();
  parser->SetClock(node->get_clock());

  double param;
  std::string frame_id;

  if (node->get_parameter("range_min", param))
  {
    RCLCPP_INFO(node->get_logger(), "range_min: %.3f", param);
    parser->SetRangeMin(param);
  }

  if (node->get_parameter("range_max", param))
  {
    RCLCPP_INFO(node->get_logger(), "range_max: %.3f", param);
    parser->SetRangeMax(param);
  }

  if (node->get_parameter("time_increment", param))
  {
    RCLCPP_INFO(node->get_logger(), "time_increment: %.6f", param);
    parser->SetTimeIncrement(param);
  }

  if (node->get_parameter("frame_id", frame_id))
  {
    RCLCPP_INFO(node->get_logger(), "frame_id: %s", frame_id.c_str());
    parser->SetFrameId(frame_id);
  }

  /* Driver loop */
  int result = ExitError;
  auto driver = std::make_unique<CSDKeliLsCommonUdp>(
    hostname,
    port,
    time_limit,
    parser.get(),
    node);

  sdkeli_ls_udp::SDKeliLsConfig cfg;
  cfg.range_min = node->get_parameter("range_min").as_double();
  cfg.range_max = node->get_parameter("range_max").as_double();
  cfg.angle_min = node->get_parameter("angle_min").as_double();
  cfg.angle_max = node->get_parameter("angle_max").as_double();
  cfg.scan_frequency = node->get_parameter("scan_frequency").as_double();
  cfg.scan_time = node->get_parameter("scan_time").as_double();
  cfg.time_offset = node->get_parameter("time_offset").as_double();
  cfg.intensity = node->get_parameter("publish_intensity").as_bool();
  cfg.inverted = node->get_parameter("inverted").as_bool();
  cfg.skip = node->get_parameter("skip").as_int();
  RCLCPP_INFO(node->get_logger(), "angle_min: %.5f", cfg.angle_min);
  RCLCPP_INFO(node->get_logger(), "angle_max: %.5f", cfg.angle_max);
  RCLCPP_INFO(node->get_logger(), "scan_frequency: %.3f", cfg.scan_frequency);
  RCLCPP_INFO(node->get_logger(), "scan_time: %.6f", cfg.scan_time);
  RCLCPP_INFO(node->get_logger(), "time_offset: %.6f", cfg.time_offset);
  RCLCPP_INFO(node->get_logger(), "publish_intensity: %s", cfg.intensity ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "inverted: %s", cfg.inverted ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "skip: %d", cfg.skip);
  driver->UpdateConfig(cfg);

  rclcpp::Rate rate(200);  // fast loop, UDP driven

  while (rclcpp::ok())
  {
    result = driver->Init();

    while (rclcpp::ok() && result == ExitSuccess)
    {
      result = driver->LoopOnce();
      rate.sleep();
    }

    if (result == ExitFatal)
    {
      RCLCPP_FATAL(node->get_logger(), "Fatal driver error, exiting");
      break;
    }

    RCLCPP_WARN(node->get_logger(), "Driver disconnected, retrying...");
  }

  rclcpp::shutdown();
  return result;
}
