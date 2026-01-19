#include "rclcpp/rclcpp.hpp"

#include "sdkeli_ls_common_udp.h"
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
  node->declare_parameter<double>("time_increment", -1.0);
  node->declare_parameter<std::string>("frame_id", "laser");

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
  std::unique_ptr<CSDKeliLsCommon> driver;

  rclcpp::Rate rate(200);  // fast loop, UDP driven

  while (rclcpp::ok())
  {
    driver.reset();

    driver = std::make_unique<CSDKeliLsCommonUdp>(
      hostname,
      port,
      time_limit,
      parser.get(),
      node);

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
