// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>
#include <utility>

// romea ros
#include "romea_localisation_gps_plugin/gps_localisation_plugin_parameters.hpp"
#include "romea_localisation_gps_plugin/gps_localisation_plugin.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_common_utils/conversions/transform_conversions.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/geodesy_parameters.hpp"
#include "romea_common_utils/qos.hpp"
#include "romea_gps_utils/gps_parameters.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GPSLocalisationPlugin::GPSLocalisationPlugin(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("gps_localisation_plugin", options)),
  plugin_(nullptr),
  position_observation_(),
  course_observation_(),
  nmea_sub_(nullptr),
  odom_sub_(nullptr),
  course_pub_(nullptr),
  position_pub_(nullptr),
  diagnostic_pub_(nullptr),
  timer_(),
  tf_world_to_map_(),
  tf_broadcaster_(node_),
  restamping_(false)
{
  declare_parameters_();
  init_plugin_();
  init_timer_();
  init_course_publisher_();
  init_position_publisher_();
  init_diagnostic_publisher_();
  init_nmea_subscriber_();
  init_odom_subscriber_();
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
GPSLocalisationPlugin::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::declare_parameters_()
{
  declare_gps_gps_fix_eure(node_);
  declare_gps_dgps_fix_eure(node_);
  declare_gps_float_rtk_fix_eure(node_);
  declare_gps_rtk_fix_eure(node_);
  declare_gps_simulation_fix_eure(node_);
  declare_gps_antenna_body_position(node_);

  declare_restamping(node_);
  declare_wgs84_anchor(node_);
  declare_minimal_fix_quality(node_);
  declare_minimal_speed_over_ground(node_);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::init_course_publisher_()
{
  course_pub_ = node_->create_publisher<ObservationCourseStampedMsg>(
    "course", sensor_data_qos());
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::init_position_publisher_()
{
  position_pub_ = node_->create_publisher<ObservationPosition2DStampedMsg>(
    "position", sensor_data_qos());
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::init_diagnostic_publisher_()
{
  diagnostic_pub_ = make_diagnostic_publisher<DiagnosticReport>(node_, node_->get_name(), 1.0);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::init_nmea_subscriber_()
{
  auto callback = std::bind(&GPSLocalisationPlugin::process_nmea_, this, std::placeholders::_1);

  nmea_sub_ = node_->create_subscription<NmeaSentenceMsg>(
    "gps/nmea_sentence", best_effort(10), callback);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::init_odom_subscriber_()
{
  auto callback = std::bind(&GPSLocalisationPlugin::process_odom_, this, std::placeholders::_1);

  odom_sub_ = node_->create_subscription<OdometryMsg>(
    "vehicle_controller/odom", best_effort(10), callback);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::init_timer_()
{
  auto callback = std::bind(&GPSLocalisationPlugin::timer_callback_, this);
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), callback);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::init_plugin_()
{
  auto gps = std::make_unique<GPSReceiver>(
    get_gps_gps_fix_eure(node_),
    get_gps_dgps_fix_eure(node_),
    get_gps_float_rtk_fix_eure(node_),
    get_gps_rtk_fix_eure(node_),
    get_gps_simulation_fix_eure(node_),
    get_gps_antenna_body_position(node_));

  plugin_ = std::make_unique<LocalisationGPSPlugin>(
    std::move(gps),
    get_minimal_fix_quality(node_),
    get_minimal_speed_over_ground(node_));

  auto wgs84_anchor = get_wgs84_anchor(node_);
  if (wgs84_anchor.has_value()) {
    plugin_->setAnchor(wgs84_anchor.value());
    advertise_map_to_world_tf_();
  }

  restamping_ = get_parameter<bool>(node_, "restamping");
}


//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::advertise_map_to_world_tf_()
{
  tf_world_to_map_.header.frame_id = "world";
  tf_world_to_map_.child_frame_id = "map";
  tf_world_to_map_.header.stamp = node_->get_clock()->now();

  to_ros_transform_msg(
    plugin_->getENUConverter().getEnuToEcefTransform(),
    tf_world_to_map_.transform);

  tf_broadcaster_.sendTransform(tf_world_to_map_);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::process_odom_(OdometryMsg::ConstSharedPtr msg)
{
  plugin_->processLinearSpeed(
    to_romea_duration(msg->header.stamp),
    msg->twist.twist.linear.x);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::process_gga_(const NmeaSentenceMsg & msg)
{
  // std::cout << " process GGA " << std::endl;
  auto stamp = restamping_ ? node_->get_clock()->now() :
    rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);

  if (plugin_->processGGA(
      to_romea_duration(stamp),
      msg.sentence,
      position_observation_))
  {
    publish_position_(stamp, "map");
    advertise_map_to_world_tf_();
  }
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::publish_position_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto position_msg = std::make_unique<ObservationPosition2DStampedMsg>();
  to_ros_msg(stamp, frame_id, position_observation_, *position_msg);
  position_pub_->publish(std::move(position_msg));
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::process_rmc_(const NmeaSentenceMsg & msg)
{
  // std::cout << " process RMC " << std::endl;
  auto stamp = restamping_ ? node_->get_clock()->now() :
    rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);

  if (plugin_->processRMC(
      to_romea_duration(stamp),
      msg.sentence,
      course_observation_))
  {
    publish_course_(stamp, msg.header.frame_id);
  }
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::publish_course_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto course_msg = std::make_unique<ObservationCourseStampedMsg>();
  to_ros_msg(stamp, frame_id, course_observation_, *course_msg);
  course_pub_->publish(std::move(course_msg));
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::process_gsv_(const NmeaSentenceMsg & msg)
{
  plugin_->processGSV(msg.sentence);
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::process_nmea_(NmeaSentenceMsg::ConstSharedPtr msg)
{
  // std::cout << " processNmea " << std::endl;
  // std::cout << msg->sentence << std::endl;

  switch (NMEAParsing::extractSentenceId(msg->sentence)) {
    case NMEAParsing::SentenceID::GGA:
      process_gga_(*msg);
      break;
    case NMEAParsing::SentenceID::RMC:
      process_rmc_(*msg);
      break;
    case NMEAParsing::SentenceID::GSV:
      process_gsv_(*msg);
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
void GPSLocalisationPlugin::timer_callback_()
{
  auto stamp = node_->get_clock()->now();
  diagnostic_pub_->publish(stamp, plugin_->makeDiagnosticReport(to_romea_duration(stamp)));
}

}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::GPSLocalisationPlugin)
