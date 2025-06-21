// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

namespace
{

//-----------------------------------------------------------------------------
void declare_gps_receiver_parameters(std::shared_ptr<rclcpp::Node> node)
{
  romea::ros2::declare_gps_gps_fix_eure(node);
  romea::ros2::declare_gps_dgps_fix_eure(node);
  romea::ros2::declare_gps_float_rtk_fix_eure(node);
  romea::ros2::declare_gps_rtk_fix_eure(node);
  romea::ros2::declare_gps_simulation_fix_eure(node);
  romea::ros2::declare_gps_antenna_body_position(node);
}

//-----------------------------------------------------------------------------
std::unique_ptr<romea::core::GPSReceiver> make_gps_receiver(std::shared_ptr<rclcpp::Node> node)
{
  return std::make_unique<romea::core::GPSReceiver>(
    romea::ros2::get_gps_gps_fix_eure(node),
    romea::ros2::get_gps_dgps_fix_eure(node),
    romea::ros2::get_gps_float_rtk_fix_eure(node),
    romea::ros2::get_gps_rtk_fix_eure(node),
    romea::ros2::get_gps_simulation_fix_eure(node),
    romea::ros2::get_gps_antenna_body_position(node));
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void declare_plugin_parameters(std::shared_ptr<rclcpp::Node> node)
{
  romea::ros2::declare_restamping(node);
  romea::ros2::declare_wgs84_anchor(node);
  romea::ros2::declare_minimal_fix_quality(node);

  if constexpr (std::is_same_v<CorePlugin, romea::core::LocalisationSingleAntennaGPSPlugin>) {
    romea::ros2::declare_minimal_speed_over_ground(node);
  }
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
std::unique_ptr<CorePlugin> make_plugin(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (std::is_same_v<CorePlugin, romea::core::LocalisationSingleAntennaGPSPlugin>) {
    return std::make_unique<romea::core::LocalisationSingleAntennaGPSPlugin>(
      make_gps_receiver(node),
      romea::ros2::get_minimal_fix_quality(node),
      romea::ros2::get_minimal_speed_over_ground(node));
  } else {
    return std::make_unique<romea::core::LocalisationDualAntennaGPSPlugin>(
      make_gps_receiver(node), romea::ros2::get_minimal_fix_quality(node));
  }
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
bool make_position_observation(
  CorePlugin & plugin,
  const romea::core::Duration & stamp,
  const std::string & sentence,
  romea::core::ObservationPosition & position_observation)
{
  return plugin.processGGA(stamp, sentence, position_observation);
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
bool make_course_observation(
  CorePlugin & plugin,
  const romea::core::Duration & stamp,
  const std::string & sentence,
  romea::core::ObservationCourse & course_observation)
{
  if constexpr (std::is_same_v<CorePlugin, romea::core::LocalisationSingleAntennaGPSPlugin>) {
    return plugin.processRMC(stamp, sentence, course_observation);
  } else {
    return plugin.processHDT(stamp, sentence, course_observation);
  }
}

}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename CorePlugin>
GPSLocalisationPluginBase<CorePlugin>::GPSLocalisationPluginBase(const rclcpp::NodeOptions & options) // NOLINT
: node_(std::make_shared<rclcpp::Node>("gps_localisation_plugin", options)),
  plugin_(nullptr),
  position_observation_(),
  course_observation_(),
  nmea_sub_(nullptr),
  course_pub_(nullptr),
  position_pub_(nullptr),
  diagnostic_pub_(nullptr),
  timer_(),
  restamping_(false)
{
  declare_parameters_();
  init_plugin_();
  init_timer_();
  init_course_publisher_();
  init_position_publisher_();
  init_diagnostic_publisher_();
  init_odom_subscriber_();
  init_nmea_subscriber_();
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
GPSLocalisationPluginBase<CorePlugin>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::declare_parameters_()
{
  declare_plugin_parameters<CorePlugin>(node_);
  declare_gps_receiver_parameters(node_);
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::init_plugin_()
{
  plugin_ = make_plugin<CorePlugin>(node_);
  plugin_->setAnchor(get_wgs84_anchor(node_));
  restamping_ = get_parameter<bool>(node_, "restamping");
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::init_course_publisher_()
{
  course_pub_ = node_->create_publisher<ObservationCourseStampedMsg>(
    "course", sensor_data_qos());
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::init_position_publisher_()
{
  position_pub_ = node_->create_publisher<ObservationPosition2DStampedMsg>(
    "position", sensor_data_qos());
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::init_diagnostic_publisher_()
{
  diagnostic_pub_ =
    make_diagnostic_publisher<core::DiagnosticReport>(
    node_, node_->get_fully_qualified_name(), 1.0);
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::init_nmea_subscriber_()
{
  auto callback =
    std::bind(&GPSLocalisationPluginBase::process_nmea_, this, std::placeholders::_1);

  nmea_sub_ = node_->create_subscription<NmeaSentenceMsg>(
    "gps/nmea_sentence", best_effort(30), callback);
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::init_odom_subscriber_()
{
  auto callback = std::bind(&GPSLocalisationPluginBase::process_odom_, this, std::placeholders::_1);

  odom_sub_ = node_->create_subscription<OdometryMsg>(
    "vehicle_controller/odom", best_effort(10), callback);
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::init_timer_()
{
  auto callback = std::bind(&GPSLocalisationPluginBase::timer_callback_, this);
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), callback);
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::process_nmea_(NmeaSentenceMsg::ConstSharedPtr msg)
{
  // std::cout << " processNmea " << std::endl;
  // std::cout << msg->sentence << std::endl;

  switch (core::NMEAParsing::extractSentenceId(msg->sentence)) {
    case core::NMEAParsing::SentenceID::GGA:
      process_position_(*msg);
      break;
    case core::NMEAParsing::SentenceID::RMC:
      if constexpr (std::is_same_v<CorePlugin, core::LocalisationSingleAntennaGPSPlugin>) {
        process_course_(*msg);
      }
      break;
    case core::NMEAParsing::SentenceID::HDT:
      if constexpr (std::is_same_v<CorePlugin, core::LocalisationDualAntennaGPSPlugin>) {
        process_course_(*msg);
      }
      break;
    case core::NMEAParsing::SentenceID::GSV:
      process_satellites_view_(*msg);
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::process_odom_(OdometryMsg::ConstSharedPtr msg)
{
  if constexpr (std::is_same_v<CorePlugin, core::LocalisationSingleAntennaGPSPlugin>) {
    plugin_->processLinearSpeed(
      to_romea_duration(msg->header.stamp),
      msg->twist.twist.linear.x);
  }
}


//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::process_position_(const NmeaSentenceMsg & msg)
{
  // std::cout << " process GGA " << std::endl;
  auto stamp = restamping_ ? node_->get_clock()->now() :
    rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);

  if (make_position_observation(
      *plugin_, to_romea_duration(stamp), msg.sentence, position_observation_))
  {
    publish_position_(stamp, "map");
  }
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::process_course_(const NmeaSentenceMsg & msg)
{
  // std::cout << " process RMC " << std::endl;
  auto stamp = restamping_ ? node_->get_clock()->now() :
    rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);

  if (make_course_observation(
      *plugin_, to_romea_duration(stamp), msg.sentence, course_observation_))
  {
    publish_course_(stamp, msg.header.frame_id);
  }
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::process_satellites_view_(const NmeaSentenceMsg & msg)
{
  plugin_->processGSV(msg.sentence);
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::publish_position_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto position_msg = std::make_unique<ObservationPosition2DStampedMsg>();
  to_ros_msg(stamp, frame_id, position_observation_, *position_msg);
  position_pub_->publish(std::move(position_msg));
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::publish_course_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto course_msg = std::make_unique<ObservationCourseStampedMsg>();
  to_ros_msg(stamp, frame_id, course_observation_, *course_msg);
  course_pub_->publish(std::move(course_msg));
}

//-----------------------------------------------------------------------------
template<typename CorePlugin>
void GPSLocalisationPluginBase<CorePlugin>::timer_callback_()
{
  auto stamp = node_->get_clock()->now();
  diagnostic_pub_->publish(stamp, plugin_->makeDiagnosticReport(to_romea_duration(stamp)));
}


}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::SingleAntennaGPSLocalisationPlugin)
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::DualAntennaGPSLocalisationPlugin)
