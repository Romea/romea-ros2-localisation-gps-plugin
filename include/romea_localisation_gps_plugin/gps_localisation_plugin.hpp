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

#ifndef ROMEA_LOCALISATION_GPS_PLUGIN__GPS_LOCALISATION_PLUGIN_HPP_
#define ROMEA_LOCALISATION_GPS_PLUGIN__GPS_LOCALISATION_PLUGIN_HPP_

// std
#include <string>
#include <memory>

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// romea
#include "romea_core_localisation_gps/LocalisationGPSPlugin.hpp"
#include "romea_localisation_utils/conversions/observation_position_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_course_conversions.hpp"
#include "romea_common_utils/conversions/diagnostic_conversions.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"


// local
#include "romea_localisation_gps_plugin/visibility_control.h"

namespace romea
{
namespace ros2
{

template<typename CorePlugin>
class GPSLocalisationPluginBase
{
public:
  using OdometryMsg = nav_msgs::msg::Odometry;
  using NmeaSentenceMsg = nmea_msgs::msg::Sentence;
  using ObservationCourseStampedMsg =
    romea_localisation_msgs::msg::ObservationCourseStamped;
  using ObservationPosition2DStampedMsg =
    romea_localisation_msgs::msg::ObservationPosition2DStamped;

public:
  ROMEA_LOCALISATION_GPS_PLUGIN_PUBLIC
  explicit GPSLocalisationPluginBase(const rclcpp::NodeOptions & options);

  ROMEA_LOCALISATION_GPS_PLUGIN_PUBLIC
  virtual ~GPSLocalisationPluginBase() = default;

  ROMEA_LOCALISATION_GPS_PLUGIN_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_plugin_();

  void init_nmea_subscriber_();

  void init_position_publisher_();

  void init_diagnostic_publisher_();

  void init_odom_subscriber_();

  void init_course_publisher_();

  void init_timer_();

  void process_nmea_(NmeaSentenceMsg::ConstSharedPtr msg);

  void process_odom_(OdometryMsg::ConstSharedPtr msg);

  void process_position_(const NmeaSentenceMsg & msg);

  void process_course_(const NmeaSentenceMsg & msg);

  void process_satellites_view_(const NmeaSentenceMsg & msg);

  void publish_position_(const rclcpp::Time & stamp, const std::string & frame_id);

  void publish_course_(const rclcpp::Time & stamp, const std::string & frame_id);

  void timer_callback_();

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CorePlugin> plugin_;
  core::ObservationPosition position_observation_;
  core::ObservationCourse course_observation_;

  rclcpp::Subscription<NmeaSentenceMsg>::SharedPtr nmea_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<ObservationCourseStampedMsg>::SharedPtr course_pub_;
  rclcpp::Publisher<ObservationPosition2DStampedMsg>::SharedPtr position_pub_;
  std::shared_ptr<StampedPublisherBase<core::DiagnosticReport>> diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool restamping_;
};

using SingleAntennaGPSLocalisationPlugin =
  GPSLocalisationPluginBase<core::LocalisationSingleAntennaGPSPlugin>;
using DualAntennaGPSLocalisationPlugin =
  GPSLocalisationPluginBase<core::LocalisationDualAntennaGPSPlugin>;

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_GPS_PLUGIN__GPS_LOCALISATION_PLUGIN_HPP_
