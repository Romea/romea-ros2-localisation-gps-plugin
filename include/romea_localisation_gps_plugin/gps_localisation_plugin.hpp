// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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
// #include <tf2_ros/static_transform_broadcaster.h>

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

class GPSLocalisationPlugin
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
  explicit GPSLocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_LOCALISATION_GPS_PLUGIN_PUBLIC
  virtual ~GPSLocalisationPlugin() = default;

  ROMEA_LOCALISATION_GPS_PLUGIN_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_plugin_();

  void init_course_publisher_();

  void init_position_publisher_();

  void init_diagnostic_publisher_();

  void init_nmea_subscriber_();

  void init_odom_subscriber_();

  void init_timer_();


  void process_nmea_(NmeaSentenceMsg::ConstSharedPtr msg);

  void process_odom_(OdometryMsg::ConstSharedPtr msg);

  void process_gga_(const NmeaSentenceMsg & msg);

  void process_rmc_(const NmeaSentenceMsg & msg);

  void process_gsv_(const NmeaSentenceMsg & msg);

  void publish_position_(const rclcpp::Time & stamp, const std::string & frame_id);

  void publish_course_(const rclcpp::Time & stamp, const std::string & frame_id);

  // void advertise_map_to_world_tf_();

  void timer_callback_();

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<romea::LocalisationGPSPlugin> plugin_;
  romea::ObservationPosition position_observation_;
  romea::ObservationCourse course_observation_;

  rclcpp::Subscription<NmeaSentenceMsg>::SharedPtr nmea_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<ObservationCourseStampedMsg>::SharedPtr course_pub_;
  rclcpp::Publisher<ObservationPosition2DStampedMsg>::SharedPtr position_pub_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // geometry_msgs::msg::TransformStamped tf_world_to_map_;
  // tf2_ros::StaticTransformBroadcaster tf_broadcaster_;

  bool restamping_;
};

}  // namespace romea

#endif  // ROMEA_LOCALISATION_GPS_PLUGIN__GPS_LOCALISATION_PLUGIN_HPP_
