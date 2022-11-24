#ifndef ROMEA_LOCALISATION_GPS_GPS_LOCALISATION_PLUGIN_PARAMETERS_HPP_
#define ROMEA_LOCALISATION_GPS_GPS_LOCALISATION_PLUGIN_PARAMETERS_HPP_

// ros
#include <rclcpp/node.hpp>

// eigen
#include <Eigen/Geometry>

// romea
#include <romea_core_gps/nmea/NMEAFixQuality.hpp>
#include <romea_core_common/geodesy/GeodeticCoordinates.hpp>

namespace romea
{

void declare_restamping(rclcpp::Node::SharedPtr node);
void declare_minimal_fix_quality(rclcpp::Node::SharedPtr node);
void declare_minimal_speed_over_ground(rclcpp::Node::SharedPtr node);
void declare_wgs84_anchor(rclcpp::Node::SharedPtr node);

bool get_restamping(rclcpp::Node::SharedPtr node);
FixQuality get_minimal_fix_quality(rclcpp::Node::SharedPtr node);
double get_minimal_speed_over_ground(rclcpp::Node::SharedPtr node);
std::optional<GeodeticCoordinates> get_wgs84_anchor(rclcpp::Node::SharedPtr node);

void declare_gps_gps_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_float_rtk_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_rtk_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_antenna_body_position(rclcpp::Node::SharedPtr node);

double get_gps_gps_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_float_rtk_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_rtk_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node);
Eigen::Vector3d get_gps_antenna_body_position(rclcpp::Node::SharedPtr node);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_GPS_GPS_LOCALISATION_PLUGIN_PARAMETERS_HPP_
