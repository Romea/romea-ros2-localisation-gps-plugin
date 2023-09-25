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
#include <string>
#include <limits>

// romea ros
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"
#include "romea_common_utils/params/geodesy_parameters.hpp"

// local
#include "romea_localisation_gps_plugin/gps_localisation_plugin_parameters.hpp"

namespace
{

const double DEFAULT_MINIMAL_SPEED_OVER_GROUND = 0.8;
// const double DEFAULT_MINIMAL_CONSTELLATION_RELIABILITY = 0.8;
const romea::FixQuality DEFAULT_MINIMAL_FIX_QUALITY = romea::FixQuality::RTK_FIX;

const char restamping_param_name[] = "restamping";
const char minimal_fix_quality_param_name[] = "minimal_fix_quality";
const char minimal_speed_over_ground_param_name[] = "minimal_speed_over_ground";
const char wgs84_anchor_param_name[] = "wgs84_anchor";

// const char gps_fix_uere_param_name[] = "gps.gps_fix_uere";
// const char dgps_fix_uere_param_name[] = "gps.dgps_fix_uere";
// const char float_rtk_fix_uere_param_name[] = "gps.float_rtk_fix_uere";
// const char rtk_fix_uere_param_name[] = "gps.rtk_fix_uere";
// const char simulation_fix_uere_param_name[] = "gps.simulation_fix_uere";

}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
void declare_restamping(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<bool>(node, restamping_param_name, false);
}

//----------------------------------------------------------------------------
void declare_minimal_fix_quality(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<int>(
    node, minimal_fix_quality_param_name,
    static_cast<int>(DEFAULT_MINIMAL_FIX_QUALITY));
}

//----------------------------------------------------------------------------
void declare_minimal_speed_over_ground(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<double>(
    node, minimal_speed_over_ground_param_name,
    DEFAULT_MINIMAL_SPEED_OVER_GROUND);
}

//----------------------------------------------------------------------------
void declare_wgs84_anchor(rclcpp::Node::SharedPtr node)
{
  declare_geodetic_coordinates_parameter(node, wgs84_anchor_param_name);
}

//----------------------------------------------------------------------------
bool get_restamping(rclcpp::Node::SharedPtr node)
{
  return get_parameter<bool>(node, restamping_param_name);
}

//----------------------------------------------------------------------------
FixQuality get_minimal_fix_quality(rclcpp::Node::SharedPtr node)
{
  return static_cast<FixQuality>(get_parameter<int>(node, minimal_fix_quality_param_name));
}

//----------------------------------------------------------------------------
double get_minimal_speed_over_ground(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, minimal_speed_over_ground_param_name);
}

//----------------------------------------------------------------------------
GeodeticCoordinates get_wgs84_anchor(rclcpp::Node::SharedPtr node)
{
  return get_geodetic_coordinates_parameter(node, wgs84_anchor_param_name);
}


// //-----------------------------------------------------------------------------
// void declare_gps_gps_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   declare_parameter<double>(node, gps_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// void declare_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   declare_parameter<double>(node, dgps_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// void declare_gps_float_rtk_gps_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   declare_parameter<double>(node, float_rtk_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// void declare_gps_rtk_gps_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   declare_parameter<double>(node, rtk_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// void declare_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   declare_parameter<double>(node, simulation_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// void declare_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return declare_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
// }

// //-----------------------------------------------------------------------------
// double get_gps_gps_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   return get_parameter<double>(node, gps_fix_uere_param_name);
// }


// //-----------------------------------------------------------------------------
// double get_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   return get_parameter<double>(node, dgps_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// double get_gps_float_rtk_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   return get_parameter<double>(node, float_rtk_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// double get_gps_rtk_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   return get_parameter<double>(node, rtk_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// double get_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node)
// {
//   return get_parameter<double>(node, simulation_fix_uere_param_name);
// }

// //-----------------------------------------------------------------------------
// Eigen::Vector3d get_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return get_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
// }

}  // namespace romea
