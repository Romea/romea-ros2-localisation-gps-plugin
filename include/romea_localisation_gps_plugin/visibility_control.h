// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_GPS__VISIBILITY_CONTROL_H_
#define ROMEA_LOCALISATION_GPS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROMEA_LOCALISATION_GPS_EXPORT __attribute__ ((dllexport))
    #define ROMEA_LOCALISATION_GPS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROMEA_LOCALISATION_GPS_EXPORT __declspec(dllexport)
    #define ROMEA_LOCALISATION_GPS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROMEA_LOCALISATION_GPS_BUILDING_DLL
    #define ROMEA_LOCALISATION_GPS_PUBLIC ROMEA_LOCALISATION_GPS_EXPORT
  #else
    #define ROMEA_LOCALISATION_GPS_PUBLIC ROMEA_LOCALISATION_GPS_IMPORT
  #endif
  #define ROMEA_LOCALISATION_GPS_PUBLIC_TYPE ROMEA_LOCALISATION_GPS_PUBLIC
  #define ROMEA_LOCALISATION_GPS_LOCAL
#else
  #define ROMEA_LOCALISATION_GPS_EXPORT __attribute__ ((visibility("default")))
  #define ROMEA_LOCALISATION_GPS_IMPORT
  #if __GNUC__ >= 4
    #define ROMEA_LOCALISATION_GPS_PUBLIC __attribute__ ((visibility("default")))
    #define ROMEA_LOCALISATION_GPS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROMEA_LOCALISATION_GPS_PUBLIC
    #define ROMEA_LOCALISATION_GPS_LOCAL
  #endif
  #define ROMEA_LOCALISATION_GPS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROMEA_LOCALISATION_GPS__VISIBILITY_CONTROL_H_