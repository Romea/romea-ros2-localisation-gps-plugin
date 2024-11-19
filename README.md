# romea_localisation_gps_plugin

This package is a gps plugin for robot localisation. It takes NMEA data coming from gps in order to provide a position and course angle observations with respect of a local tangent plane ENU reference frame.

## Plugin node description ##

#### 1) Subscribed Topics ####


- gps/nmea_sentence : (nmea_msgs::msg::Sentence)

  This topic is provided by nmea_topic_driver or romea_gps_driver. Only GGA and RMC frames are used to fix position and course angle

- vehicle_controler/odom : (nav_msgs::msg::Odometry)

  This topic is provided by vehicle controllers. It will be used to decuded if vehicle moves in forward or reverse direction

#### 2) Published Topics ####

- position (romea_localisation_msgs::msg::ObservationPositionStamped)

    Position and its covariance are deduced from GGA frame and published only if :
    - Horizontal Dilution of Precision is lower than 5
    - Number of satellites used to compute fix is than 6
    - Fix quatity is upper or equal to minimal fix quality define by user

- course (romea_localisation_msgs::msg::ObservationCourseStamped)

    The course angle and its standard deviation are deduced from RMC frame.
    It is equal to &#960;/2-track angle when vehicle move in forward direction and 3&#960;/2-track overwise. Course angle is only published if speed over ground is upper than minimal speed define by user.  

#### 3) Parameters ####

- ~minimal_fix_quality (integer, default=4)

  In order to only pusblished fix position when an acceptable accurracy is reached a minimal fix quality must be defined by user according experimentation requierements. Here is the list of useful fix qualities as a reminder :
    - gps fix :2
    - dgps fix : 3
    - rtk fix : 4
    - float rtk fix : 5

- ~minimal_speed_over_ground_to_use_track_angle (float, default=0.8)

  The track angle is reliable only when the vehicle moves fast enough. A minimal speed over ground must be defined by user in order to computed and published the course angle when this speed is reached by the vehicle.

- ~restamping (bool, default: false)

  If this parameter is set to true stamp of position and course messages is equal to computer current time else this stamp is equal gps nmea message stamp.  This paremeter will be used when gps data are coming from a remote master.


- ~gps.gps_fix_uere( double, default 3.0)

  User Equivalent Range Error for gps fix quality

- ~gps.dpgs_fix_uere( double, default 3.0)

  User Equivalent Range Error for dgps fix quality

- ~gps.float_rtk_fix_eure( double, default 3.0)

  User Equivalent Range Error for float rtk fix quality

- ~gps.rtk_fix_eure( double, default 3.0)

  User Equivalent Range Error for rtk fix quality

- ~gps.xyz

  Position of GPS receiver antenna in localisation body reference frame (usually base_footprint_link)

- ~wgs84_anchor.latitude 
  
  Reference latitude in degree

- ~wgs84_anchor.longitude 
  
  Reference longitude in degree

- ~wgs84_anchor.altidue 
  
  Reference altitude in meter


## **Contributing**

If you'd like to contribute to this project, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to yo

## License

This project is released under the Apache License 2.0. See the LICENSE file for details.

### Authors

 romea_localisation_gps_plugin project was developed by **Jean Laneurit** in the context of BaudetRob2 ANR project.

### Contact

If you have any questions or comments about romea_localisation_gps_plugin project, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 
