# 1 Overview #

This package is a gps plugin for vehicle localisation. It takes NMEA data coming from gps in order to provide a position and course angle with respect of a local tangent plane ENU reference frame.

# 2 Node #

### 2.1 Subscribed Topics ###


- gps/nmea_sentence : (nmea_msgs::Sentence)

  This topic is provided by nmea_topic_driver or romea_gps_driver. Only GGA and RMC frames are used to fix position and course angle

- vehicle_controler/odom : (nav_msgs::Odometry)

  This topic is provided by vehicle controllers. It will be used to decuded if vehicle moves in forward or reverse direction

### 2.2 Published Topics ###

- fix (romea_localisation_msgs::ObservationPositionStamped)

    Position and its covariance are deduced from GGA frame and published only if :
    - Horizontal Dilution of Precision is lower than 5
    - Number of satellites used to compute fix is than 6
    - Fix quatity is upper or equal to minimal fix quality define by user

- course (romea_localisation_msgs::ObservationCourseStamped)

    The course angle and its standard deviation are deduced from RMC frame.
    It is equal to &#960;/2-track angle when vehicle move in forward direction and 3&#960;/2-track overwise. Course angle is only published if speed over ground is upper than minimal speed define by user.  

### 2.3 Parameters ###

- ~minimal_fix_quality (integer, default=4)

  In order to only pusblished fix position when an acceptable accurracy is reached a minimal fix quality must be defined by user according experimentation requierements. Here is the list of useful fix qualities as a reminder :
    - gps fix :2
    - dgps fix : 3
    - rtk fix : 4
    - float rtk fix : 5

- ~minimal_speed_over_ground_to_use_track_angle (float, default=0.8)

  The track angle is reliable only when the vehicle moves fast enough. A minimal speed over ground must be defined by user in order to computed and published the course angle when this speed is reached by the vehicle.

- ~restamping (bool, default: false)

    If this parameter is set to true stamp of psotion and course messages is equal to computer current time else this stamp is equal gps nmea message stamp.  This paremeter will be used when gps data are coming from a remote master.


- ~gps/gps_fix_uere( double, default 3.0)

  User Equivalent Range Error for gps fix quality

- ~gps/dpgs_fix_uere( double, default 3.0)

  User Equivalent Range Error for dgps fix quality

- ~gps/float_rtk_fix_eure( double, default 3.0)

  User Equivalent Range Error for float rtk fix quality

- ~gps/rtk_fix_eure( double, default 3.0)

  User Equivalent Range Error for rtk fix quality

- ~gps/xyz

  Position of GPS receiver antenna in vehicle body reference frame

- ~wgs84_anchor ([latitude,longitude,altitude])

  Origin coordinates of the local tangent plan reference frame. If this parameter is not provided by user, it will be deduced from the first received GGA frame. In order to be user friendly latitude and longitude are defined in degrees.
