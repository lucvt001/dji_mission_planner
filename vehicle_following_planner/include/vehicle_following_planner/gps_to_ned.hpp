#ifndef GPS_TO_NED_HPP
#define GPS_TO_NED_HPP

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geodesy/utm.h>
#include <cmath>

using Point = geometry_msgs::msg::Point;
using NavSatFix = sensor_msgs::msg::NavSatFix;

// A helper function that takes in car_gps and drone_gps
// and returns the NED (North, East, Down) coordinates of the car position relative to the drone position
Point gps_to_ned(
    const NavSatFix &car_gps,
    const NavSatFix &drone_gps);

#endif // GPS_TO_NED_HPP
