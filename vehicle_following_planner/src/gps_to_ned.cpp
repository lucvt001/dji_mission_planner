#include "vehicle_following_planner/gps_to_ned.hpp"

using GeoPoint = geographic_msgs::msg::GeoPoint;
using UTMPoint = geodesy::UTMPoint;

Point gps_to_ned(
    const NavSatFix &car_gps,
    const NavSatFix &drone_gps)
{
    GeoPoint car_geo_point;
    GeoPoint drone_geo_point;

    car_geo_point.latitude = car_gps.latitude;
    car_geo_point.longitude = car_gps.longitude;
    car_geo_point.altitude = car_gps.altitude;

    drone_geo_point.latitude = drone_gps.latitude;
    drone_geo_point.longitude = drone_gps.longitude;
    drone_geo_point.altitude = drone_gps.altitude;

    UTMPoint car_utm(car_geo_point);
    UTMPoint drone_utm(drone_geo_point);

    Point ned;
    ned.x = car_utm.northing - drone_utm.northing;
    ned.y = car_utm.easting - drone_utm.easting;
    ned.z = -(car_geo_point.altitude - drone_geo_point.altitude);

    return ned;
}
