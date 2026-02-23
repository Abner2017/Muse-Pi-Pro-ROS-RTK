#ifndef FBRTK_DRIVER_H
#define FBRTK_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/FluidPressure.h>
#include <cstdint>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <vector>

namespace fbrtkros {

struct IMUData {
    double accel_x = 0, accel_y = 0, accel_z = 0;
    double gyro_x = 0, gyro_y = 0, gyro_z = 0;
    double angle_x = 0, angle_y = 0, angle_z = 0;
    double temperature = 0;
};

struct GPSData {
    double longitude = 0, latitude = 0, altitude = 0;
    double speed_knots = 0;  // Speed in knots
    double speed_ms = 0;     // Speed in m/s
    int satellites = 0;
    int fix_quality = 0; // 0: invalid, 1: GPS fix, 2: DGPS fix, 4: RTK fixed, 5: RTK float
    double hdop = 99.9;
    double pdop = 99.9;
    double vdop = 99.9;
};

struct RobotState {
    double in_voltage = 0;
    double battery_voltage = 0;
};

struct BaroData {
    double pressure = 0; // Pascals (after scale)
    double altitude = 0; // Meters (after scale)
};

struct WaypointData {
    double t_longitude = 0; // degrees
    double t_latitude = 0;  // degrees
    double t_altitude = 0;  // meters
    int16_t distance = 0;        // meters traveled (signed short)
    int16_t t_distance = 0;      // total meters (signed short)
};

class FBRTKDriver {
public:
    FBRTKDriver(ros::NodeHandle& nh);
    ~FBRTKDriver();
    
    void spin();
    bool initialize();
    void shutdown();

private:
    // Serial communication
    void readData();
    bool parseFrame(const uint8_t* data, size_t length);
    void processIMUData(const uint8_t* data);
    void processGPSData(const uint8_t* data);
    void processWaypointData(const uint8_t* data);
    void processBatteryData(const uint8_t* data);
    void processBaroData(const uint8_t* data);
    
    // Data conversion
    int16_t bytesToInt16(const uint8_t* data);
    int32_t bytesToInt32(const uint8_t* data);
    double bytesToFloat32(const uint8_t* data);
    
    // Message publishing
    void publishIMU();
    void publishGPS();
    void publishBatteryStatus();
    void publishBaro();
    void publishWaypoint();
    void jpegCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void lineTrackingCallback(const std_msgs::Int32::ConstPtr& msg);
    
    // Robot control callbacks
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void sendControlCommand(double linear_vel, double angular_vel);
    
    // ROS publishers
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher baro_pressure_pub_;
    ros::Publisher baro_altitude_pub_;
    ros::Publisher waypoint_fix_pub_;
    ros::Publisher waypoint_distance_pub_;
    ros::Publisher waypoint_total_distance_pub_;
    ros::Subscriber jpeg_sub_;
    ros::Subscriber line_tracking_sub_;
    
    // ROS subscribers
    ros::Subscriber cmd_vel_sub_;
    
    // Serial port
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::string port_;
    int baudrate_;
    
    // Data storage
    IMUData imu_data_;
    GPSData gps_data_;
    RobotState robot_state_;
    BaroData baro_data_;
    WaypointData waypoint_data_;
    
    // Configuration
    std::string imu_frame_id_;
    std::string gps_frame_id_;
    std::string base_frame_id_;
    std::string baro_frame_id_;
    
    // Control parameters
    double max_linear_vel_;
    double max_angular_vel_;
    double baro_pressure_scale_;
    double baro_altitude_scale_;
    int jpeg_max_size_kb_;
    
    // Buffer for serial data
    std::vector<uint8_t> buffer_;
    static const size_t MAX_BUFFER_SIZE = 1024;
};

} // namespace fbrtkros

#endif // FBRTK_DRIVER_H
