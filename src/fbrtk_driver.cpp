#include "fbrtkros/fbrtk_driver.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

namespace fbrtkros {

FBRTKDriver::FBRTKDriver(ros::NodeHandle& nh) :
    nh_(nh),
    io_(),
    serial_(io_),
    max_linear_vel_(1.0),
    max_angular_vel_(1.0)
{
    // Read parameters - fixed port for FB-RTK device
    nh_.param<std::string>("port", port_, "/dev/ttyACM2");
    nh_.param<int>("baudrate", baudrate_, 115200);
    nh_.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");
    nh_.param<std::string>("gps_frame_id", gps_frame_id_, "gps_link");
    nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    nh_.param<std::string>("baro_frame_id", baro_frame_id_, "baro_link");
    nh_.param<double>("max_linear_velocity", max_linear_vel_, 1.0);
    nh_.param<double>("max_angular_velocity", max_angular_vel_, 1.0);
    nh_.param<double>("baro_pressure_scale", baro_pressure_scale_, 1.0); // default: data already in Pa
    nh_.param<double>("baro_altitude_scale", baro_altitude_scale_, 0.01); // default: raw cm -> m
    nh_.param<int>("jpeg_max_size_kb", jpeg_max_size_kb_, 100); // JPEG max size default 100KB
    
    // Initialize publishers
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/fbrtk/imu/data", 10);
    gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/fbrtk/gps/fix", 10);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/fbrtk/velocity", 10);
    battery_pub_ = nh_.advertise<std_msgs::Float32>("/fbrtk/battery_voltage", 10);
    baro_pressure_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("/fbrtk/baro/pressure", 10);
    baro_altitude_pub_ = nh_.advertise<std_msgs::Float32>("/fbrtk/baro/altitude", 10);
    // Standard message outputs for compatibility
    waypoint_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/fbrtk/waypoint/fix", 10);
    waypoint_distance_pub_ = nh_.advertise<std_msgs::Int16>("/fbrtk/waypoint/distance", 10);
    waypoint_total_distance_pub_ = nh_.advertise<std_msgs::Int16>("/fbrtk/waypoint/total_distance", 10);
    
    // Initialize subscribers for robot control
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &FBRTKDriver::cmdVelCallback, this);
    jpeg_sub_ = nh_.subscribe("/fbrtk/jpeg", 1, &FBRTKDriver::jpegCallback, this);
    line_tracking_sub_ = nh_.subscribe("/fbrtk/line_tracking", 1, &FBRTKDriver::lineTrackingCallback, this);
    
    // Reserve buffer space
    buffer_.reserve(MAX_BUFFER_SIZE);
    
    ROS_INFO("FB-RTK Driver with integrated control initialized");
}

FBRTKDriver::~FBRTKDriver() {
    shutdown();
}

bool FBRTKDriver::initialize() {
    // Try to open the serial port, retrying every 1 second until success or ROS shuts down.
    while (ros::ok()) {
        try {
            // Open serial port
            serial_.open(port_);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            ROS_INFO("Serial port %s opened at %d baud", port_.c_str(), baudrate_);
            return true;
        }
        catch (const std::exception& e) {
            // Ensure closed state before retrying
            if (serial_.is_open()) {
                try { serial_.close(); } catch (...) {}
            }
            ROS_ERROR("Failed to open serial port %s: %s. Retrying in 1 second...", port_.c_str(), e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // ros is shutting down
    return false;
}

void FBRTKDriver::shutdown() {
    if (serial_.is_open()) {
        serial_.close();
    }
}

void FBRTKDriver::spin() {
    while (ros::ok()) {
        readData(); // Blocking read, no rate limiting needed
        ros::spinOnce();
    }
}

void FBRTKDriver::readData() {
    try {
        if (!serial_.is_open()) {
            return;
        }
        
        uint8_t byte;
        // Blocking read - wait for data to arrive
        size_t n = boost::asio::read(serial_, boost::asio::buffer(&byte, 1));
        
        if (n != 1) {
            return;
        }
        
        buffer_.push_back(byte);
        
        // Keep buffer size manageable
        if (buffer_.size() > MAX_BUFFER_SIZE) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + MAX_BUFFER_SIZE/2);
        }
        
        // Look for frame start (0x55)
        if (buffer_.size() >= 2 && buffer_[0] != 0x55) {
            buffer_.erase(buffer_.begin());
            return;
        }
        
        // Try to parse complete frames
        const size_t FRAME_LEN = 11; // Standard frame length
        while (buffer_.size() >= FRAME_LEN) {
            if (parseFrame(buffer_.data(), FRAME_LEN)) {
                buffer_.erase(buffer_.begin(), buffer_.begin() + FRAME_LEN);
            } else {
                buffer_.erase(buffer_.begin());
                break;
            }
        }
    }
    catch (const std::exception& e) {
        ROS_WARN("Serial read error: %s", e.what());
    }
}

bool FBRTKDriver::parseFrame(const uint8_t* data, size_t length) {
    if (length < 11 || data[0] != 0x55) {
        return false;
    }
    
    uint8_t frame_type = data[1];
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 0; i < 10; ++i) {
        checksum += data[i];
    }
    
    // Skip checksum validation for now (can be enabled if needed)
    // if (checksum != data[10]) {
    //     return false;
    // }
    
    switch (frame_type) {
        case 0x50: // Time frame
            return true;
            
        case 0x51: // Acceleration data
            processIMUData(data);
            return true;
            
        case 0x52: // Angular velocity data
            processIMUData(data);
            return true;
            
        case 0x53: // Angle data
            processIMUData(data);
            publishIMU(); // Publish after Angle data
            return true;

        case 0x54: // Angular velocity raw data
            return true;

        case 0x55: // Battery voltage
            processBatteryData(data);
            publishBatteryStatus();
            return true;
        case 0x56: // Barometer: pressure and altitude
            processBaroData(data);
            publishBaro();
            return true;
            
        case 0x57: // GPS position
            processGPSData(data);
            return true;
            
        case 0x58: // GPS altitude and speed
            processGPSData(data);
            publishGPS(); // Publish after altitude data
            return true;

        case 0x59: // Waypoint longitude/latitude
            processWaypointData(data);
            return true;

        case 0x5A: // Waypoint altitude and progress
            processWaypointData(data);
            publishWaypoint(); // Publish after altitude/progress data
            return true;
            
        default:
            return false;
    }
}

void FBRTKDriver::processIMUData(const uint8_t* data) {
    uint8_t frame_type = data[1];
    
    switch (frame_type) {
        case 0x51: // Acceleration
            imu_data_.accel_x = bytesToInt16(&data[2]) / 32768.0 * 2.0 * 9.81; // ±2g range
            imu_data_.accel_y = bytesToInt16(&data[4]) / 32768.0 * 2.0 * 9.81;
            imu_data_.accel_z = bytesToInt16(&data[6]) / 32768.0 * 2.0 * 9.81;
            imu_data_.temperature = bytesToInt16(&data[8]) / 100.0; // Temperature in Celsius
            break;
            
        case 0x52: // Angular velocity
            imu_data_.gyro_x = bytesToInt16(&data[2]) / 32768.0 * 250.0 * M_PI / 180.0; // ±250°/s range
            imu_data_.gyro_y = bytesToInt16(&data[4]) / 32768.0 * 250.0 * M_PI / 180.0;
            imu_data_.gyro_z = bytesToInt16(&data[6]) / 32768.0 * 250.0 * M_PI / 180.0;
            break;
            
        case 0x53: // Angle
            imu_data_.angle_x = bytesToInt16(&data[2]) / 32768.0 * 180.0; // Roll
            imu_data_.angle_y = bytesToInt16(&data[4]) / 32768.0 * 180.0; // Pitch
            imu_data_.angle_z = bytesToInt16(&data[6]) / 32768.0 * 180.0; // Yaw
            break;
    }
}

void FBRTKDriver::processBaroData(const uint8_t* data) {
    // According to user hint: output_wit(0x56, p&0xFFFF, p>>16, alt&0xFFFF, alt>>16)
    // So data[2..3] = low 16 bits of pressure, data[4..5] = high 16 bits of pressure
    //     data[6..7] = low 16 bits of altitude, data[8..9] = high 16 bits of altitude
    // Reconstruct 32-bit signed integers
    uint32_t p_u = (static_cast<uint32_t>(static_cast<uint16_t>(bytesToInt16(&data[4]))) << 16)
                 | static_cast<uint32_t>(static_cast<uint16_t>(bytesToInt16(&data[2])));
    uint32_t alt_u = (static_cast<uint32_t>(static_cast<uint16_t>(bytesToInt16(&data[8]))) << 16)
                   | static_cast<uint32_t>(static_cast<uint16_t>(bytesToInt16(&data[6])));
    int32_t p_raw = static_cast<int32_t>(p_u);
    int32_t alt_raw = static_cast<int32_t>(alt_u);

    // Apply default scaling: assume pressure already in Pa; altitude raw in cm unless configured
    baro_data_.pressure = static_cast<double>(p_raw) * baro_pressure_scale_;
    baro_data_.altitude = static_cast<double>(alt_raw) * baro_altitude_scale_;
}

void FBRTKDriver::processGPSData(const uint8_t* data) {
    uint8_t frame_type = data[1];
    
    switch (frame_type) {
        case 0x57: // Position
            gps_data_.longitude = bytesToInt32(&data[2]) / 10000000.0; // Degrees
            gps_data_.latitude = bytesToInt32(&data[6]) / 10000000.0;  // Degrees
            break;
            
        case 0x58: // Altitude, status and speed
            gps_data_.altitude = bytesToInt32(&data[2]) / 1000.0; // Meters
            gps_data_.satellites = data[6];
            gps_data_.fix_quality = data[7];
            // Speed in knots from data[8] and data[9] (short type, divide by 100)
            gps_data_.speed_knots = bytesToInt16(&data[8]) / 100.0; // knots
            // Convert knots to m/s (1 knot = 0.514444 m/s)
            gps_data_.speed_ms = gps_data_.speed_knots * 0.514444;
            // ROS_INFO("GPS Speed: %.2f Knots, %.2f m/s, Satellites: %d, Fix Quality: %d",
            //          gps_data_.speed_knots, gps_data_.speed_ms, gps_data_.satellites, gps_data_.fix_quality);
            break;
    }
}

void FBRTKDriver::processWaypointData(const uint8_t* data) {
    uint8_t frame_type = data[1];

    switch (frame_type) {
        case 0x59: // Waypoint longitude/latitude
            waypoint_data_.t_longitude = bytesToInt32(&data[2]) / 10000000.0; // Degrees
            waypoint_data_.t_latitude  = bytesToInt32(&data[6]) / 10000000.0; // Degrees
            break;

        case 0x5A: // Waypoint altitude and progress
            waypoint_data_.t_altitude = bytesToInt32(&data[2]) / 1000.0; // Meters
            waypoint_data_.distance   = bytesToInt16(&data[6]);
            waypoint_data_.t_distance = bytesToInt16(&data[8]);
            break;
    }
}

void FBRTKDriver::processBatteryData(const uint8_t* data) {
    robot_state_.battery_voltage = bytesToInt16(&data[4]) / 100.0;   // V
    robot_state_.in_voltage = bytesToFloat32(&data[6]); // In voltage
    // ROS_INFO("Battery Voltage: %.2f V, In Voltage: %.2f V", 
    //          robot_state_.battery_voltage, robot_state_.in_voltage);
}

void FBRTKDriver::publishBatteryStatus() {
    // Publish battery voltage
    std_msgs::Float32 battery_msg;
    battery_msg.data = robot_state_.battery_voltage;
    battery_pub_.publish(battery_msg);
    
    // Log battery status if low
    if (robot_state_.battery_voltage < 3.5 && robot_state_.battery_voltage > 0) {
        ROS_WARN("Low battery voltage: %.2fV", robot_state_.battery_voltage);
    }
}

void FBRTKDriver::publishIMU() {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = imu_frame_id_;
    
    // Linear acceleration
    imu_msg.linear_acceleration.x = imu_data_.accel_x;
    imu_msg.linear_acceleration.y = imu_data_.accel_y;
    imu_msg.linear_acceleration.z = imu_data_.accel_z;
    
    // Angular velocity
    imu_msg.angular_velocity.x = imu_data_.gyro_x;
    imu_msg.angular_velocity.y = imu_data_.gyro_y;
    imu_msg.angular_velocity.z = imu_data_.gyro_z;
    
    // Orientation from Euler angles
    tf2::Quaternion q;
    q.setRPY(imu_data_.angle_x * M_PI / 180.0, 
             imu_data_.angle_y * M_PI / 180.0, 
             imu_data_.angle_z * M_PI / 180.0);
    imu_msg.orientation = tf2::toMsg(q);
    
    // Covariance matrices (diagonal, adjust values as needed)
    for (int i = 0; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
    
    imu_pub_.publish(imu_msg);
}

void FBRTKDriver::publishBaro() {
    ros::Time stamp = ros::Time::now();

    sensor_msgs::FluidPressure pmsg;
    pmsg.header.stamp = stamp;
    pmsg.header.frame_id = baro_frame_id_;
    pmsg.fluid_pressure = baro_data_.pressure; // Pascals
    pmsg.variance = 0.0; // unknown
    baro_pressure_pub_.publish(pmsg);

    std_msgs::Float32 amsg;
    amsg.data = static_cast<float>(baro_data_.altitude); // meters
    baro_altitude_pub_.publish(amsg);
}

void FBRTKDriver::publishGPS() {
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.header.frame_id = gps_frame_id_;
    
    gps_msg.latitude = gps_data_.latitude;
    gps_msg.longitude = gps_data_.longitude;
    gps_msg.altitude = gps_data_.altitude;
    
    // Status
    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    if (gps_data_.fix_quality == 0) {
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    } else if (gps_data_.fix_quality == 1 || gps_data_.fix_quality == 2) {
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    } else if (gps_data_.fix_quality == 4 || gps_data_.fix_quality == 5) {
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; // RTK
    }
    
    // Position covariance (simplified)
    double pos_cov = (gps_data_.fix_quality >= 4) ? 0.01 : 1.0; // RTK vs standard GPS
    for (int i = 0; i < 9; ++i) {
        gps_msg.position_covariance[i] = 0.0;
    }
    gps_msg.position_covariance[0] = pos_cov;
    gps_msg.position_covariance[4] = pos_cov;
    gps_msg.position_covariance[8] = pos_cov * 4; // Altitude less accurate
    gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    
    gps_pub_.publish(gps_msg);
    
    // Publish velocity (only speed magnitude, no direction info available)
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header = gps_msg.header;
    vel_msg.twist.linear.x = gps_data_.speed_ms; // Speed in forward direction
    vel_msg.twist.linear.y = 0.0;
    vel_msg.twist.linear.z = 0.0;
    vel_msg.twist.angular.x = 0.0;
    vel_msg.twist.angular.y = 0.0;
    vel_msg.twist.angular.z = 0.0;
    
    velocity_pub_.publish(vel_msg);
}

void FBRTKDriver::publishWaypoint() {
    ROS_INFO("Waypoint: lon=%.7f, lat=%.7f, alt=%.2f m, distance=%d m, total=%d m",
             waypoint_data_.t_longitude, waypoint_data_.t_latitude, waypoint_data_.t_altitude,
             waypoint_data_.distance, waypoint_data_.t_distance);

    // Publish standard messages
    sensor_msgs::NavSatFix wfix;
    wfix.header.stamp = ros::Time::now();
    wfix.header.frame_id = gps_frame_id_;
    wfix.latitude = waypoint_data_.t_latitude;
    wfix.longitude = waypoint_data_.t_longitude;
    wfix.altitude = waypoint_data_.t_altitude;
    // Mark as non-sensor target: no fix
    wfix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    wfix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    wfix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    waypoint_fix_pub_.publish(wfix);

    std_msgs::Int16 dmsg; dmsg.data = waypoint_data_.distance; waypoint_distance_pub_.publish(dmsg);
    std_msgs::Int16 tmsg; tmsg.data = waypoint_data_.t_distance; waypoint_total_distance_pub_.publish(tmsg);
}

// Robot control callbacks
void FBRTKDriver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_vel = std::max(-max_linear_vel_, std::min(max_linear_vel_, msg->linear.x));
    double angular_vel = std::max(-max_angular_vel_, std::min(max_angular_vel_, msg->angular.z));
    
    sendControlCommand(linear_vel, angular_vel);
}

void FBRTKDriver::sendControlCommand(double linear_vel, double angular_vel) {
    if (!serial_.is_open()) {
        return;
    }
    
    try {
        // Control AT+CTRL=0,1000
        uint8_t cmd[32];
        
        // Convert velocities to 16-bit integers (scaled by 1000)
        int16_t linear_int = static_cast<int16_t>(linear_vel * 32767);
        int16_t angular_int = static_cast<int16_t>(angular_vel * 32767);

        snprintf(reinterpret_cast<char*>(cmd), sizeof(cmd), "AT+CTRL=%d,%d\r\n", angular_int, linear_int);
        // Send command
        boost::asio::write(serial_, boost::asio::buffer(cmd, strlen(reinterpret_cast<char*>(cmd))));

        // Log control commands for debugging
        ROS_DEBUG("Control command sent: linear=%.3f, angular=%.3f", linear_vel, angular_vel);
        
    }
    catch (const std::exception& e) {
        ROS_WARN("Failed to send control command: %s", e.what());
    }
}

int16_t FBRTKDriver::bytesToInt16(const uint8_t* data) {
    return (int16_t)((data[1] << 8) | data[0]);
}

int32_t FBRTKDriver::bytesToInt32(const uint8_t* data) {
    return (int32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
}

double FBRTKDriver::bytesToFloat32(const uint8_t* data) {
    union {
        uint32_t i;
        float f;
    } value;
    value.i = bytesToInt32(data);
    return static_cast<double>(value.f);
}

void FBRTKDriver::jpegCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    if (!serial_.is_open()) {
        ROS_WARN("Serial port not open; cannot send JPEG");
        return;
    }

    // Check format (optional): accept jpeg/jpg
    std::string fmt = msg->format;
    for (auto& c : fmt) c = static_cast<char>(::tolower(c));
    if (fmt.find("jpeg") == std::string::npos && fmt.find("jpg") == std::string::npos) {
        ROS_WARN("CompressedImage format is '%s', expected 'jpeg' or 'jpg'", msg->format.c_str());
        // Continue anyway if data exists
    }

    const std::vector<uint8_t>& data = msg->data;
    const size_t max_bytes = static_cast<size_t>(std::max(1, jpeg_max_size_kb_)) * 1024;
    if (data.empty()) {
        ROS_WARN("JPEG data is empty; skip sending");
        return;
    }
    if (data.size() > max_bytes) {
        ROS_WARN("JPEG too large: %zu bytes > %zu bytes (limit %dKB); drop", data.size(), max_bytes, jpeg_max_size_kb_);
        return;
    }

    try {
    // Send command first: AT+JPEG=<size>\r\n
    std::string cmd = std::string("AT+JPEG=") + std::to_string(data.size()) + "\r\n";
    boost::asio::write(serial_, boost::asio::buffer(cmd.data(), cmd.size()));

    // Wait 200 ms before sending data
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Then send raw JPEG bytes
    if (!data.empty()) {
        boost::asio::write(serial_, boost::asio::buffer(data.data(), data.size()));
    }

    ROS_INFO("JPEG sent: %zu bytes", data.size());
    } catch (const std::exception& e) {
        ROS_WARN("Failed to send JPEG: %s", e.what());
    }
}

void FBRTKDriver::lineTrackingCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (!serial_.is_open()) {
        ROS_WARN("Serial port not open; cannot send NVS command");
        return;
    }
    try {
        int value = msg->data;
        char cmd[32];
        int n = snprintf(cmd, sizeof(cmd), "AT+NVS=%d\r\n", value);
        if (n <= 0) {
            ROS_WARN("Failed to format NVS command for value %d", value);
            return;
        }
        boost::asio::write(serial_, boost::asio::buffer(cmd, static_cast<size_t>(n)));
        ROS_INFO("Sent NVS command: %s", cmd);
    } catch (const std::exception& e) {
        ROS_WARN("Failed to send NVS command: %s", e.what());
    }
}

} // namespace fbrtkros
