#!/usr/bin/env python

"""
FB-RTK Data Monitor
Simple script to monitor all FB-RTK sensor data
"""

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
import sys

class FBRTKMonitor:
    def __init__(self):
        rospy.init_node('fbrtk_monitor', anonymous=True)
        
        # Subscribers
        rospy.Subscriber('/fbrtk/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/fbrtk/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/fbrtk/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/fbrtk/velocity', TwistStamped, self.velocity_callback)
        rospy.Subscriber('/fbrtk/battery_voltage', Float32, self.battery_callback)
        
        # Data storage
        self.latest_data = {
            'imu': None,
            'gps': None,
            'odom': None,
            'velocity': None,
            'battery': None
        }
        
        print("FB-RTK Data Monitor Started")
        print("=" * 50)
    
    def imu_callback(self, msg):
        self.latest_data['imu'] = msg
        print(f"IMU: Accel({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}) "
              f"Gyro({msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f})")
    
    def gps_callback(self, msg):
        self.latest_data['gps'] = msg
        status_text = {0: "NO_FIX", 1: "FIX", 2: "SBAS_FIX", 3: "GBAS_FIX"}
        status = status_text.get(msg.status.status, "UNKNOWN")
        print(f"GPS: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Alt={msg.altitude:.2f}m, Status={status}")
    
    def odom_callback(self, msg):
        self.latest_data['odom'] = msg
        print(f"Odom: Pos({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}) "
              f"Vel({msg.twist.twist.linear.x:.2f}, {msg.twist.twist.angular.z:.2f})")
    
    def velocity_callback(self, msg):
        self.latest_data['velocity'] = msg
        speed = (msg.twist.linear.x**2 + msg.twist.linear.y**2)**0.5
        print(f"GPS Velocity: Speed={speed:.2f}m/s, East={msg.twist.linear.x:.2f}, North={msg.twist.linear.y:.2f}")
    
    def battery_callback(self, msg):
        self.latest_data['battery'] = msg
        print(f"Battery: {msg.data:.2f}V")
    
    def run(self):
        rate = rospy.Rate(0.5)  # 0.5 Hz for status summary
        
        while not rospy.is_shutdown():
            try:
                print("\n" + "="*50)
                print("FB-RTK System Status Summary")
                print("="*50)
                
                if self.latest_data['imu']:
                    print("✓ IMU data received")
                else:
                    print("✗ No IMU data")
                
                if self.latest_data['gps']:
                    print("✓ GPS data received")
                else:
                    print("✗ No GPS data")
                
                if self.latest_data['battery']:
                    voltage = self.latest_data['battery'].data
                    if voltage > 11.0:
                        print(f"✓ Battery OK: {voltage:.2f}V")
                    else:
                        print(f"⚠ Battery Low: {voltage:.2f}V")
                else:
                    print("✗ No battery data")
                
                rate.sleep()
                
            except KeyboardInterrupt:
                print("\nMonitor stopped by user")
                break

if __name__ == '__main__':
    try:
        monitor = FBRTKMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
