#!/usr/bin/env python

"""
Simple Robot Control Test Script
Demonstrates basic robot movement commands using /cmd_vel topic at 5Hz
"""

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

def get_key():
    """Get a single key press from keyboard"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class RobotController:
    """Robot controller that sends cmd_vel messages at 5Hz"""
    
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.current_twist = Twist()
        self.lock = threading.Lock()
        
        # Create timer for 5Hz publishing
        self.timer = rospy.Timer(rospy.Duration(0.2), self.publish_cmd_vel)
        
        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
    
    def publish_cmd_vel(self, event):
        """Timer callback to publish cmd_vel at 5Hz"""
        with self.lock:
            self.cmd_pub.publish(self.current_twist)
    
    def set_forward(self):
        """Set robot to move forward"""
        with self.lock:
            self.current_twist.linear.x = self.linear_speed
            self.current_twist.angular.z = 0.0
    
    def set_left_turn(self):
        """Set robot to turn left"""
        with self.lock:
            self.current_twist.linear.x = 1.0
            self.current_twist.angular.z = self.angular_speed
    
    def set_right_turn(self):
        """Set robot to turn right"""
        with self.lock:
            self.current_twist.linear.x = 1.0
            self.current_twist.angular.z = -self.angular_speed
    
    def set_stop(self):
        """Stop the robot"""
        with self.lock:
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0

def main():
    rospy.init_node('simple_robot_control', anonymous=True)
    
    # Create robot controller
    controller = RobotController()
    
    print("Simple Robot Control Test (5Hz cmd_vel)")
    print("=" * 40)
    print("Controls:")
    print("  w - Forward (%.1f m/s)" % controller.linear_speed)
    print("  a - Left turn (%.1f rad/s)" % controller.angular_speed)
    print("  d - Right turn (%.1f rad/s)" % controller.angular_speed)
    print("  s - Stop")
    print("  q - Quit")
    print("=" * 40)
    print("Commands are sent at 5Hz to /cmd_vel")
    print("Press keys to control the robot...")
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key == 'w':
                controller.set_forward()
                print("Command: Forward")
            elif key == 'a':
                controller.set_left_turn()
                print("Command: Left turn")
            elif key == 'd':
                controller.set_right_turn()
                print("Command: Right turn")
            elif key == 's':
                controller.set_stop()
                print("Command: Stop")
            elif key == 'q':
                controller.set_stop()
                print("Command: Stop")
                print("Exiting...")
                break
            else:
                print(f"Unknown key: {key}")
                
    except KeyboardInterrupt:
        print("\nStopping robot and exiting...")
        controller.set_stop()
        rospy.sleep(0.5)  # Allow time for stop command to be sent

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
