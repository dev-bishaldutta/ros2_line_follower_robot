#!/usr/bin/env python3

# Copyright 2025 Bishal Dutta (Bishalduttaoffcial@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You mayP copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
# --- Removed threading, sys, select, termios, tty ---

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        self.bridge = CvBridge()
        
        # --- MODIFIED: Set running to True to start automatically ---
        self.running = True
        
        # --- PID Controller Variables ---
        self.last_error = 0.0
        self.integral = 0.0  # --- ADDED: Integral term for PID ---
        # --- ADDED: "Memory" for search behavior ---
        self.last_known_error = 0.0 
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.image_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # --- REMOVED all keyboard monitoring code ---
        
        self.get_logger().info("🚀 Intelligent PID Line Follower Node Started. Running AUTOMATICALLY.")
        
    # --- REMOVED configure_terminal method ---
            
    # --- REMOVED restore_terminal method ---
                
    # --- REMOVED keyboard_monitor method ---
                
    # --- REMOVED check_keyboard method ---
        
    # --- REMOVED toggle_running method ---
            
    def image_callback(self, msg):
        """Process camera image and control robot"""
        
        # --- MODIFIED: Removed 'if not self.running' check ---
        # The node is always running now.
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
            
        frame = cv_image
        height, width = frame.shape[:2]
        
        # --- MODIFIED: Increased processing bands for more stability at speed ---
        num_bands = 5 # Was 3
        band_height = height // 10
        cx_sum = 0.0
        valid_bands = 0
        
        for i in range(num_bands):
            # Process bottom 5 bands of the image
            y_start = height - (i + 1) * band_height
            y_end = height - i * band_height
            
            y_start = max(0, y_start)
            y_end = min(height, y_end)
            
            if y_start >= y_end:
                continue
                
            roi = frame[y_start:y_end, :]
            
            # Convert to grayscale, blur, and threshold to get binary image
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # --- MODIFIED: Adjusted threshold for better line detection ---
            _, binary = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV) # Was 60
            
            # Morphology to clean up noise
            kernel = np.ones((3, 3), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            # Find contours (the line)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find the largest contour (should be the line)
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Calculate the center of the contour
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cx_sum += cx
                    valid_bands += 1
                    
        cmd_vel = Twist()
        
        if valid_bands > 0:
            # --- STATE 1: LINE IS VISIBLE (Follow normally) ---
            # We found the line
            avg_cx = cx_sum / valid_bands
            # Calculate error (how far from center is the line?)
            error = avg_cx - width / 2
            
            # --- ADDED: Store this error in "memory" ---
            self.last_known_error = error
            
            # --- UPGRADED TO PID CONTROLLER ---
            
            # --- P (Proportional) ---
            Kp = 0.003  
            
            # --- I (Integral) ---
            Ki = 0.0001 # New Integral Gain
            self.integral += error
            # --- Integral "Windup" Prevention ---
            max_integral = 1000
            if self.integral > max_integral: self.integral = max_integral
            if self.integral < -max_integral: self.integral = -max_integral
            
            # --- D (Derivative) ---
            Kd = 0.0015 
            derivative = error - self.last_error
            self.last_error = error
            
            # --- MODIFIED: Increased linear speed ---
            cmd_vel.linear.x = 0.5 # Was 0.4
            
            # --- The FINAL "Intelligent" Calculation ---
            cmd_vel.angular.z = -Kp * error - Ki * self.integral - Kd * derivative
            
            if abs(error) > 50:  
                self.get_logger().debug(f"🎯 Error: {error:.1f}, P:{(-Kp*error):.2f}, I:{(-Ki*self.integral):.2f}, D:{(-Kd*derivative):.2f}")
                
        else:
            # --- STATE 2: LINE IS LOST (Initiate Search) ---
            # This happens at a 90-degree turn.
            # Instead of stopping, we use our "memory" (last_known_error)
            # to decide which way to turn to find the line again.
            
            self.get_logger().warn("⚠️ Line lost! Initiating search turn...")
            
            # Reset PID controller to prevent wild behavior when line is refound
            self.integral = 0.0
            self.last_error = 0.0
            
            # Slow down linear speed to make the turn stable
            cmd_vel.linear.x = 0.1 # Slow crawl
            
            # Set a strong, fixed turning rate
            search_turn_rate = 1.0 # Radians/sec
            
            # --- INTELLIGENT DECISION ---
            # If the line was last seen to the left (negative error),
            # we need to turn LEFT (positive angular.z) to find it.
            if self.last_known_error < 0:
                self.get_logger().info("Searching LEFT")
                cmd_vel.angular.z = search_turn_rate 
            # If the line was last seen to the right (positive error),
            # we need to turn RIGHT (negative angular.z) to find it.
            else:
                self.get_logger().info("Searching RIGHT")
                cmd_vel.angular.z = -search_turn_rate
            
        self.cmd_vel_pub.publish(cmd_vel)
        
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        # Send a final stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # --- REMOVED restore_terminal() call ---
        
        self.get_logger().info("🛑 Line Follower Node shutting down")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = LineFollowerNode()
        
        # --- MODIFIED: Simplified startup message (and fixed typo) ---
        print("\n" + "="*50) # Was "="*5T
        print("🤖 INTELLIGENT PID ROBOT CONTROLLER")
        print("="*50)
        print("Starting robot automatically...")
        print("Waiting for camera feed to begin following.")
        print("Press Ctrl+C in this terminal to stop.")
        print("="*50 + "\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received (Ctrl+C)")
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        print("👋 Line Follower Node terminated")

if __name__ == '__main__':
    main()

