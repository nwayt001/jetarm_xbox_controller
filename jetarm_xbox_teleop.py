#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, os
import threading
from SimpleJoystick import SimpleJoystick
import numpy as np
import signal
from jetarm_xbox_controller.msg import SerialServoMove


class JetArm_Xbox():

    def __init__(self):
        # initialize joystic thread
        self.joystick = SimpleJoystick()

        self.js_thread = threading.Thread(target=self.joystick.poll)
        self.js_thread.start()

        # publish command velocities to servo move topic
        self.pub_servo = rospy.Publisher('/jetarm_sdk/serial_servo/move', SerialServoMove, queue_size=1)
        self.sub_servo = rospy.Subscriber('/jetarm_sdk/serial_servo/move', SerialServoMove, self.servo_data_callback)    
        self.servo_data_detection = False

        self.current_joint_id = 0  # Initialize at the first joint (ID 0)
        self.num_joints = 6  # Assuming there are 6 joints (IDs 0 to 5)
        self.current_servo_positions = [500] * self.num_joints  # Initialize all joints to the middle position (500)
        self.joint_increment = 50  # Sensitivity for servo movement
        self.rate = rospy.Rate(10) # 10hz

    def servo_data_callback(self, msg):
        self.servo_data_detection
        rospy.loginfo('Servo data: {}'.format(msg))
        if msg.servo_id != []:
            self.servo_data_detection = True

    def start(self):
    
        while not rospy.is_shutdown():

            # Check if the 'a' button is pressed to toggle through joint IDs
            if self.joystick.button_states['a']:
                self.current_joint_id = ((self.current_joint_id + 1) % self.num_joints)  # Cycle through joint IDs


            # get joystick values
            js_val_x = self.joystick.axis_states['x'] 
            js_val_y = self.joystick.axis_states['y'] 
            rospy.loginfo(js_val_y)
            # apply threasholding for deadzone
            if np.abs(js_val_x) < 0.4:
                js_val_x = 0.0
            if np.abs(js_val_y) < 0.15:
                js_val_y = 0.0
            
            # Calculate delta movements
            delta_y = js_val_y * self.joint_increment  # Adjust sensitivity as needed
            delta_x = js_val_x * self.joint_increment  # Adjust sensitivity as needed
            
            # Update servo position for the current joint based on delta values
            self.current_servo_positions[self.current_joint_id] += delta_y
            self.current_servo_positions[self.current_joint_id] = np.clip(self.current_servo_positions[self.current_joint_id], 0, 1000)  # Ensure within valid range
            
            # Prepare the servo control message
            data = SerialServoMove()
            data.servo_id = self.current_joint_id + 1 # Servo ID
            data.position = int(self.current_servo_positions[self.current_joint_id])  # Servo position
            data.duration = 250  # Duration in milliseconds

            # publish
            rospy.loginfo(data.position)
            
            self.pub_servo.publish(data)
            
            rospy.sleep(0.25)

        self.joystick.polling = False
        self.js_thread.join()


if __name__ == '__main__':

    # init node
    rospy.init_node('jetarm_xbox_teleop', anonymous=True)

    xbox_teleop = JetArm_Xbox()

    try:
        xbox_teleop.start()
    except rospy.ROSInterruptException:
        pass

