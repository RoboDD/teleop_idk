#!/usr/bin/env python
import os
import sys
import numpy as np
import math
from time import sleep

# ! old dependency, use sm later
from scipy.spatial.transform import Rotation as R

# ROS Messages
import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

# Robotics Toolbox
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg

# XArm Messages
from xarm_msgs.msg import *
from xarm_msgs.srv import *
from xarm_planner.srv import *

from Rosmaster_Lib import Rosmaster

global USE_ROSMASTER_SERVO
USE_ROSMASTER_SERVO = True

########################################################################################
def denormalize(values, orig_range):
    a, b = orig_range
    return [value * (b - a) + a for value in values]


class TeleoperationSyncToReal:

    def __init__(self):

        rospy.logdebug("XArm (Lite6) Teleoperation ROS node initialized!")
        rospy.init_node("teleop_kinova")

        
        ########################################################################

        self.user_control_sub = rospy.Subscriber("/pose_sp", 
                                                 PoseStamped, 
                                                 self.user_control_cb)
        self.user_sp = []

        self.user_gripper_sub = rospy.Subscriber("/gripper_sp", 
                                                 PoseStamped, 
                                                 self.user_gripper_cb)
        self.user_gripper_sp = []

        if USE_ROSMASTER_SERVO == True:
            self.servo_gripper_ctl = Rosmaster()

        ########################################################################

        # XArm Setup
        # Doc: https://github.com/xArm-Developer/xarm_ros/tree/master

        mode_req = SetInt16Request()
        mode_req.data = 5 # 5: Cartesian Velocity; 7: Move line
        
        state_req = SetInt16Request()
        state_req.data = 0 # Ready State

        max_acc_req = SetFloat32Request()
        max_acc_req.data = 50000  # maximum 50000 

        ufactory_state_srv = rospy.ServiceProxy('/ufactory/set_state', 
                                                SetInt16)
        ufactory_mode_srv = rospy.ServiceProxy('/ufactory/set_mode', 
                                               SetInt16)
        ufactory_max_acc_line_srv = rospy.ServiceProxy('/ufactory/set_max_acc_line', 
                                                  SetFloat32)

        ufactory_mode_srv(mode_req)
        ufactory_state_srv(state_req)
        ufactory_max_acc_line_srv(max_acc_req)
        # ! need to set /xarm/wait_for_finish

        ########################################################################

        self.ufactory_ctl = rospy.ServiceProxy('/ufactory/velo_move_line_timed', 
                                               MoveVelocity)
        self.robot_state = rospy.Subscriber('ufactory/robot_states', 
                                            RobotMsg, 
                                            self.robot_states)
        self.gripper_ctl = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', 
                                              SetInt16)

        ########################################################################

        # Robotics Toolbox
        self.p_servo_gain = np.array([1,1,1, 2, 2, 2])
        self.controller = rospy.Timer(rospy.Duration(0.1), 
                                      self.rrmc_controller)
        
        ########################################################################
    ########################################################################################

    ########################################################################################
    def robot_states(self,data):

        # Update end-feector position and velocity
        states = data.pose

        self.eff_pose = np.array([states[0]/1000, states[1]/1000, states[2]/1000, # EFF position [meter]
                         states[3], states[4], states[5] # EFF angle [rad]
                        ])
    ########################################################################################

    ########################################################################################
    def user_gripper_cb(self,data):

        # Get User Input, the position of hand controller
        # ! change to SE3 using sp
        value = data.pose.position.x

        if USE_ROSMASTER_SERVO == True:
            # Example normalized values
            a = 100
            b = 160
            denormalized_values = value * (b - a) + a

            self.servo_gripper_ctl.set_pwm_servo(3, denormalized_values)

        req = SetInt16Request()
        if value < 0.5:
            req.data = 0 # 0 or 1
        if value > 0.5:
            req.data = 1 # 0 or 1

        rospy.logdebug('gripper moving')
        try:
            self.gripper_ctl(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call Srospy.wait_for_serviceendGripperCommand")
            pass  

    
    ########################################################################################


    ########################################################################################
    def user_control_cb(self,data):

        # Get User Input, the position of hand controller
        # ! change to SE3 using sp
        r = R.from_quat([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
        _angle = r.as_euler('zyx', degrees=True)

        _theta_x = (-_angle[0]+180) % 360 # Roll
        _theta_y = (_angle[1]) % 360 # Pitch -90 for horizontal tasks (have some bugs)
        _theta_z = (_angle[2]-180) % 360 # Yaw

        self.user_sp = np.array([data.pose.position.x, 
                                 data.pose.position.y, 
                                 data.pose.position.z, 
                                 _theta_x, 
                                 _theta_y, 
                                 -_theta_z])
        # print(self.user_sp[3:6])
    ########################################################################################

    ########################################################################################
    # ! Resolved-Rate Motion Control (RRMC) (using the Roboticcs Toolbox)
    def rrmc_controller(self, event):

        rospy.logdebug("Controller called one!")

        arrived = False
        
        while not arrived:

            # Note: The end-effector pose (using .A to get a numpy array instead of an SE3 object)
            # Get current end-effector pose [m, rad]
            Te = sm.SE3.Trans(self.eff_pose[0], 
                              self.eff_pose[1], 
                              self.eff_pose[2]) * sm.SE3.RPY(self.eff_pose[3],
                                                             self.eff_pose[4],
                                                             self.eff_pose[5],unit="rad")

  
            # Get position setpoints [m, deg]
            Tep = sm.SE3.Trans(self.user_sp[0], 
                               self.user_sp[1], 
                               self.user_sp[2]) * sm.SE3.RPY(self.user_sp[3],
                                                             self.user_sp[4],
                                                             self.user_sp[5],unit="deg")

            # Calculate the required end-effector velocity and whether the robot has arrived
            ev, arrived = rtb.p_servo(Te, Tep, gain=self.p_servo_gain, threshold=1, method='angle-axis')

            rospy.logdebug(ev)
           
            # Apply the required end-effector velocity to the robot
            req = MoveVelocityRequest()
            req.speeds = [ev[0]*1000, ev[1]*1000, ev[2]*1000, ev[3], ev[4], ev[5]]
            req.is_tool_coord = 0
            req.is_sync = 0
            req.duration = 0.1

            try:
                self.ufactory_ctl(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call SendGripperCommand")
                pass
        # end while
    ########################################################################################

    ########################################################################################
    def qrmc_controller(self, event):
        
        # TODO Quadratic Rate Motion Control
        rospy.logdebug("QRMC Controller")

        None
    ########################################################################################

    ######################################################################################## 
    def gripper_control(self, value):

        req = SetInt16Request()
        req.data = value # 0 or 1

        rospy.logdebug('gripper moving')
        try:
            self.gripper_ctl(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call Srospy.wait_for_serviceendGripperCommand")
            pass  
    ########################################################################################

########################################################################################
if __name__ == '__main__':

    TeleoperationSyncToReal()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down") 
        
########################################################################################