#!/usr/bin/env python3

'''
ROS compatible driver for AR2 Robot https://github.com/Chris-Annin/AR2

Much of code is modified from AR2.py in the above repository

Note - we use python3 here to maintain protocol compatibility with ARbot.cal generated
by the AR2 windows application.

Author: Xuchu (Dennis) Ding, xuchu.ding@gmail.com
'''

import serial
import pickle
import time
import sys

import rospy
import roslib
import math

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from ar2_driver.msg import ArmJointStep
from urdf_parser_py.urdf import URDF

# default control settings from AR2.py
Speed = "25"
ACCdur = "15"
ACCspd = "10"
DECdur = "20"
DECspd = "5"

# default motor directions from calibration page
J1motdir = "0"
J2motdir = "0"
J3motdir = "0"
J4motdir = "0"
J5motdir = "0"
J6motdir = "0"

# calibration file location
calibration_filename = "/home/xding/Dropbox/BlueHill/AR2 2.0 software exe files/ARbot.cal"

# load data from calibration file
try:
    Cal = pickle.load(open(calibration_filename, "rb"))
except:
    print("Unable to open calibration file: " + calibration_filename)
    sys.exit()

J1StepCur   = Cal[0]
J1AngCur    = Cal[1]
J2StepCur   = Cal[2]
J2AngCur    = Cal[3]
J3StepCur   = Cal[4]
J3AngCur    = Cal[5]
J4StepCur   = Cal[6]
J4AngCur    = Cal[7]
J5StepCur   = Cal[8]
J5AngCur    = Cal[9]
J6StepCur   = Cal[10]
J6AngCur    = Cal[11]
comPort     = Cal[12]
Prog        = Cal[13]
Servo0on    = Cal[14]
Servo0off   = Cal[15]
Servo1on    = Cal[16]
Servo1off   = Cal[17]
DO1on       = Cal[18]
DO1off      = Cal[19]
DO2on       = Cal[20]
DO2off      = Cal[21]
UFx         = Cal[22]
UFy         = Cal[23]
UFz         = Cal[24]
UFrx        = Cal[25]
UFry        = Cal[26]
UFrz        = Cal[27]
TFx         = Cal[28]
TFy         = Cal[29]
TFz         = Cal[30]
TFrx        = Cal[31]
TFry        = Cal[32]
TFrz        = Cal[33]
FineCalPos  = Cal[34]
J1NegAngLim = int(Cal[35])
J1PosAngLim = int(Cal[36])
J1StepLim   = int(Cal[37])
J2NegAngLim = int(Cal[38])
J2PosAngLim = int(Cal[39])
J2StepLim   = int(Cal[40])
J3NegAngLim = int(Cal[41])
J3PosAngLim = int(Cal[42])
J3StepLim   = int(Cal[43])
J4NegAngLim = int(Cal[44])
J4PosAngLim = int(Cal[45])
J4StepLim   = int(Cal[46])
J5NegAngLim = int(Cal[47])
J5PosAngLim = int(Cal[48])
J5StepLim   = int(Cal[49])
J6NegAngLim = int(Cal[50])
J6PosAngLim = int(Cal[51])
J6StepLim   = int(Cal[52])
DHr1        = Cal[53]
DHr2        = Cal[54]
DHr3        = Cal[55]
DHr4        = Cal[56]
DHr5        = Cal[57]
DHr6        = Cal[58]
DHa1        = Cal[59]
DHa2        = Cal[60]
DHa3        = Cal[61]
DHa4        = Cal[62]
DHa5        = Cal[63]
DHa6        = Cal[64]
DHd1        = Cal[65]
DHd2        = Cal[66]
DHd3        = Cal[67]
DHd4        = Cal[68]
DHd5        = Cal[69]
DHd6        = Cal[70]
DHt1        = Cal[71]
DHt2        = Cal[72]
DHt3        = Cal[73]
DHt4        = Cal[74]
DHt5        = Cal[75]
DHt6        = Cal[76]
CalDir      = Cal[77]
MotDir      = Cal[78]
TrackcurPos = Cal[79]
TrackLength = Cal[80]
TrackStepLim= Cal[81]
VisFileLoc  = Cal[82]
VisProg     = Cal[83]
VisOrigXpix = Cal[84]
VisOrigXmm  = Cal[85]
VisOrigYpix = Cal[86]
VisOrigYmm  = Cal[87]
VisEndXpix  = Cal[88]
VisEndXmm   = Cal[89]
VisEndYpix  = Cal[90]
VisEndYmm   = Cal[91]

# Compute joint angle in degrees per step
J1DegPerStep = float((J1PosAngLim - J1NegAngLim)/float(J1StepLim))
J2DegPerStep = float((J2PosAngLim - J2NegAngLim)/float(J2StepLim))
J3DegPerStep = float((J3PosAngLim - J3NegAngLim)/float(J3StepLim))
J4DegPerStep = float((J4PosAngLim - J4NegAngLim)/float(J4StepLim))
J5DegPerStep = float((J5PosAngLim - J5NegAngLim)/float(J5StepLim))
J6DegPerStep = float((J6PosAngLim - J6NegAngLim)/float(J6StepLim))

# joint names, will check in run time if joint names are correct
joint_name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


class AR2Joint:
    def __init__(self, PosAngLim, NegAngLim):
        self.PosAngLim = PosAngLim
        self.NegAngLim = NegAngLim


class JointDriver:
    def __init__(self):
        self.StepCur = [J1StepCur, J2StepCur, J3StepCur, J4StepCur, J5StepCur, J6StepCur]
        self.AngCur = [J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur]
        self.motdir = [J1motdir, J2motdir, J3motdir, J4motdir, J5motdir, J6motdir]
        self.drivecmd = ["A", "B", "C", "D", "E", "F"]
        self.neg_motdir = []
        for i in range(len(self.motdir)):
            if self.motdir[i] == "0":
                self.neg_motdir.append("1")
            else:
                self.neg_motdir.append("0")
        print("motdir: " + str(self.motdir) + ", neg_motdir: " + str(self.neg_motdir))

        self.DegPerStep = [J1DegPerStep, J2DegPerStep, J3DegPerStep, J4DegPerStep, J5DegPerStep, J6DegPerStep]

        self.current_velocity = [0, 0, 0, 0, 0, 0]
        self.current_effort = [0, 0, 0, 0, 0, 0]

        # set up joints
        self.joints = [
            AR2Joint(J1PosAngLim, J1NegAngLim),
            AR2Joint(J2PosAngLim, J2NegAngLim),
            AR2Joint(J3PosAngLim, J3NegAngLim),
            AR2Joint(J4PosAngLim, J4NegAngLim),
            AR2Joint(J5PosAngLim, J5NegAngLim),
            AR2Joint(J6PosAngLim, J6NegAngLim),
        ]

        # comm port
        port = "/dev/ttyArduino"
        baud = 115200
        self.ser = serial.Serial(port, baud)

    def jog(self, index, jog_deg):
        Degs = jog_deg

        StepCur = self.StepCur[index]
        AngCur = self.AngCur[index]

        motdir = self.motdir[index]
        neg_motdir = self.neg_motdir[index]

        DegPerStep = self.DegPerStep[index]
        jogSteps = int(Degs/DegPerStep)

        PosAngLim = self.joints[index].PosAngLim
        NegAngLim = self.joints[index].NegAngLim

        n = self.drivecmd[index]

        if Degs > 0:
            dir = neg_motdir
        else:
            dir = motdir

        rospy.loginfo("dir: " + dir + " jog_deg: " + str(jog_deg))

        cmd_valid = False
        if 0 < Degs <= (PosAngLim - AngCur):
            command = "MJ"+n+dir+str(jogSteps)+"S"+Speed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
            cmd_valid = True
        if 0 >= Degs >= NegAngLim - AngCur:
            command = "MJ"+n+dir+str(-jogSteps)+"S"+Speed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
            cmd_valid = True
        else:
            rospy.logerr("J" + str(index+1) + " AXIS LIMIT")

        if cmd_valid:
            rospy.loginfo("Sending command to arduino: " + command)
            # write data
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(.2)
            self.ser.read()

            # update current state
            self.StepCur[index] = StepCur + int(jogSteps)
            self.AngCur[index] = round(NegAngLim + (StepCur * DegPerStep),2)

            # save data to file
            # pickle.dump(Cal, open(calibration_filename, "wb"))

    def joint_cmd_cb(self, data):
        rospy.logdebug(rospy.get_caller_id() + ": Got joint cmd %s", data.points)

        # compute desired steps and command each joint
        for index in range(len(self.joints)):
            jog_deg = math.degrees(data.points[0].positions[index]) - self.AngCur[index]
            if abs(jog_deg) < 0.5:
                jog_deg = 0
            if jog_deg != 0:
                rospy.loginfo("Jogging J" + str(index+1) + " " + str(jog_deg) + " deg")
                self.jog(index, jog_deg)


if __name__ == '__main__':
    try:
        rospy.init_node('ar2_driver')

        robot = URDF.from_parameter_server()
        joint_name_check = []
        for joint in robot.joints:
            joint_name_check.append(joint.name)
        if joint_name != joint_name_check:
            rospy.logfatal("Joint name mismatch! " + str(joint_name) + " vs: " + str(joint_name_check))

        driver = JointDriver()
        rospy.Subscriber("/arm_controller/command", JointTrajectory, driver.joint_cmd_cb)
        step_pub = rospy.Publisher("/joint_steps", ArmJointStep, queue_size=100)
        state_pub = rospy.Publisher("/joint_states", JointState, queue_size=100)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            state_out = JointState()
            state_out.header.stamp = rospy.Time.now()
            state_out.name = joint_name
            state_out.position = [math.radians(val) for val in driver.AngCur]
            state_out.velocity = driver.current_velocity
            state_out.effort = driver.current_effort
            state_pub.publish(state_out)
            # pub.publish(hello_str)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
