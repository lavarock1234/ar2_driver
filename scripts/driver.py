#!/usr/bin/env python3

import serial
import pickle
import time

import rospy
import roslib
import math

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from ar2_driver.msg import ArmJointStep
from urdf_parser_py.urdf import URDF

# joint names, will check in run time if joint names are correct
joint_name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# default Control settings from AR2.py
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
Cal = pickle.load(open(calibration_filename, "rb"))
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


class JointDriver:
    def __init__(self):
        self.current_position = [0, 0, 0, 0, 0, 0]
        self.current_velocity = [0, 0, 0, 0, 0, 0]
        self.current_effort = [0, 0, 0, 0, 0, 0]

        # comm port
        port = "/dev/ttyACM0"
        baud = 115200
        self.ser = serial.Serial(port, baud)

    def J1jogPos(self, jog_deg):
        global J1StepCur
        global J1AngCur

        J1Degs = jog_deg
        J1jogSteps = int(J1Degs/J1DegPerStep)

        #calc pos dir output
        if J1motdir == "0":
            J1drivedir = "1"
        else:
            J1drivedir = "0"

        if J1Degs <= (J1PosAngLim - J1AngCur):
            command = "MJA"+J1drivedir+str(J1jogSteps)+"S"+Speed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(.2)
            self.ser.read()
            J1StepCur = J1StepCur + int(J1jogSteps)
            J1AngCur = round(J1NegAngLim + (J1StepCur * J1DegPerStep),2)

            # save data to file
            # pickle.dump(Cal, open(calibration_filename, "wb"), protocol=2)
        else:
            rospy.logwarn("J1 AXIS LIMIT")

    def J6jogPos(self, jog_deg):
        global J6StepCur
        global J6AngCur

        J6Degs = jog_deg
        J6jogSteps = int(J6Degs/J6DegPerStep)

        #calc pos dir output
        if J6motdir == "0":
            J6drivedir = "1"
        else:
            J6drivedir = "0"

        if (J6Degs <= (J6PosAngLim - J6AngCur)):
            command = "MJF"+J6drivedir+str(J6jogSteps)+"S"+Speed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(.2)
            self.ser.read()
            J6StepCur = J6StepCur + int(J6jogSteps)
            J6AngCur = round(J6NegAngLim + (J6StepCur * J6DegPerStep),2)

            # save data to file
            # pickle.dump(Cal, open(calibration_filename, "wb"), protocol=2)
        else:
            rospy.logwarn("J6 AXIS LIMIT")

    def J6jogNeg(self, jog_deg):
        global J6StepCur
        global J6AngCur

        J6Degs = jog_deg
        J6jogSteps = int(J6Degs/J6DegPerStep)

        if (J6Degs <= -(J6NegAngLim - J6AngCur)):
            command = "MJF"+J6motdir+str(J6jogSteps)+"S"+Speed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(.2)
            self.ser.read()
            J6StepCur = J6StepCur - int(J6jogSteps)
            J6AngCur = round(J6NegAngLim + (J6StepCur * J6DegPerStep),2)

            # save data to file
            # pickle.dump(Cal, open(calibration_filename, "wb"), protocol=2)
        else:
            rospy.logwarn("J6 AXIS LIMIT")

    def joint_cmd_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + ": Got joint cmd %s", data.points)

        # compute desired steps and command each joint
        J1deg = math.degrees(data.points[0].positions[0] - self.current_position[0])
        if J1deg > 0:
            rospy.loginfo("Jogging J1 positive " + str(J1deg) + " deg")
            self.J1jogPos(J1deg)

        J6deg = math.degrees(data.points[0].positions[5] - self.current_position[5])
        if J6deg > 0:
            rospy.loginfo("Jogging J6 positive " + str(J6deg) + " deg")
            self.J6jogPos(J6deg)
        else:
            rospy.loginfo("Jogging J6 negative " + str(-J6deg) + " deg")
            self.J6jogNeg(-J6deg)

        # set current position as command, this is open loop with no position feedback!
        self.current_position = data.points[0].positions


if __name__ == '__main__':
    try:
        rospy.init_node('ar2_driver', anonymous=False)

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
            state_out.position = driver.current_position
            state_out.velocity = driver.current_velocity
            state_out.effort = driver.current_effort
            state_pub.publish(state_out)
            # pub.publish(hello_str)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
