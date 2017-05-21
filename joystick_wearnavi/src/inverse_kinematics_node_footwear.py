#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from wearnavi_msg.msg import  JoystickSpeedStamped
from duckietown_msgs.srv import SetValueRequest, SetValueResponse, SetValue
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from numpy import *
import rospkg
import yaml
import time
import os.path


# Inverse Kinematics Node
# Author: Robert Katzschmann, Shih-Yuan Liu

class InverseKinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]        

        # Set parameters using yaml file
        self.readParamFromFile()

        # Set local variable by reading parameters
        self.gain = self.setup_parameter("~gain", 1.0)
        self.trim = self.setup_parameter("~trim", 0.0)
        self.baseline = self.setup_parameter("~baseline", 0.1)
        self.radius = self.setup_parameter("~radius", 0.0318)
        self.k = self.setup_parameter("~k", 27.0)
        self.limit = self.setup_parameter("~limit", 1.0)
        self.limit_max = 1.0
        self.limit_min = 0.0

        # Prepare services
        self.srv_set_gain = rospy.Service("~set_gain", SetValue, self.cbSrvSetGain)
        self.srv_set_trim = rospy.Service("~set_trim", SetValue, self.cbSrvSetTrim)
        self.srv_set_baseline = rospy.Service("~set_baseline", SetValue, self.cbSrvSetBaseline)
        self.srv_set_radius = rospy.Service("~set_radius", SetValue, self.cbSrvSetRadius)
        self.srv_set_k = rospy.Service("~set_k", SetValue, self.cbSrvSetK)
        self.srv_set_limit = rospy.Service("~set_limit", SetValue, self.cbSrvSetLimit)
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration)

        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.car_cmd_callback)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.sub_speed_state = rospy.Subscriber("~speed_state",JoystickSpeedStamped,self.speed_set)

        rospy.loginfo("[%s] Initialized.", self.node_name)
        self.printValues()

    def speed_set(self,speed_state_msg):#change speed
        rospy.loginfo("[%s] speed_set",self.node_name)
        self.speed_state = speed_state_msg.speed_state
        fname = self.getFilePath(self.veh_name)
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." % (self.node_name, fname))
            fname = self.getFilePath("default")
        #rospy.loginfo("fname = %s",fname)

        stream = open(fname,"r")
        yaml_dict = yaml.safe_load_all(stream)
        try:
            yaml_param =yaml_dict.next()
        except StopIteration:
            rospy.loginfo("StopIteration")        

        for param_name in ["gain"]:
            param_value = yaml_param.get(param_name)

        if (self.speed_state == 1 ): # 1 => speed up
            rospy.loginfo("[%s] speed up",self.node_name)
            param_value += 0.1   
            if(param_value >= 2.5):
                param_value = 2.5

        elif (self.speed_state == 2): # 2 => speed down
            rospy.loginfo("[%s] speed down",self.node_name)        
            param_value -= 0.1
            if param_value <= 0:
                param_value = 0;
        else: # 0 
            rospy.loginfo("Nothing")
        
        data = dict(
            baseline = self.baseline,
            calibration_time = "1970-01-01-03-29-58",
            gain = param_value,
            k = self.k,
            limit = self.limit,
            radius = self.radius,
            trim = self.trim

        )
        self.gain = param_value
        rospy.loginfo("[%s] gain = %s " %(self.node_name, self.gain))

        with open(fname, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
    
        stream.close()


    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
            fname = self.getFilePath("default")

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('duckietown')+'/config/baseline/calibration/kinematics/' + name + ".yaml"        

    def saveCalibration(self):
        # Write to yaml
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.gain,
            "trim": self.trim,
            "baseline": self.baseline,
            "radius": self.radius,
            "k": self.k,
            "limit": self.limit,
        }

        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))
        # Printout
        self.printValues()
        rospy.loginfo("[%s] Saved to %s" %(self.node_name, file_name))

    def cbSrvSaveCalibration(self, req):
        self.saveCalibration()
        return EmptyResponse()

    def cbSrvSetGain(self, req):
        self.gain = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetTrim(self, req):
        self.trim = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetBaseline(self, req):
        self.baseline = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetRadius(self, req):
        self.radius = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetK(self, req):
        self.k = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetLimit(self, req):
        self.limit = self.setLimit(req.value)
        self.printValues()
        return SetValueResponse()

    def setLimit(self, value):
        if value > self.limit_max:
            rospy.logwarn("[%s] limit (%s) larger than max at %s" % (self.node_name, value, self.limit_max))
            limit = self.limit_max
        elif value < self.limit_min:
            rospy.logwarn("[%s] limit (%s) smaller than allowable min at %s" % (self.node_name, value, self.limit_min))
            limit = self.limit_min
        else:
            limit = value
        return limit

    def printValues(self):
        rospy.loginfo("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))

    def car_cmd_callback(self, msg_car_cmd):
        # assuming same motor constants k for both motors
        k_r = self.k
        k_l = self.k

        # adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l
        
        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        
        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
