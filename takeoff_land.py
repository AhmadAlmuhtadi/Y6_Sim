#!/usr/bin/env python
import rospy

import mavros
from mavros import command
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
#from mavros_msgs.msg import State
#from sensor_msgs.msg import NavSatFix
#from geometry_msgs.msg import PoseStamped



class Takeoff_land(object):

    #init method/ constructor
    def __init__(self):
        rospy.init_node("TakeoffLand")
        mavros.set_namespace()
        #self.state_subscriber = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.Takeoff= rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)
        self.arming  = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.Set_mode= rospy.ServiceProxy("mavros/set_mode", SetMode)
        #self.state = State() #for precautionary measures, not used yet
        #self.waypoint_list = []
        self.rate = rospy.Rate(20)
        self.latitude = 0
        self.longitude = 0

    def set_mode(self, mode="GUIDED"):
        rospy.wait_for_service("mavros/set_mode")
        isModeChanged = self.Set_mode(custom_mode=mode)
        return isModeChanged
        
    def arm(self, arming=True):
        rospy.wait_for_service("mavros/cmd/arming")
        return self.arming(arming)
        

    #Can be used to confirm mode change or arming. Not used for now
    #def state_cb(self, data):
    #self.state = data

    #
    #def set_position(self, x, y, z):
    
    def main(self):
        self.set_mode(mode='GUIDED')
        self.arm()
        command.takeoff(min_pitch=0, yaw=0, latitude=self.latitude, longitude=self.longitude, altitude=3)
        rospy.sleep(10)
        #self.waypoint_list = [[3,3,5], [6,6,2],[4,4,6]]

        command.land(min_pitch=0.0, yaw=0, latitude=self.latitude, longitude=self.longitude, altitude=0.0)


#
if __name__ == "__main__":
    
    TakeoffLand = Takeoff_land()
    TakeoffLand.main()