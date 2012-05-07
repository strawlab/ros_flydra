#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_flydra')
import rospy
from ros_flydra.msg import *
from ros_flydra.srv import *
import time

class Flydra_Service_Listener:
    def __init__(self):
        rospy.wait_for_service("flydra_super_packet_service")
        self.get_latest_flydra_data = rospy.ServiceProxy("flydra_super_packet_service", super_packet_service)

    def print_data_every_second(self):
        time_prev = time.time()        
        while 1:
            if time.time()-time_prev >= 1:
                super_packet = self.get_latest_flydra_data()
                print super_packet
                time_prev = time.time() 

if __name__ == '__main__':
    flydra_service_listener = Flydra_Service_Listener()
    flydra_service_listener.print_data_every_second()
    
