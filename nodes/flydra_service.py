#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_flydra')
import rospy
from ros_flydra.msg import *
from ros_flydra.srv import *

class Flydra_Service:
    def __init__(self):
        self.super_packet = None
        self.service = rospy.Service("flydra_super_packet_service", super_packet_service, self.service_callback)
        self.subscriber = rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.listen_callback)
        rospy.init_node('listener', anonymous=True)
        rospy.spin()

    def listen_callback(self, super_packet):
        #print 'super packet, ', len(super_packet.packets), ' packets'  
        self.super_packet = super_packet

    def service_callback(self, request):
        # request is null, just a placeholder
        return super_packet_serviceResponse(self.super_packet)

if __name__ == '__main__':
    flydra_service = Flydra_Service()
