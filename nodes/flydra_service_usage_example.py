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
                superpacket = self.get_latest_flydra_data().packets
                
                for packet in superpacket.packets:
                    print 
                    print '*'*80
                    for obj in packet.objects:
                        position = [obj.position.x, obj.position.y, obj.position.z]
                        velocity = [obj.velocity.x, obj.velocity.y, obj.velocity.z]
                        print
                        print 'obj id: ', obj.obj_id
                        print position
                        print velocity
                    
                time_prev = time.time() 

if __name__ == '__main__':
    flydra_service_listener = Flydra_Service_Listener()
    flydra_service_listener.print_data_every_second()
    
