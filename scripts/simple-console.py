#!/usr/bin/env python
'''
start stop experiments via commandline
'''
import argparse
import rospy
import std_msgs.msg
import time
import std_srvs.srv
import uuid


def sync_cameras():
    print 'sync cameras'
    xx = rospy.ServiceProxy('/flydra_mainbrain/do_synchronization', std_srvs.srv.Empty)
    xx.call()
    rospy.sleep(1)

def take_background():
    print 'take background'
    xx = rospy.ServiceProxy('/flydra_mainbrain/take_background', std_srvs.srv.Empty)
    xx.call()
    rospy.sleep(1)

def stop_saving_hdf5():
    print 'stop saving HDF5'
    xx = rospy.ServiceProxy('/flydra_mainbrain/stop_saving_data', std_srvs.srv.Empty, persistent=True)
    xx.call()
    rospy.sleep(1)

def start_saving_hdf5():
    print 'start saving HDF5'
    xx = rospy.ServiceProxy('/flydra_mainbrain/start_saving_data', std_srvs.srv.Empty)
    xx.call()
    rospy.sleep(1)

def stop_saving_fmf():
    print 'stop saving FMF'
    xx = rospy.ServiceProxy('/flydra_mainbrain/stop_recording', std_srvs.srv.Empty)
    xx.call()
    rospy.sleep(1)

def start_saving_fmf():
    print 'start saving FMF'
    xx = rospy.ServiceProxy('/flydra_mainbrain/start_recording', std_srvs.srv.Empty)
    xx.call()
    rospy.sleep(1)

def stop_saving_ufmf():
    print 'stop saving UFMF'
    xx = rospy.ServiceProxy('/flydra_mainbrain/stop_small_recording', std_srvs.srv.Empty)
    xx.call()
    rospy.sleep(1)

def start_saving_ufmf():
    print 'start saving UFMF'
    xx = rospy.ServiceProxy('/flydra_mainbrain/start_small_recording', std_srvs.srv.Empty)
    xx.call()
    rospy.sleep(1)

def publish_experiment_uuid():
    rospy.init_node('experiment_uuid')
    pub = rospy.Publisher('/experiment_uuid', std_msgs.msg.String, latch=False)
    rospy.sleep(2)
    _uuid = uuid.uuid1().get_hex()
    print 'published experiment uuid', _uuid
    pub.publish(_uuid)
    rospy.sleep(2)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--sync-cameras', action='store_true')
    parser.add_argument(
        '--take-background', action='store_true')
    parser.add_argument(
        '--stop-saving-hdf5', action='store_true')
    parser.add_argument(
        '--start-saving-hdf5', action='store_true')
    parser.add_argument(
        '--publish-experiment-uuid', action='store_true')
    parser.add_argument(
        '--stop-saving-fmf', action='store_true')
    parser.add_argument(
        '--start-saving-fmf', action='store_true')
    parser.add_argument(
        '--stop-saving-ufmf', action='store_true')
    parser.add_argument(
        '--start-saving-ufmf', action='store_true')

    args = parser.parse_args()

    if args.sync_cameras:
        sync_cameras()
    if args.take_background:
        take_background()
    if args.stop_saving_hdf5:
        stop_saving_hdf5()
    if args.start_saving_hdf5:
        start_saving_hdf5()
    if args.stop_saving_fmf:
        stop_saving_fmf()
    if args.start_saving_fmf:
        start_saving_fmf()
    if args.stop_saving_ufmf:
        stop_saving_ufmf()
    if args.start_saving_ufmf:
        start_saving_ufmf()
    if args.publish_experiment_uuid:
        publish_experiment_uuid()

    print 'done'
