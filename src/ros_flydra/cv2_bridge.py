import array
import cv2
import numpy as np

import roslib;
roslib.load_manifest('sensor_msgs')
import sensor_msgs.msg

_NP_TO_ENC = {
    np.dtype(np.uint8):"8UC",
    np.dtype(np.uint16):"16UC"
}
_ENC_TO_NP = {
    "8UC1":(np.uint8,1),
    "8UC2":(np.uint8,2),
    "8UC3":(np.uint8,3),
    "16UC1":(np.uint16,1),
    "16UC2":(np.uint16,2),
    "16UC3":(np.uint16,3),
    "bayer_bggr8":(np.uint8,1),
}
_ENC_TO_ARR = {
    "8UC1":"B",
    "8UC2":"B",
    "8UC3":"B",
    "16UC1":"H",
    "16UC2":"H",
    "16UC3":"H",
    "bayer_bggr8":"B",
}


def numpy_to_imgmsg(image,stamp=None, tostring=True):
    enc = _NP_TO_ENC[image.dtype]

    rosimage = sensor_msgs.msg.Image()
    rosimage.height = image.shape[0]
    rosimage.width = image.shape[1]
    rosimage.encoding = '%s%d' % (enc,image.shape[2])
    rosimage.step = image.shape[2] * rosimage.width

    #FIXME: Why are both legal?
    if tostring:
        rosimage.data = image.ravel().tostring()
    else:
        rosimage.data = image.ravel().tolist()

    if stamp is not None:
        rosimage.header.stamp = stamp

    return rosimage

def imgmsg_to_numpy(msg):
    dtype,nchan = _ENC_TO_NP[msg.encoding]

    #FIXME: I have no idea why I can legally serialze images
    #as lists of ints, but deserialize them from strings. Yay ROS
    #
    #count improves performance a little
    if type(msg.data) is str:
        arr = np.fromstring(msg.data, dtype=dtype, count=msg.step*msg.height)
    else:
        arr = np.fromiter(msg.data, dtype=dtype, count=msg.step*msg.height)

    arr.shape = (msg.height, msg.width, nchan)

    return arr

def imgmsg_to_array(msg):
    typecode = _ENC_TO_ARR[msg.encoding]
    return array.array(typecode, msg.data)

if __name__ == "__main__":
    for nchan in (1,2,3):
        for d in (np.uint8, np.uint16):
            m1 = np.zeros((640,480,nchan), dtype=d)
            msg = numpy_to_imgmsg(m1)
            m2 = imgmsg_to_numpy(msg)
            assert np.allclose(m1,m2)
            assert m1.dtype == m2.dtype

