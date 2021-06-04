# usage: python stream_acc.py [mac1] [mac2] ... [mac(n)]
# python3 stream_quat.py D5:A1:B9:55:64:92 ED:58:BA:AB:A6:21
from __future__ import print_function
import ctypes
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from time import sleep
import time

import platform
import sys
import six

if sys.version_info[0] == 2:
    range = xrange

# ----------------helper function------------------------
import numpy as np
import os


def euler_from_quaternion(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z
# -----------------------------------------------------
import math

def euler_yzx_to_axis_angle(z_e, x_e, y_e, normalize=True):
    # Assuming the angles are in radians.
    c1 = math.cos(z_e/2)
    s1 = math.sin(z_e/2)
    c2 = math.cos(x_e/2)
    s2 = math.sin(x_e/2)
    c3 = math.cos(y_e/2)
    s3 = math.sin(y_e/2)
    c1c2 = c1*c2
    s1s2 = s1*s2
    w = c1c2*c3 - s1s2*s3
    x = c1c2*s3 + s1s2*c3
    y = s1*c2*c3 + c1*s2*s3
    z = c1*s2*c3 - s1*c2*s3
    angle = 2 * math.acos(w)
    if normalize:
        norm = x*x+y*y+z*z
        if norm < 0.001:
            # when all euler angles are zero angle =0 so
            # we can set axis to anything to avoid divide by zero
            x = 1
            y = 0
            z = 0
        else:
            norm = math.sqrt(norm)
            x /= norm
            y /= norm
            z /= norm
    return z, x, y, angle
# -----------------------------------------------------
class M1:
    def __init__(self, device):
        self.device = device
        self.samples = 1
        self.values = 0
        self.timeStamp = 0
        self.pitchMax = 0
        self.pitchMin = 0
        self.callback = FnVoid_VoidP_DataP(self.data_handler)

    def data_handler(self, ctx, data):
        self.values = parse_value(data, n_elem=1)
        self.samples += 1
        temp = euler_from_quaternion(
            self.values.x, self.values.y, self.values.z, self.values.w)
        print("x->%.4f y->%.4f z->%.4f" % (temp[0], temp[1], temp[2]))
        if(temp[1] > self.pitchMax):
            pitchMax = temp[1]
        elif(temp[1] < self.pitchMin):
            pitchMin = temp[1]
        if((time.time() - self.timeStamp) > 0.5):
            self.timeStamp = time.time()
            self.rollStamp = temp[0]
            self.pitchStamp = temp[1]
            print("ReleaseKey(0x11)")
            self.pressed = False
        else:
            if(abs(temp[0] - self.rollStamp) > 20):
                print("PressKey(0x11)")
                self.pressed = True
                self.timeStamp = time.time()
                self.rollStamp = temp[0]

            
        # print("%s w->%.4f x->%.4f y->%.4f z->%.4f" % (self.device.address,
        #                                       self.values.w, self.values.x, self.values.y, self.values.z))

d1 = MetaWear("D5:A1:B9:55:64:92", hci_mac="00:1A:7D:DA:71:15")
d1.connect()
print("Connected to " + d1.address)
s1 = M1(d1)
sleep(1)

print("Configuring device M1")
libmetawear.mbl_mw_settings_set_connection_parameters(
    s1.device.board, 7.5, 7,5, 0, 6000)
sleep(1.5)

libmetawear.mbl_mw_sensor_fusion_set_mode(
    s1.device.board, SensorFusionMode.NDOF)
libmetawear.mbl_mw_sensor_fusion_set_acc_range(
    s1.device.board, SensorFusionAccRange._8G)
libmetawear.mbl_mw_sensor_fusion_set_gyro_range(
    s1.device.board, SensorFusionGyroRange._2000DPS)
libmetawear.mbl_mw_sensor_fusion_write_config(s1.device.board)

signal1 = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
    s1.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_datasignal_subscribe(signal1, None, s1.callback)

libmetawear.mbl_mw_sensor_fusion_enable_data(
    s1.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_sensor_fusion_start(s1.device.board)

sleep(100)

libmetawear.mbl_mw_sensor_fusion_stop(s1.device.board)

signal1 = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
    s1.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_datasignal_unsubscribe(signal1)
libmetawear.mbl_mw_debug_disconnect(s1.device.board)

sleep(1)

print("Total Samples Received")
print("%s -> %d" % (s1.device.address, s1.samples))