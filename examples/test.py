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


# -----------------keyboard input----------------------

# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------


class M1:
    def __init__(self, device):
        self.device = device
        self.samples = 1
        self.values = 0
        self.callback = FnVoid_VoidP_DataP(self.data_handler)
        self.timeStamp = 0
        self.rollStamp = 0
        self.pitchStamp = 0
        self.pressed = False

    def data_handler(self, ctx, data):
        global temp
        self.values = parse_value(data, n_elem=1)
        temp = euler_from_quaternion(
            self.values.x, self.values.y, self.values.z, self.values.w)
        # print("%s w->%s x->%s y->%s z->%s" % (self.device.address,
        #                                       self.values.w, self.values.x, self.values.y, self.values.z))

        if((time.time() - self.timeStamp) > 0.5):
            self.timeStamp = time.time()
            self.rollStamp = temp[0]
            self.pitchStamp = temp[1]
            # ReleaseKey(0x16)
            self.pressed = False
        else:
            if((abs(temp[1] - self.pitchStamp) > 50)):
                self.timeStamp = time.time()
                self.pitchStamp = temp[1]
                # PressKey(0x16)
                self.pressed = True
        print("pitch->%s" % (temp[1]))
        self.samples += 1

class M2:
    def __init__(self, device):
        self.device = device
        self.samples = 1
        self.values = 0
        self.callback = FnVoid_VoidP_DataP(self.data_handler)
        self.timeStamp = 0
        self.rollStamp = 0
        self.pitchStamp = 0
        self.pressed = False

    def data_handler(self, ctx, data):
        global temp
        self.values = parse_value(data, n_elem=1)
        temp = euler_from_quaternion(
            self.values.x, self.values.y, self.values.z, self.values.w)
        # print("%s w->%s x->%s y->%s z->%s" % (self.device.address,
        #                                       self.values.w, self.values.x, self.values.y, self.values.z))
        if((time.time() - self.timeStamp) > 0.5):
            self.timeStamp = time.time()
            self.rollStamp = temp[0]
            self.pitchStamp = temp[1]
            # ReleaseKey(0x11)
            self.pressed = False
        else:
            if(abs(temp[0] - self.rollStamp) > 20):
                # PressKey(0x11)
                self.pressed = True
                self.timeStamp = time.time()
                self.rollStamp = temp[0]
        print("roll->%s" % (temp[0]))

        self.samples += 1

class State:
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.callback = FnVoid_VoidP_DataP(self.data_handler)

    def data_handler(self, ctx, data):
        print("%s -> %s" % (self.device.address, parse_value(data)))
        self.samples+= 1

d1 = MetaWear("ED:58:BA:AB:A6:21", hci_mac="04:ED:33:0F:00:A4")
d2 = MetaWear("D5:A1:B9:55:64:92", hci_mac="04:ED:33:0F:00:A4")
d3 = MetaWear("F8:57:8E:36:16:31", hci_mac="04:ED:33:0F:00:A4")
d4 = MetaWear("C8:CA:62:8D:52:2E", hci_mac="04:ED:33:0F:00:A4")
d1.connect()
print("Connected to " + d1.address)
d2.connect()
print("Connected to " + d2.address)
d3.connect()
print("Connected to " + d3.address)
d4.connect()
print("Connected to " + d4.address)
s1 = M1(d1)
s2 = M2(d2)
s3 = State(d3)
s4 = State(d4)
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

# sleep(1)

print("Configuring device M2")
libmetawear.mbl_mw_settings_set_connection_parameters(
    s2.device.board, 7.5, 7,5, 0, 6000)
sleep(1.5)

libmetawear.mbl_mw_sensor_fusion_set_mode(
    s2.device.board, SensorFusionMode.NDOF)
libmetawear.mbl_mw_sensor_fusion_set_acc_range(
    s2.device.board, SensorFusionAccRange._8G)
libmetawear.mbl_mw_sensor_fusion_set_gyro_range(
    s2.device.board, SensorFusionGyroRange._2000DPS)
libmetawear.mbl_mw_sensor_fusion_write_config(s2.device.board)

signal2 = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
    s2.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_datasignal_subscribe(signal2, None, s2.callback)

libmetawear.mbl_mw_sensor_fusion_enable_data(
    s2.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_sensor_fusion_start(s2.device.board)

print("Configuring device M3")
libmetawear.mbl_mw_settings_set_connection_parameters(s3.device.board, 7.5, 7.5, 0, 6000)
acc3 = libmetawear.mbl_mw_acc_get_acceleration_data_signal(s3.device.board)
libmetawear.mbl_mw_datasignal_subscribe(acc3, None, s3.callback)

gyro3 = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(s3.device.board)
libmetawear.mbl_mw_datasignal_subscribe(gyro3, None, s3.callback)

libmetawear.mbl_mw_acc_enable_acceleration_sampling(s3.device.board)
libmetawear.mbl_mw_acc_start(s3.device.board)

libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(s3.device.board)
libmetawear.mbl_mw_gyro_bmi160_start(s3.device.board)

print("Configuring device M4")
libmetawear.mbl_mw_settings_set_connection_parameters(s4.device.board, 7.5, 7.5, 0, 6000)
acc4 = libmetawear.mbl_mw_acc_get_acceleration_data_signal(s4.device.board)
libmetawear.mbl_mw_datasignal_subscribe(acc4, None, s4.callback)

gyro4 = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(s4.device.board)
libmetawear.mbl_mw_datasignal_subscribe(gyro4, None, s4.callback)

libmetawear.mbl_mw_acc_enable_acceleration_sampling(s4.device.board)
libmetawear.mbl_mw_acc_start(s4.device.board)

libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(s4.device.board)
libmetawear.mbl_mw_gyro_bmi160_start(s4.device.board)


sleep(30)
# ----------------------------------------------------------------------------
    # count_frequency = threading.Thread(target=freq)
    # count_frequency.start()
    # main = threading.Thread(target=main)
    # main.start()

    # main.join()
    # count_frequency.join()
# ----------------------------------------------------------------------------


libmetawear.mbl_mw_sensor_fusion_stop(s1.device.board)

signal1 = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
    s1.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_datasignal_unsubscribe(signal1)
libmetawear.mbl_mw_debug_disconnect(s1.device.board)

sleep(1)

libmetawear.mbl_mw_sensor_fusion_stop(s2.device.board)

signal2 = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
    s2.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_datasignal_unsubscribe(signal2)
libmetawear.mbl_mw_debug_disconnect(s2.device.board)

sleep(1)

libmetawear.mbl_mw_acc_stop(s3.device.board)
libmetawear.mbl_mw_acc_disable_acceleration_sampling(s3.device.board)
    
libmetawear.mbl_mw_gyro_bmi160_stop(s3.device.board)
libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(s3.device.board)

acc3 = libmetawear.mbl_mw_acc_get_acceleration_data_signal(s3.device.board)
libmetawear.mbl_mw_datasignal_unsubscribe(acc3)
    
gyro3 = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(s3.device.board)
libmetawear.mbl_mw_datasignal_unsubscribe(gyro3)

sleep(1)
    
libmetawear.mbl_mw_debug_disconnect(s3.device.board)

libmetawear.mbl_mw_acc_stop(s3.device.board)
libmetawear.mbl_mw_acc_disable_acceleration_sampling(s3.device.board)
    
libmetawear.mbl_mw_gyro_bmi160_stop(s3.device.board)
libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(s3.device.board)

acc4 = libmetawear.mbl_mw_acc_get_acceleration_data_signal(s3.device.board)
libmetawear.mbl_mw_datasignal_unsubscribe(acc4)
    
gyro4 = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(s3.device.board)
libmetawear.mbl_mw_datasignal_unsubscribe(gyro4)
    
libmetawear.mbl_mw_debug_disconnect(s3.device.board)

print("Total Samples Received")
print("%s -> %d" % (s1.device.address, s1.samples))
print("%s -> %d" % (s2.device.address, s2.samples))
print("%s -> %d" % (s3.device.address, s3.samples))
print("%s -> %d" % (s4.device.address, s4.samples))