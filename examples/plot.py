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

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

# Set graphical window, its title and size
win = pg.GraphicsWindow(title="Sample process")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example')

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

# Random data process
p6 = win.addPlot(title="Updating plot")
curve = p6.plot(pen='y')
data = np.random.normal(size=(10,1000)) #  If the Gaussian distribution shape is, (m, n, k), then m * n * k samples are drawn.

# plot counter
ptr = 0

# Function for updating data display
def update():
    global curve, data, ptr, p6
    curve.setData(data[ptr%10])
    if ptr == 0:
        p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr += 1

# Update data display    
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)

# -----------------------------------------------------

class M1:
    def __init__(self, device):
        self.device = device
        self.samples = 1
        self.values = 0
        self.callback = FnVoid_VoidP_DataP(self.data_handler)
        import sys
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def data_handler(self, ctx, data):
        self.values = parse_value(data, n_elem=1)
        self.samples += 1

        global curve, ptr, p6
        curve.setData(self.values.w)
        if ptr == 0:
            p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
        ptr += 1

        print("%s w->%s x->%s y->%s z->%s" % (self.device.address,
                                              self.values.w, self.values.x, self.values.y, self.values.z))

d1 = MetaWear("D5:A1:B9:55:64:92", hci_mac="04:ED:33:0F:00:A4")
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

sleep(30)

libmetawear.mbl_mw_sensor_fusion_stop(s1.device.board)

signal1 = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
    s1.device.board, SensorFusionData.QUATERNION)
libmetawear.mbl_mw_datasignal_unsubscribe(signal1)
libmetawear.mbl_mw_debug_disconnect(s1.device.board)

sleep(1)

# pg.QtGui.QApplication.exec_()
print("Total Samples Received")
print("%s -> %d" % (s1.device.address, s1.samples))