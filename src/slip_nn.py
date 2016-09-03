#!/usr/bin/env python

#
# This script/library will serve as the holding ground for the neural network
# function(s)
#

from __future__ import division

import numpy as np
import numpy.matlib as npm


# =============== Define Biotac Class ===============
class BiotacData(object):
    """docstring for Slip"""

    np_average = np.average
    w = 5  # window size for mvavg

    def __init__(self):
        self._angle_window = [0]*self.w # np.array([0]*5)
        self.angle_mvavg = [0]
        # self._angle_put = self._angle_window.put

        self._elect_db = []  # [0]*50
        self.elect_base = [float(0)]*24 #  np.array([float(0)]*24) #np.matrix([[0]]*24)

        self._pac0_db = []  # collection of baseline values
        self._pac0_window = [0]*self.w  # window of last 5 values for moving average
        self.pac0_base = [0]  # baseline averge for reference
        self.pac0_mvavg = [0]  # moving average

        self._pac1_db = []  # [0]*50
        self._pac1_window = [0]*self.w # np.array([0]*5) #[0]*5
        self.pac1_base = [0]
        self.pac1_mvavg = [0]

        self._pdc_db = []
        self._pdc_window = [0]*self.w # np.array([0]*5) #[0]*5
        self.pdc_base = [0]
        self.pdc_mvavg = [0]

        self._tac_db = []
        self._tac_window = [0]*self.w
        self.tac_base = [0]
        self.tac_mvavg = [0]

        self._tdc_db = []
        self._tdc_window = [0]*self.w
        self.tdc_base = [0]
        self.tdc_mvavg = [0]

    def new(self, x, name=None):
        if name == "angle":
            self._angle_window.pop()
            self._angle_window.insert(0, x)
        elif name == "pac0":
            self._pac0_window.pop()
            self._pac0_window.insert(0, x)
        elif name == "pac1":
            self._pac1_window.pop()
            self._pac1_window.insert(0, x)
        elif name == "pdc":
            self._pdc_window.pop()
            self._pdc_window.insert(0, x)
        elif name == "tac":
            self._tac_window.pop()
            self._tac_window.insert(0, x)
        elif name == "tdc":
            self._tdc_window.pop()
            self._tdc_window.insert(0, x)
        else:
            print("Proper name value not provided! ['angle', 'pac0', 'pac1', \
                'pdc', 'tac', 'tdc']")
            return
        self._update_mvavg()

    def _update_mvavg(self):
        self.angle_mvavg[0] = sum(self._angle_window)/self.w
        self.pac0_mvavg[0] = sum(self._pac0_window)/self.w
        self.pac1_mvavg[0] = sum(self._pac1_window)/self.w
        self.pdc_mvavg[0] = sum(self._pdc_window)/self.w
        self.tac_mvavg[0] = sum(self._tac_window)/self.w
        self.tdc_mvavg[0] = sum(self._tdc_window)/self.w

    def baseline(self, name=None):
        if name == "electrodes":
            self.elect_base[:] = np.average(self._elect_db, 0)
        elif name == "pac0":
            self.pac0_base[0] = np.average(self._pac0_db)
        elif name == "pac1":
            self.pac1_base[0] = np.average(self._pac1_db)
        elif name == "pdc":
            self.pdc_base[0] = np.average(self._pdc_db)
        elif name == "tac":
            self.tac_base[0] = np.average(self._tac_db)
        elif name == "tdc":
            self.tdc_base[0] = np.average(self._tdc_db)

    def database(self, x, name=None):
        if name == "electrodes":
            self._elect_db.append(x)
            if len(self._elect_db) >= 50:
                self.baseline(name="electrodes")
        elif name == "pac0":
            self._pac0_db.append(x)
            if len(self._pac0_db) >= 50:
                self.baseline(name="pac0")
        elif name == "pac1":
            self._pac1_db.append(x)
            if len(self._pac1_db) >= 50:
                self.baseline(name="pac1")
        elif name == "pdc":
            self._pdc_db.append(x)
            if len(self._pdc_db) >= 50:
                self.baseline(name="pdc")
        elif name == "tac":
            self._tac_db.append(x)
            if len(self._tac_db) >= 50:
                self.baseline(name="tac")
        elif name == "tdc":
            self._tdc_db.append(x)
            if len(self._tdc_db) >= 50:
                self.baseline(name="tdc")
        else:
            print("Proper name value not provided! ['electrdes', 'pac0', 'pac1', \
            'pdc', 'tac', 'tdc']")
            return
