import pandas as pd
import numpy as np

import time

class DataRecorder(object):
    def __init__(self, tag = ''):
        self.tag = tag
        self.pos = {'index': [], 'x' : [], 'y': [], 'th':[]}
        self.posIndex = 0
        self.vel = {'index': [], 'v' : [], 'w':[], 'vr' :[], 'vl':[]}
        self.velIndex = 0
        self.sim = {'index': [], 'commandedLinearVelocity' : [], 'commandedAngularVelocity':[], 'commandedWheelAngle':[],
                    'angularVelocitySim':[] , 'linearVelocitySim' :[], 'wheelAngleSim':[],
                    'vr' :[], 'vl':[], 'vr_c' :[], 'vl_c':[]}
        self.simIndex = 0


    def recordPosition(self, x, y,th):
        ts = time.time()
        self.posIndex = self.posIndex +1
        self.pos['index'].append(self.posIndex)
        self.pos['x'].append(x)
        self.pos['y'].append(y)
        self.pos['th'].append(th)

    def recordVelocity(self, v, w, vr, vl):
        self.velIndex = self.velIndex +1
        self.vel['index'].append(self.velIndex)
        self.vel['v'].append(v)
        self.vel['w'].append(w)
        self.vel['vr'].append(vr)
        self.vel['vl'].append(vl)

    def recordSimData(self, commandedLinearVelocity,
                            commandedAngularVelocity,
                            commandedWheelAngle,
                            linearVelocitySim,
                            angularVelocitySim,
                            wheelAngleSim,
                            vr, vl, vr_c, vl_c):
        self.simIndex = self.simIndex +1
        self.sim['index'].append(self.simIndex)
        self.sim['commandedLinearVelocity'].append(commandedLinearVelocity)
        self.sim['commandedAngularVelocity'].append(commandedAngularVelocity)
        self.sim['commandedWheelAngle'].append(commandedWheelAngle)
        self.sim['angularVelocitySim'].append(angularVelocitySim)
        self.sim['linearVelocitySim'].append(linearVelocitySim)
        self.sim['wheelAngleSim'].append(wheelAngleSim)
        self.sim['vr'].append(vr)
        self.sim['vl'].append(vl)
        self.sim['vr_c'].append(vr_c)
        self.sim['vl_c'].append(vl_c)

    def save(self):
        df = pd.DataFrame(self.pos)
        df.to_csv('{0}pos.csv'.format(self.tag))
        dfv = pd.DataFrame(self.sim)
        dfv.to_csv('{0}vel.csv'.format(self.tag))
