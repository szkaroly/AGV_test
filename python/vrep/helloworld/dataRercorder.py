import pandas as pd
import numpy as np

import time


class dataRecorder(object):
    def __init__(self):
        self.pos = {'index': [], 'x' : [], 'y': [], 'th':[]}
        self.posIndex = 0
        self.vel = {'index': [], 'v' : [], 'w':[], 'vr' :[], 'vl':[]}
        self.velIndex = 0
        self.sim = {'index': [], 'v_c' : [], 'w_ang_c':[], 'w_ang_sim':[] , 'vr' :[], 'vl':[]}
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

    def recordSimData(self, v_c, w_ang_c, w_ang_sim, vr, vl):
        self.simIndex = self.simIndex +1
        self.sim['index'].append(self.simIndex)
        self.sim['v_c'].append(v_c)
        self.sim['w_ang_c'].append(w_ang_c)
        self.sim['w_ang_sim'].append(w_ang_sim)
        self.sim['vr'].append(vr)
        self.sim['vl'].append(vl)

    def save(self):
        df = pd.DataFrame(self.pos)
        #df.to_csv('pos.csv')
        dfv = pd.DataFrame(self.sim)
        dfv.to_csv('vel.csv')
