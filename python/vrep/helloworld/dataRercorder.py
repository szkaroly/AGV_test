import pandas as pd
import numpy as np

import time


class dataRecorder(object):
    def __init__(self):
        self.pos = {'index': [], 'x' : [], 'y': [], 'th':[]}
        self.posIndex = 0
        self.vel = {'index': [], 'v' : [], 'w':[]}
        self.velIndex = 0

    def recordPosition(self, x, y,th):
        ts = time.time()
        self.posIndex = self.posIndex +1
        self.pos['index'].append(self.posIndex)
        self.pos['x'].append(x)
        self.pos['y'].append(y)
        self.pos['th'].append(th)

    def recordVelocity(self, v, w):
        self.velIndex = self.velIndex +1
        self.vel['index'].append(self.velIndex)
        self.vel['v'].append(v)
        self.vel['w'].append(w)

    def save(self):
        df = pd.DataFrame(self.pos)
        print(df)
        df.to_csv('pos.csv')
        dfv = pd.DataFrame(self.vel)
        dfv.to_csv('vel.csv')
