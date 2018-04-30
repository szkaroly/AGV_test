import pandas as pd
import numpy as np

import time


class dataRecorder(object):
    def __init__(self):
        self.pos = {'x': [], 'y': [], 'th':[]}

    def recordPosition(self, x, y,th):
        ts = time.time()
        self.pos['x'].append(x)
        self.pos['y'].append(y)
        self.pos['th'].append(th)

    def save(self):
        df = pd.DataFrame(self.pos)
        df.to_csv('pos.csv')
