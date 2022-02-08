import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

PATH= r'~/tmp/vins_mono_log/39/'
TRANSLATION_PATH = PATH + r'extrinsic_parameter_T.txt'
ROTATION_PATH = PATH + r'extrinsic_parameter_R.txt'
VEL_PATH = PATH + r'vs.txt'
BA_PATH = PATH + r'ba.txt'
BG_PATH = PATH + r'bg.txt'
ORI_PATH = PATH + r'ori.txt'

t = pd.read_csv(TRANSLATION_PATH, sep=" ", header=None, skipinitialspace=True)
r = pd.read_csv(ROTATION_PATH, sep=" ", header=None, skipinitialspace=True)
vel = pd.read_csv(VEL_PATH, sep=" ", header=None, skipinitialspace=True)
vel_norm = np.sqrt(np.square(vel).sum(axis=1)) #calaulate the norm as speed
ba = pd.read_csv(BA_PATH, sep=" ", header=None, skipinitialspace=True)
ba_norm = np.sqrt(np.square(ba).sum(axis=1))
bg = pd.read_csv(BG_PATH, sep=" ", header=None, skipinitialspace=True)
bg_norm = np.sqrt(np.square(bg).sum(axis=1))
ori = pd.read_csv(ORI_PATH, sep=" ", header=None, skipinitialspace=True)

#print(t.shape);print(r.shape);print(vel.shape);print(bg.shape);print(ba.shape)
#print(speed.shape)

startIndex = 0

fig1, ax = plt.subplots(10, 1, sharex=True)
ax[0].plot(vel_norm[startIndex:])
ax[1].plot(ori.iloc[startIndex:, 0])
ax[2].plot(ba_norm[startIndex:])
ax[3].plot(bg_norm[startIndex:])
ax[4].plot(t.iloc[startIndex:, 0])
ax[5].plot(t.iloc[startIndex:, 1])
ax[6].plot(t.iloc[startIndex:, 2])
ax[7].plot(r.iloc[startIndex:, 0])
ax[8].plot(r.iloc[startIndex:, 1])
ax[9].plot(r.iloc[startIndex:, 2])
plt.show()


