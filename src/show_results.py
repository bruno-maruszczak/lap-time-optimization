from visualiser import Visualiser
from mpc.track import Track
import os
import json
import matplotlib.pyplot as plt
import numpy as np
from mpc.model import VehicleModel


track = Track("MX-5", "buckmore", "curvature", 1000)
model = VehicleModel(os.path.join(os.getcwd(), "data", "vehicles", "MX5.json"), track)
_ = Visualiser(track, os.path.join(os.getcwd(), "sim_results.json"))

sim_data = {}
with open(os.path.join(os.getcwd(), "sim_results.json"), 'r') as f:
    data = json.load(f)
    for key, value in data.items():
        sim_data[key] = np.array(value)

n = len(sim_data["x"])
states = sim_data["x"]
rs = states[:, 5, 0]
ss = states[:, 0, 0]
ks = model.k(ss)
print(ks)
Fy_fs = sim_data["Fy"][:, 0]
Fy_rs = sim_data["Fy"][:, 1]
alphas_f = sim_data["alpha"][:, 0]
alphas_r = sim_data["alpha"][:, 1]


fig, ax = plt.subplots(6, figsize=(16, 9))
fig.suptitle("Internal Params")

ax[0].plot(ks)
ax[0].set_ylabel('k')
ax[1].plot(alphas_f)
ax[1].set_ylabel('af')
ax[2].plot(alphas_r)
ax[2].set_ylabel('ar')
ax[3].plot(Fy_fs)
ax[3].set_ylabel('Fy_f')
ax[4].plot(Fy_rs)
ax[4].set_ylabel('Fy_r')
ax[5].plot(rs)
ax[5].set_ylabel('r')


plt.show()