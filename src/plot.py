#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import os

os.chdir("./cmake/build")
data = np.genfromtxt("./data.csv", delimiter="\t", skip_header=0)
print(data)
print(data.shape)
time = data[:,0]
ep0 = data[:,1]
ep1 = data[:,2]
ep2 = data[:,3]

pc0 = data[:,4]
pc1 = data[:,5]
pc2 = data[:,6]


pd0 = data[:,7]
pd1 = data[:,8]
pd2 = data[:,9]


# plt.figure()
# plt.plot(time,ep0,Label="ep x")
# plt.plot(time,ep1,Label="ep y")
# plt.plot(time,ep2,Label="ep z")


# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Error in position (EE)")
# plt.title("Error  - time")


# plt.figure()
# plt.plot(time,pc0,Label="p x")
# plt.plot(time,pc1,Label="p y")
# plt.plot(time,pc2,Label="p z")


# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Actual position (EE)")
# plt.title("Actual  - time")

# plt.figure()
# plt.plot(time,pd0,Label="pd x")
# plt.plot(time,pd1,Label="pd y")
# plt.plot(time,pd2,Label="pd z")


# plt.legend()
# plt.xlabel("t_real")
# plt.ylabel("Desired position (EE)")
# plt.title("Desired  - time")



plt.figure()
plt.plot(time,pc0,Label="p x")
plt.plot(time,pc1,Label="p y")
plt.plot(time,pc2,Label="p z")

plt.plot(time,pd0,Label="pd x")
plt.plot(time,pd1,Label="pd y")
plt.plot(time,pd2,Label="pd z")


plt.legend()
plt.xlabel("t_real")
plt.ylabel("Desired position (EE)")
plt.title("Desired  - time")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')
