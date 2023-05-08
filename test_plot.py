# Import libraries
import os 
import sys
import numpy as np
import matplotlib.pyplot as plt
 

def distance_p2p(x1, y1, a, b, c):
    d = abs((a * x1 + b * y1 + c)) / (np.sqrt(a * a + b * b))
    return d

# Import Data
current_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(current_dir, 'Data')
sys.path.append(data_dir)
from data5 import *
from waypoint import *

path = waypointPose4

add_data = 0
size_data = len(data)
size_path = len(path)
# print(size_data)

# Data Inizialitation
x = [None] * (size_data + add_data)
y = [None] * (size_data + add_data)
theta = [None] * (size_data + add_data)
u = [None] * (size_data + add_data)
v = [None] * (size_data + add_data)

path_x = [None] * len(path)
path_y = [None] * len(path)

RMSE = 0

# Parsing and Processing Data
for i in range(size_data):
    x[i] = 10*data[i][0]
    y[i] = 10*data[i][1]
    theta[i] = data[i][2]
    u[i] = np.cos(data[i][2])
    v[i] = np.sin(data[i][2])

for i in range(size_path):
    path_x[i] = 10 * path[i][0]
    path_y[i] = 10 * path[i][1]


# Error Analysis
for i in range(size_data):
    # error analysis p2p
    RMSE += distance_p2p(x[i],y[i],1, 1, 0)

    # error analysis path tracking
    # MSE += (x[i]-path_x[i])**2+(y[i]-path_y[i])**2
    # RMSE += np.sqrt(MSE)
    
RMSE = RMSE/size_data
print(RMSE)

# Movement 
for i in range(len(x)):
    if i in [0, len(x)-1]:
        continue
    length = np.sqrt(u[i]**2 + v[i]**2)
    width = 0.005 # def = 0.005
    hal = hl = 1. / width * length
    plt.quiver(x[i], y[i], u[i], v[i],
            angles='xy', 
            scale_units='xy', 
            color='#b2b2b2',
            scale=0.5,

            headwidth=hl*0.5,
            headaxislength=hal*0.5,
            headlength=hl*0.6,
            width=width*0.5)

## === Overall - Path Tracking === ###

plt.plot(path_x,path_y, ls='dashed')

# ### === Overall - Point to Point === ###
# # Path P2P
# x_path = [x[0],x[-1]]
# y_path = [y[0],y[-1]]

# plt.plot(x_path, y_path, ls='dashed', c='#000000')

# # Initial and Target Plot 
# data_add_x = [x[0],x[-1]]
# data_add_y = [y[0],y[-1]]
# data_add_u = [u[0],u[-1]]
# data_add_v = [v[0],v[-1]]

# for i in range(len(data_add_x)):
#     length = np.sqrt(u[i]**2 + v[i]**2)
#     width = 0.005 # def = 0.005
#     hal = hl = 1. / width * length
#     plt.quiver(data_add_x[i], data_add_y[i], data_add_u[i], data_add_v[i],
#             angles='xy', 
#             scale_units='xy', 
#             color='#000000',
#             scale=0.5,

#             headwidth=hl*0.5,
#             headaxislength=hal*0.5,
#             headlength=hl*0.6,
#             width=width*0.5)

# ### === End of Overall - Point to Point === ###


# plt.suptitle("Point to point - LQR Controller")
plt.suptitle("Path Tracking - LQR Controller")
# plt.title("Kp = [10, 10, 7], Ki = [0, 0, 0], Kd = [0, 0, 0] | RMSE = %1.3f" %(RMSE))
# plt.title("Q = diag(75, 75, 35) R = diag(1,1,1) | RMSE = %1.3f" %(RMSE))

# plt.title("Kp = [15, 15, 10], Ki = [0, 0, 0], Kd = [0, 0, 0], Offset = 5")
plt.title("Q = diag(225, 225, 105), R = diag(1,1,1), Offset =  5")

plt.xlabel("x-position")
plt.ylabel("y-position")
plt.axis([-20, 20, -20, 20])
plt.grid()
plt.show()



# ### === Each Axis Analysis - Point to Point === ###
# plt.subplot(3, 1, 1)
# plt.title("Point to point - PID Controller")
# plt.axhline(y = 15, color = 'r', linestyle = 'dashed')
# plt.plot(x[:40])
# plt.axis([0, 40, -20, 20])
# plt.ylabel("x-position")


# plt.subplot(3, 1, 2)
# plt.axhline(y = 15, color = 'r', linestyle = 'dashed')
# plt.plot(y[:40])
# plt.axis([0, 40, -20, 20])
# plt.ylabel("y-position")

# plt.subplot(3, 1, 3)
# plt.axhline(y = -180, color = 'r', linestyle = 'dashed')
# plt.plot(np.rad2deg(theta[:40]))
# plt.axis([0, 40, 0, -200])
# plt.ylabel("theta angle")
# plt.xlabel("iteration")

# plt.show()


