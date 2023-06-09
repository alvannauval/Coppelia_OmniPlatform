#!/usr/bin/env python3

import time
import keyboard
import numpy as np
import sys
import sim

import numpy as np
import matplotlib
import scipy.linalg as la

from waypoint import *

print("Done.")

# === Function Definition ===
# ---------------------------
def connectSimulator():
    # Close all opened connections
    sim.simxFinish(-1)
    # Connect to CoppeliaSim
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Connection not successful")
        sys.exit("Could not connect")
    return clientID


def setObjectHandler(clientID, path):
    objectHandler = sim.simxGetObjectHandle(clientID, path, sim.simx_opmode_blocking)[1]
    return objectHandler


def getObjectPose(clientID, objectHandle, block=None, robot=None):
    if(block == None):
        position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_oneshot)[1]
        orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_oneshot)[1]
    else:
        position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_blocking)[1]
        orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_blocking)[1]

    # print("Pose of: x = {:.2f} y = {:.2f} theta = {:.2f}".format(position[0], position[1], orientation[2]*(180/np.pi)))
    if(robot == True): # transformating axis
        orientation = sim.simxGetObjectOrientation(clientID, robotOrientationHandler, -1, sim.simx_opmode_oneshot)[1]
        orientation[2] = transformAngle(orientation[2])
    
    return position[0], position[1], orientation[2]


def transformAngle(angle):
    if(angle > np.deg2rad(0) and angle < np.deg2rad(180)):
        angle = np.deg2rad(180) - angle
    elif(angle < np.deg2rad(0) and angle > np.deg2rad(-180)):
        angle = -(np.deg2rad(180) + angle)
    return angle


def setRobotMotion(clientID, motors, velocity):
    sim.simxSetJointTargetVelocity(clientID, motors[0], velocity[0], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, motors[1], velocity[1], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, motors[2], velocity[2], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, motors[3], velocity[3], sim.simx_opmode_oneshot)


def inverseKinematics(local_speed):

    L = 0.5 # distance to wheel
    alpha_1 = np.deg2rad(-135)
    alpha_2 = np.deg2rad(-45)
    alpha_3 = np.deg2rad(45)
    alpha_4 = np.deg2rad(135)

    V = np.array([[local_speed[0]],
                  [local_speed[1]],
                  [local_speed[2]]])

    A = np.array([[-np.sin(alpha_1), np.cos(alpha_1), -L],
                  [-np.sin(alpha_2), np.cos(alpha_2), -L],
                  [-np.sin(alpha_3), np.cos(alpha_3), -L],
                  [-np.sin(alpha_4), np.cos(alpha_4), -L]])

    Phi = np.matmul(A, V)

    # # Speed Hard Limiter
    # for i in range(4):
    #     if(Phi[i] >= 15):
    #         Phi[i] = 15
    #     elif(Phi[i] <= -15):
    #         Phi[i] = -15

    # Speed Normalizer
    if(np.max(Phi) >= 20):
        max_speed = np.max(Phi)
        normalization_factor = 20 / max_speed
        Phi = normalization_factor * Phi
        # print(max_speed)
        # print(normalization_factor)   

    # print("Phi0 = {} Phi1 = {} Phi2 = {} Phi3 = {}".format(Phi[0], Phi[1], Phi[2], Phi[3]))    
    return float(Phi[0]), float(Phi[1]), float(Phi[2]), float(Phi[3])


def keyboardRoutine(speed):

    localSpeed = [0, 0, 0]
    # Horizontal and Vertical
    if keyboard.is_pressed('w'):
        localSpeed += np.array([0, 1, 0])
    if keyboard.is_pressed('x'):
        localSpeed += np.array([0, -1, 0])
    if keyboard.is_pressed('a'):
        localSpeed += np.array([-1, 0, 0])
    if keyboard.is_pressed('d'):
        localSpeed += np.array([1, 0, 0])

    # Diagonal
    if keyboard.is_pressed('q'):
        localSpeed += np.array([-1, 1, 0])
    if keyboard.is_pressed('e'):
        localSpeed += np.array([1, 1, 0])
    if keyboard.is_pressed('z'):
        localSpeed += np.array([-1, -1, 0])
    if keyboard.is_pressed('c'):
        localSpeed += np.array([1, -1, 0])

    # Rotation
    if keyboard.is_pressed('r'):
        localSpeed += np.array([0, 0, 1])
    if keyboard.is_pressed('t'):
        localSpeed += np.array([0, 0, -1])

    localSpeed = speed*np.array(localSpeed)
    print(localSpeed)

    return localSpeed


def globalToLocal(globalSpeed, heading):
    localSpeed = [0, 0, 0]

    localSpeed[0] = globalSpeed[0]*np.cos(heading) + globalSpeed[1]*np.sin(heading)
    localSpeed[1] = -globalSpeed[0]*np.sin(heading) + globalSpeed[1]*np.cos(heading)
    localSpeed[2] = globalSpeed[2]

    return localSpeed

def movePID(robotPose, waypoint):

    globalSpeed = [0, 0, 0]

    proportional = [0, 0, 0]
    error = [0, 0, 0]
    kp = [15, 15, 10] # P2P : 10 10 7 # Tracking 15 15 10
    ki = [0, 0, 0]
    kd = [0, 0, 0]

    # print("target[2] = {:.2f} | pose[2] = {:.2f}".format(waypoint[2]*180/np.pi, robotPose[2]*180/np.pi))
    # P controller # todo pid controller
    for i in range(3):
        error[i] = waypoint[i] - robotPose[i]

        # Nearest Angle
        if(i == 2 and abs(error[2]) >= np.pi):
            if(error[2] > 0):
                error[2] = error[2] - 2*np.pi
            else:
                error[2] = error[2] + 2*np.pi

        proportional[i] = kp[i] * error[i]

        globalSpeed[i] = proportional[i]

    return globalSpeed

def moveLQR(robotPose, waypoint, dt):

    localSpeed = [0, 0, 0]

    error_x     = waypoint[0] - robotPose[0]
    error_y     = waypoint[1] - robotPose[1]
    error_theta = waypoint[2] - robotPose[2]

    # nearest angle
    if(abs(error_theta) >= np.pi):
        if(error_theta > 0):
            error_theta = error_theta - 2*np.pi
        else:
            error_theta = error_theta + 2*np.pi

    # print("x = {:.4f} x_target = {:.4f} error_x = {:.4f}".format(robotPose[0], waypoint[0], error_x))
    # print("y = {:.4f} y_target = {:.4f} error_y = {:.4f}".format(robotPose[1], waypoint[1], error_y))
    # print("theta = {:.4f} theta_target = {:.4f} error_theta = {:.4f}".format(robotPose[2], waypoint[2], error_theta))
 
    A = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]]) # State Variable Matrix (Identity)
    B = np.array([[-np.cos(robotPose[2])*dt,  np.sin(robotPose[2])*dt,    0],
                  [-np.sin(robotPose[2])*dt, -np.cos(robotPose[2])*dt,    0],
                  [                       0,                        0, 1*dt]]) # Input Control Matrix
    Q = np.array([[75, 0, 0], # P2P: 75 75 35 | Tracking 225 225 105
                  [0, 75, 0],
                  [0, 0, 75]]) # State Variable Cost Matrix
    Q = Q * 3
    R = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]]) # Input Control Cost Matrix

    E = np.array([[error_x],
                  [error_y],
                  [error_theta]]) # Error Matrix

    # Method 1
    # P = la.solve_discrete_are(A, B, Q, la.inv(R)) # Solve Ricatti Equation
    # K = la.inv(R) @ B.T @ P # Solve LQR Gain

    # Method 2
    P = la.solve_discrete_are(A, B, Q, la.inv(R))
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)

    u = -K @ E # Solve Input Control

    u = list(u.flatten())
    u[2] *= -1
    localSpeed = u

    # print(P)
    # print(K)
    # print(u)

    return localSpeed


def purePursuit(robotPose, waypoints, offset):

    target = robotPose
    epsilon = 0.1

    # pure pursuit
    if(purePursuit.wp < len(waypoints)-1):
        target = waypoints[purePursuit.wp]
        while(np.linalg.norm(np.array(target[0:2]) - np.array(robotPose[0:2])) < offset and purePursuit.wp < len(waypoints)-1):
            target = waypoints[purePursuit.wp]
            purePursuit.wp += 1

    elif(purePursuit.wp == len(waypoints)-1):
        target = waypoints[purePursuit.wp]
        if((np.linalg.norm(np.array(target[0:3]) - np.array(robotPose[0:3])) < epsilon)):
            purePursuit.count += 1
        else:
            purePursuit.count = 0
        if(purePursuit.count == 25):
            purePursuit.wp += 1
            purePursuit.count = 0

    else:
        pass
    
    # print("wp = ", purePursuit.wp, "/", len(waypoints), "target = ", np.round(target,2), "count = ", purePursuit.count)

    return list(target)
purePursuit.wp = 0
purePursuit.count = 0


# === Main Program ===
# --------------------
print("Program started")
# --- Connection to Coppelia Simulator ---
client_id = connectSimulator()
 
# --- Get Object Handler ---
robotHandler = setObjectHandler(client_id, "/OmniPlatform")
robotOrientationHandler = setObjectHandler(client_id, "/OmniPlatform/link[0]")
wheelHandler = [None]*4
for i in range(4):
    wheelHandler[i] = setObjectHandler(client_id, "/OmniPlatform/link[" + str(i) + "]/regularRotation")

waypointHandler = [None]*2
for i in range(2):
    waypointHandler[i] = setObjectHandler(client_id, "/Cylinder[" + str(i) + "]")

# --- Get initial Pose ---
robotPose = getObjectPose(client_id, robotHandler, block=True, robot=True)
waypointPose = [None]*10
waypointPose[0] = getObjectPose(client_id, waypointHandler[0], block=True)

# --- Initial Param ---
samp_time = 0.1
n = 1
time_start = time.time()

wheelsPhi = [0, 0, 0, 0]
localSpeed = [0, 0, 0]
globalSpeed = [0, 0, 0]
target = [0, 0, 0]

mode = 5

time_a = time.time()
time_b = time.time()

while True:
    t_now = time.time() - time_start
    if t_now >= samp_time*n:
        # Update time
        n += 1
        dt = time_b - time_a
        time_a = time.time()

        if(mode == 1): # Keyboard control
            localSpeed = keyboardRoutine(speed=8)
        
        elif(mode == 2): # Move to point PID
            globalSpeed = movePID(robotPose, waypointPose[0])
            localSpeed = globalToLocal(globalSpeed, robotPose[2])

        elif(mode == 3): # Pure Pursuit PID
            target = purePursuit(robotPose, waypointPose3, 0.5)
            globalSpeed = movePID(robotPose, target)
            localSpeed = globalToLocal(globalSpeed, robotPose[2])

        elif(mode == 4): # move to point LQR
            localSpeed = moveLQR(robotPose, waypointPose[0], dt)

        elif(mode == 5): # Pure Pursuit LQR
            target = purePursuit(robotPose, waypointPose3, 0.5)
            localSpeed = moveLQR(robotPose, target, dt)

        wheelsPhi = inverseKinematics(localSpeed)
        setRobotMotion(client_id, wheelHandler, wheelsPhi)

        # Update poses
        robotPose = getObjectPose(client_id, robotHandler, robot=True)
        waypointPose[0] = getObjectPose(client_id, waypointHandler[0])
        time_b = time.time()

        # print("err_x = {} err_y = {} err_theta = {}".format(waypointPose[0][0] - robotPose[0], waypointPose[0][1] - robotPose[1], waypointPose[0][2] - robotPose[2]))
        # print("Global Speed: {:.2f}, {:.2f}, {:.2f} | Wheels Phi: {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(globalSpeed[0], globalSpeed[1], globalSpeed[2], wheelsPhi[0], wheelsPhi[1], wheelsPhi[2], wheelsPhi[3]))
        # print("Time = {} dt = {:.2e} | Robot Pose: {:.2f}, {:.2f}, {:.2f}\n".format(round(t_now,2), dt, robotPose[0], robotPose[1], robotPose[2]*(180/np.pi)))

        # Buat Ambil Data
        print("[{:.4f},{:.4f},{:.4f}],".format(robotPose[0],robotPose[1],robotPose[2]))
    
# --- End of Simulation ---
sim.simxFinish(client_id)
print("Program ended")