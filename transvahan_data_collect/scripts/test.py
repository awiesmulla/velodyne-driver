#!/usr/bin/env python3

import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from witmotion import IMU
from datetime import date


def lp_filter(arr, alpha = 0.02):
    fil_arr = [arr[0]]
    for a in arr[1:]:
        fil_arr.append(alpha*a+(1-alpha)*fil_arr[-1])
    return fil_arr

def hp_filter(arr, alpha = 0.02):
    fil_arr = [arr[0]]
    for i in range(arr.shape[0]):
        if i==0:
            continue
        fil_arr.append(alpha*fil_arr[-1]+(1-alpha)*(arr[i]-arr[i-1]))
    return fil_arr

def prior(x, u, z, P, sigma_u, R, dt):

    temp_x = x + (u * dt)

    G = np.eye(3)
    G = G * dt
    
    F = np.eye(3)

    Q = np.dot(G, sigma_u)
    Q = np.dot(Q, np.transpose(G))

    temp_p = np.dot(F, P)
    temp_p = np.dot(temp_p, np.transpose(F))
    temp_p = temp_p + Q

    H = np.array([[1,0,0],
                  [0,1,0]])
    h = np.dot(H, np.transpose(temp_x))

    K = np.dot(temp_p, np.transpose(H))
    temp = np.dot(H, temp_p)
    temp = np.dot(temp, np.transpose(H))
    temp = np.linalg.pinv(temp + R)
    K = np.dot(K, temp)

    P = np.dot((np.eye(3) - np.dot(K, H)), temp_p)
    x = temp_x + np.dot(K, (z - h))

    return x, P

def main():

    data_acc = np.genfromtxt("./Data/2021-12-14/2021-12-14_run2_acc.csv",delimiter=',')
    data_yaw = np.genfromtxt("./Data/2021-12-14/2021-12-14_run2_yaw.csv",delimiter=',')
    data_gps = np.genfromtxt("./Data/2021-12-14/2021-12-14_run2_gps.csv",delimiter=',')

    a_x = lp_filter(data_acc[1:,1])
    a_y = lp_filter(data_acc[1:,2])
    w_z = (data_yaw[1:,1])
    data_gps = data_gps[1:]

    path = []
    vel = []
    gps = []

    x = np.array([0.0,0.0,0.0])
    u = np.array([0.0,0.0,0.0])
    z = np.array([0.0,0.0])
    dt = 0.1

    P = np.eye(3)
    P = P * 0.027

    sigma_u = np.eye(3)
    sigma_u = sigma_u * 2#0.027
    sigma_u[2][2] = 0.027

    R = np.eye(3)
    R = 0.01

    path.append(x)
    vel.append(u)

    index = 0

    for i in range(1400):
        
        temp = np.array([a_x[i], a_y[i], w_z[i]])
        u = u + (dt * temp)
        u[2] = w_z[i]
        
        if i % 11 == 0:
            z = data_gps[index,1:3]
            gps.append(z)
            x, P = prior(x, u, z, P, sigma_u, R, dt)
            index = index + 1
        
        else:
            x = x + ((u * dt) + (0.5 * temp * dt * dt))
        
        
        path.append(x)
        vel.append(u)

    path = np.array(path)
    vel = np.array(vel)
    gps = np.array(gps)
    plt.plot(path[:,0],path[:,1],'.', color="blue")
    plt.plot(gps[:,0],gps[:,1], '.',color="red")
    #plt.plot(path[:,2],'.')

    plt.show()

if __name__ == '__main__':
    main()