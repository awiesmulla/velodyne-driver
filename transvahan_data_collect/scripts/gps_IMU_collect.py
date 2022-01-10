#!/usr/bin/env python3

import numpy as np
import serial
import math
import os
import pandas as pd
import sys
import time
import matplotlib.pyplot as plt
from datetime import date
from witmotion import IMU

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


class Readgps():
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        #self.ser = serial.Serial('/dev/ttyACM0', 4800, timeout=5)
        #Constants used for conversion
        self.a = 6378137.0 #Radius of the earth
        self.odom_origin = np.array([0 , 0])
        #Counter for initializing GPS
        self.gps_counter = 0
        #Angular offset between odom and map frame
        #Multiplying rotation matrix to pose will rotate to axes anti-clockwise
        #We have to align x with East so rotate accordingly
        self.angle = np.deg2rad(0)
        self.rotation_matrix = np.array([[np.cos(self.angle), np.sin(self.angle)],
                                         [-np.sin(self.angle), np.cos(self.angle)]])
    
    def read_gps(self):
        """
        Read Lat, Long and altitude values
        """
        if self.gps_counter == 0:
            ser = serial.Serial('/dev/ttyACM0', 4800, timeout=5)
            line = ser.readline()
            line = line.decode('utf-8')
            splitline = line.split(',')
            print(line)
            print("--------------------------------------------")
        #ser = serial.Serial('/dev/ttyACM0', 4800, timeout=5)
        if splitline[0] == '$GPGGA':

            if splitline[6] == '0':
                print("Current GPS data invalid")
                self.gps_counter = 0

            else:
                lat = splitline[2]
                lat_deg = lat[:2] 
                lat_min = lat[2:] 
                latdirection = splitline[3]

                lon = splitline[4]
                lon_deg = lon[:3].lstrip("0")
                lon_min = lon[3:]
                londirection = splitline[5]

                num_satellites = splitline[7]

                self.latitude = int(lat_deg) + float(lat_min)/60
                self.longitude = int(lon_deg) + float(lon_min)/60
                self.altitude = float(splitline[9])
                #Convert to xyz
                self.conv_relative()
                if self.gps_counter == 0:
                    self.odom_origin[0] = self.x
                    self.odom_origin[1] = self.y
                    self.x = 0
                    self.y = 0
                else:
                    path.append(np.array([self.x, self.y, time.time()]))
                self.gps_counter += 1

    def conv_relative(self):
        '''
        Convert to relative coordinates using mercartor scale
        '''
        s = np.cos(self.latitude * np.pi/180)
        self.x = s * self.a * (np.pi*self.longitude/180)
        self.x = self.x - self.odom_origin[0]
        self.y = s * self.a * np.log(np.tan(np.pi*(90 + self.latitude)/360))
        self.y = self.y - self.odom_origin[1]
        temp = np.array([self.x, self.y])
        temp = np.dot(temp, self.rotation_matrix)
        
        if not self.gps_counter == 0:
            self.x = temp[0]
            self.y = temp[1]
        self.z = self.altitude

def main():

    global path
    path = []

    imu = IMU()
    acc_data = []
    yaw_rate = []
    time.sleep(1)
    imu_counter = 0

    Readgpsobj = Readgps()

    try:
        
        while True:
            temp = imu.get_acceleration()
            acc_data.append(temp)
            temp_yaw = imu.get_angular_velocity()
            yaw_rate.append([temp_yaw[2], time.time()])
            imu_counter = imu_counter + 1
            
            if imu_counter % 1 == 0:
                Readgpsobj.read_gps()
                continue
            time.sleep(0.1)

    except KeyboardInterrupt:

        df_acc = pd.DataFrame(acc_data)
        df_yaw = pd.DataFrame(yaw_rate)
        df_path = pd.DataFrame(path)
        df_acc.to_csv('./Data/' + str(date.today()) + '_' + sys.argv[1] + '_acc.csv')
        df_yaw.to_csv('./Data/' + str(date.today()) + '_' + sys.argv[1] + '_yaw.csv')
        df_path.to_csv('./Data/' + str(date.today()) + '_' + sys.argv[1] + '_gps.csv')

        acc_data = np.array(acc_data)
        yaw_rate = np.array(yaw_rate)
        path = np.array(path)

        if path.shape[0] == 0:
            print("No data recorded")

        fig, axes = plt.subplots(3, 2, figsize=(15,10))
        axes[0][0].plot(acc_data[:,0], '.')
        axes[0][0].grid()
        axes[0][0].set_ylabel('acc_x')

        axes[0][1].plot(lp_filter(acc_data[:,0]), '.')
        axes[0][1].grid()
        axes[0][1].set_ylabel('acc_x_filter')

        axes[1][0].plot(acc_data[:,1], '.')
        axes[1][0].grid()
        axes[1][0].set_ylabel('acc_y')

        axes[1][1].plot(lp_filter(acc_data[:,1]), '.')
        axes[1][1].grid()
        axes[1][1].set_ylabel('acc_y_filter')

        axes[2][0].plot(yaw_rate[:,0], '.')
        axes[2][0].grid()
        axes[2][0].set_ylabel('yaw_rate')

        axes[2][1].plot(hp_filter(yaw_rate[:,0]), '.')
        axes[2][1].grid()
        axes[2][1].set_ylabel('yaw_rate_filter')

        plt.show()

if __name__ == '__main__':
    main()