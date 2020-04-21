#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  14 22:12:24 2020
@author: ros

Ver: 1.1  
Changelog:
    - Columns Recieved Signal Strenght (mW) and Noise Level (mW) completely removed.

During the connectivity  experiment I faced some issues and in some cases I had
to stop the experiment and start later. However, the whole process takes a lot
of time, so I couldn't start everithng from the begining. For this reason, in 
these cases there are two different files, one containes the data before and one 
that contains data after the interuption. The puspose of this script is to merge
these two into one file, ensuring that the data has the right format.

The output data should have the following formats:
        
    NPY: list [
                packet_transmit         [np(11x11)] 
                packet_receive          [np(11x11)] 
                packet_loss_count       [np(11x11)] 
                packet_loss_rate        [np(11x11)]
                rtt_min                 [np(11x11)]
                rtt_avg                 [np(11x11)]
                rtt_max                 [np(11x11)]
                rtt_mdev                [np(11x11)]
                packet_duplicate_count  [np(11x11)] 
                packet_duplicate_rate   [np(11x11)]
                measurement_count       [np(11x11)]
                link_quality            [np(11x11)]
                link_quatity_amount     [np(11x11)]
                signal_dbm              [np(11x11)]
                noise_dbm               [np(11x11)]
                bitrate                 [np(11x11)]]
    
"""
import numpy as np
import sys
#np.set_printoptions(threshold=np.inf)
np.set_printoptions(threshold=sys.maxsize)
#np.set_printoptions(threshold=1000) #defult

#    Create a new list of all np.array variables filled with -999
newFile = np.full((18,4,11,11),fill_value = -999.9)

# === [ File paths ] === 

# Anafi 0.5m
anafi05_1 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_142351_(0.5m)_(0.0-4.2)_an.npy'
loadedData05_1 = np.load(anafi05_1)
anafi05_2 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_150126_(0.5m)_(4.2 -10.10)_an.npy'
loadedData05_2 = np.load(anafi05_2)

# Anafi 1.5m
anafi15_0 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_122853_(1.5m)_an.npy'
loadedData15_0 = np.load(anafi15_0)

# Anafi 2.5m
anafi25_1 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_090115_(2.5m)_(1m).npy'
loadedData25_1 = np.load(anafi25_1)
anafi25_2 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_101224_(2.5m)_(point 8.7 - 10.10)_an.npy'
loadedData25_2 = np.load(anafi25_2)

# Anafi 3.5m
anafi35_0 =  '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_105532_(3.5m)_an.npy'
loadedData35_0 = np.load(anafi35_0)

# ANAFI 0.5m data fixing
newFile[:,0,0:4,:] = loadedData05_1[:,0,0:4,:]
newFile[:,0,4:5,0:2] = loadedData05_1[:,0,4:5,0:2]

row_offset = 4
col_offset = 2

for row in range(7):            # Rows
    for col in range(11):       # Cols
        if row >= 6 and col > 8:
            break
        elif col <= 8:
            newFile[:,0,row_offset+row,col_offset+col] = loadedData05_2[:,0,row,col]
        else:
            newFile[:,0,row_offset+row+1,col-9] = loadedData05_2[:,0,row,col]

# ANAFI 1.5m data fixing
newFile[:,1,:,:] = loadedData15_0[:,0,:,:]

# ANAFI 2.5m data fixing
newFile[:,2,:,:] = loadedData25_1[:,0,:,:]
newFile[:,2,8,7:] = loadedData25_2[:,0,0,0:4]
temp = np.reshape(loadedData25_2[:,0],(18,121))[:,4:26].reshape((18,1,2,11))
newFile[:,2,9:,:] = temp[:,0,:,:]

# ANAFI 3.5m data fixing
newFile[:,3,:,:] = loadedData35_0[:,0,:,:]

# manual corrections:
# Layer 0 (0.5m)
newFile[1,0,4,1] = 24
newFile[2,0,4,1] = 1
newFile[3,0,4,1] = 4
newFile[11,0,4,1] = 64
newFile[12,0,4,1] = 91
newFile[13,0,4,1] = -46
newFile[14,0,4,1] = 25.12
newFile[17,0,4,1] = 130

# Layer 2 (2.5m)
newFile[1,2,8,6] = 25
newFile[2,2,8,6] = 0
newFile[3,2,8,6] = 0
newFile[4,2,8,6] = 8.513
newFile[5,2,8,6] = 41.299
newFile[6,2,8,6] = 123.969
newFile[7,2,8,6] = 40.275
newFile[11,2,8,6] = 67
newFile[12,2,8,6] = 95
newFile[13,2,8,6] = -43
newFile[14,2,8,6] = 50.12
newFile[17,2,8,6] = 130

#    Create a new list of all np.array variables filled with -999
newFile2 = np.full((16,4,11,11),fill_value = -999.9)

# Ardrone data merging (removing Recieved Signal Strenght (mW) and Noise Level (mW))
newFile2[0:14,:,:,:] = newFile[0:14,:,:,:]
newFile2[14,:,:,:] = newFile[15,:,:,:]
newFile2[15,:,:,:] = newFile[17,:,:,:]

## Export NPY 
#npyPath = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_150126_(full)_(ANAFI_FIXED_FINAL).npy"
#np.save(npyPath, newFile2)
#print("NPY Saved!")

#   Sel | Variable
# ==================================================
#  0    | Packets Transmitted (#)
#  1    | Packets Received (#)
#  2    | Packet Loss (#)
#  3    | Packet Loss (%)
#  4    | Minimum Time (ms)
#  5    | Avarage Time (ms)
#  6    | Maximun Time (ms)
#  7    | Mean Deviation (ms)
#  8    | Duplicate Packets (#)
#  9    | Duplicate Packets (%)
#  10   | Wifi Readings (#/Point)
#  11   | Recieved Signal Strenght Index (#/70)
#  12   | Recieved Signal Strenght (%)
#  13   | Recieved Signal Strenght (dBm)
#  14   | Noise Level (dBm)
#  15   | Bitrate (Mb/s)
# ------------------------------------------------

sel = 15
print("\nnewFile:\n",newFile2[sel])
#print("\nnewFile:\n",newFile[sel,2,10,1])


    
    
    
    
    
    
    
    
    


