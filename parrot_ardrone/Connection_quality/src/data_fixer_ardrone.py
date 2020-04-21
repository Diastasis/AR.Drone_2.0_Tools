#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 22:50:24 2020

@author: ros

During the connectivity  experiment I faced some issues and in some cases I had
to stop the experiment and cantinue later. However, the whole process takes a lot
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
                signal_mw               [np(11x11)]
                noise_dbm               [np(11x11)]
                noise_mw                [np(11x11)]
                bitrate                 [np(11x11)]
            ]
    
"""
import numpy as np
import sys
#np.set_printoptions(threshold=np.inf)
np.set_printoptions(threshold=sys.maxsize)
#np.set_printoptions(threshold=1000) #defult

#    Create a new list of all np.array variables filled with -999
newFile = np.full((18,4,11,11),fill_value = -999.9)

# === [ File paths ] === 

# Ardrone 0.5m
ardrone05_0 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_160320_115629_(0.5m)_(AR).npy'
loadedData05_0 = np.load(ardrone05_0)

# Ardrone 1.5m
ardrone15_0 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_160320_115629_(1.5m)_(AR).npy'
loadedData15_0 = np.load(ardrone15_0)

# Ardrone 2.5m
ardrone25_0 = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(AR_2.5).npy'
loadedData25_0 = np.load(ardrone25_0)

# Ardrone 3.5m
ardrone35_0 =  '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_170320_121610_(3.5m)_(AR).npy'
loadedData35_0 = np.load(ardrone35_0)


# Ardrone data merging
newFile[:,0,:,:] = loadedData05_0[:,0,:,:]
newFile[:,1,:,:] = loadedData15_0[:,1,:,:]
newFile[:,2,:,:] = loadedData25_0[:,0,:,:]
newFile[:,3,:,:] = loadedData35_0[:,0,:,:]


 Export NPY 
npyPath = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(full)_(ARDRONE_FIXED2).npy"
np.save(npyPath, newFile)
print("NPY Saved!")
#
##   Sel | Variable
## ==================================================
##  0    | Packets Transmitted (#)
##  1    | Packets Received (#)
##  2    | Packet Loss (#)
##  3    | Packet Loss (%)
##  4    | Minimum Time (ms)
##  5    | Avarage Time (ms)
##  6    | Maximun Time (ms)
##  7    | Mean Deviation (ms)
##  8    | Duplicate Packets (#)
##  9    | Duplicate Packets (%)
##  10   | Wifi Readings (#/Point)
##  11   | Recieved Signal Strenght Index (#/70)
##  12   | Recieved Signal Strenght (%)
##  13   | Recieved Signal Strenght (dBm)
##  14   | Recieved Signal Strenght (mW)
##  15   | Noise Level (dBm)
##  16   | Noise Level (mW)
##  17   | Bitrate (Mb/s)
## ------------------------------------------------

sel = 4
print("\nnewFile:\n",newFile[sel])
print("\nnewFile:\n",newFile.shape)
#print("\nnewFile:\n",newFile[sel,2,10,1])
#print("\nloadedData25_1\n",loadedData25_1[sel])
#print("\nloadedData25_2\n",loadedData25_2[sel])

    
    
    
    
    
    
    
    
    


