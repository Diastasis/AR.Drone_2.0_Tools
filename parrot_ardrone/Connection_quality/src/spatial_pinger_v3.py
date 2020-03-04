#!/usr/bin/env python3

# ==========================
# Imports
# ==========================

import subprocess
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import csv
#from mpl_toolkits import mplot3d
#from math import log10

# =========================
# Mode selection variables:
# =========================

# Enable debugging
dbg_mode = False
# Enable figure generation
fig_mode = True
# Enable manual mode
manual_mode = True



def get_timestamp():
    # Converting datetime object to string
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%d%m%y.%H%M%S")
    return timestampStr

def save_dict2csv(list_of_dict):
    file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/ConQlt_{}.csv".format(get_timestamp())
    with open(file_path, mode='w') as csv_file:
        fieldnames = ['x','y','z','packets transmitted', 'received', 'packet loss','time', 'min', 'max', 'avg', 'mdev','link_number','link_percent', 'level_dBm','level_mW', 'noise_dBm', 'noise_mW']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        
        for element in list_of_dict:
            writer.writerow(element)
    print('File saved!')

    
def terminal_cap(args):
#   subprocess allow the execution of linux commands and it captures the output
    return subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate()


class Connectivity:
    def __init__(self, ip):
        self.ip = str(ip)
    
    def ping(self, ping_count=3):
#        ping command
        args = ["ping","-c", str(ping_count), self.ip]
        output = terminal_cap(args)

        if "Name or service not known" in str(output[1]):
            print("Invalid IP address")
            return {}
        elif "Network is unreachable" in str(output[1]):
            print("Network is unreachable")
            return {}
        elif "Destination Host Unreachable" in str(output[0]):
            print("Destination Host Unreachable")
            return {}
        else:
#            extract useful data from the output string
            output = str(output[0]).split("--- " + self.ip + " ping statistics ---")[1].replace("'","").replace("ms","").replace("%","").strip("\\n").split("\\n")

#        store pakages info in a dictionary
        ping_values = {}
        for word in output[0].split(","):
            word1, word2 = word.strip().split(" ",1)
            if word1.isnumeric():
                ping_values[word2] = int(word1)
            else:
                ping_values[word1] = int(word2)
                
        if len(output) < 2:
            print("\nDictionary: ", ping_values)
            return {}
        
#        store ping stats in the dictionary
        names = output[1].replace("ms","").replace("rtt","").replace(" ","").strip(" ").split("=")[0].split("/")
        values = output[1].replace("ms","").replace("rtt","").replace(" ","").strip(" ").split("=")[1].split("/")
        for i, name in enumerate(names):
            try:
                ping_values[name] = float(values[i])
            except ValueError:
                ping_values[name] = -1.0
                
        return ping_values
    
    def wifi_stats(self,stats_count=3):
        cc = 0
        link_sum = 0
        level_sum = 0
        noise_sum = 0
            
        for i in range(stats_count):
            args = ["grep", "-i", "wlp2s0","/proc/net/wireless"]
            link, level, noise = str(terminal_cap(args))[16:35].replace(".","").strip().split()
#            link quality = link/70, Signal level = dBm, noise = floor noise (dBm) | dBm to mW = 10 ⋅ log10( P(mW) / 1mW)
            link_sum += int(link)
            level_sum += int(level)
            noise_sum += int(noise)
            cc += 1

        link = link_sum / cc
        level = level_sum / cc
        noise = noise_sum / cc
        
        stats = {"link_number":link,"link_percent":int(link*(10/7)), "level_dBm":level, "level_mW":round((10**(level/10.))*1000000,2), "noise_dBm":noise, "noise_mW":round((10**((noise)/10.))*1000000,2)}
        return stats
#End of class Connectivity

#================================
# Start
#================================

# gh = Connectivity("192.168.1.1",10)
# gh = Connectivity("172.18.29.1",10)
gh = Connectivity("localhost")
#gh = Connectivity("172.16.0.14")
#gh = Connectivity("178.239.173.175") # VPN Gateway
#gh = Connectivity("192.168.43.76")
#====================

# define the rooms diamensions (x,y,z -> width,length, height)
width = 4
w_step = 1

length = 4
l_step = 1

height = 1
h_step = 0.5

# Pre-calculate how many points required with the given dimentions
h_points = int((height-h_step)/h_step)
w_points = int((width-w_step)/w_step)
l_points = int((length-l_step)/l_step)

# Variable init - Initialiaze all fields to np arrays filled with zeros
min_val, max_val, avg_val, mdev_val = (np.zeros((w_points,l_points)) for i in range(4)) 
pkg_transmit, pkg_receive, pkg_loss, pkg_time = (np.zeros((w_points,l_points)) for i in range(4)) 
link_number, link_percent, level_dBm, level_mW, noise_dBm, noise_mW = (np.zeros((w_points,l_points)) for i in range(6)) 

count = 0
export_dict = {}
export_list = []



# =========================
# Put data in a Dictionary
# =========================

for h in range(h_points):
    for w in range(w_points):
        for l in range(l_points):
            
            if manual_mode == True:
                try:
                    print("\nNext point:",(w,l,h),"[Enter]")
                    input("")
                except SyntaxError:
                    pass
            
            temp = gh.ping(3)
            print(temp)
            temp2 = gh.wifi_stats(3)
            print(temp2)
            
            
            try:
                pkg_transmit[w,l] = temp["packets transmitted"] # Number
                pkg_receive[w,l] = temp["received"]             # Number
                pkg_loss[w,l] = temp["packet loss"]             # %
                pkg_time[w,l] = temp["time"]                    # msec
            
                min_val[w,l] = temp["min"]                      # msec
                max_val[w,l] = temp["max"]                      # msec
                avg_val[w,l] = temp["avg"]                      # msec
                mdev_val[w,l] = temp["mdev"]                    # msec
                
                link_number[w,l] = temp2["link_number"]         # #/70
                link_percent[w,l] = temp2["link_percent"]       # %
                level_dBm[w,l] = temp2["level_dBm"]             # dBm
                level_mW[w,l] = temp2["level_mW"]               # mW
                noise_dBm[w,l] = temp2["noise_dBm"]             # dBm
                noise_mW[w,l] = temp2["noise_mW"]               # mW
                
            except TypeError:
                pkg_transmit[w,l] = -1.0                        
                pkg_receive[w,l] = -1.0                         
                pkg_loss[w,l] = -1.0                            
                pkg_time[w,l] = -1.0                            
            
                min_val[w,l] = -1.0
                max_val[w,l] = -1.0
                avg_val[w,l] = -1.0
                mdev_val[w,l] = -1.0
                
                link_number[w,l] = -1.0         
                link_percent[w,l] = -1.0       
                level_dBm[w,l] = -1.0             
                level_mW[w,l] = -1.0               
                noise_dBm[w,l] = -1.0             
                noise_mW[w,l] = -1.0               
                
            except KeyError:
                pkg_transmit[w,l] = -1.0                        
                pkg_receive[w,l] = -1.0                         
                pkg_loss[w,l] = -1.0                            
                pkg_time[w,l] = -1.0                            
            
                min_val[w,l] = -1.0
                max_val[w,l] = -1.0
                avg_val[w,l] = -1.0
                mdev_val[w,l] = -1.0
            
                link_number[w,l] = -1.0         
                link_percent[w,l] = -1.0       
                level_dBm[w,l] = -1.0             
                level_mW[w,l] = -1.0               
                noise_dBm[w,l] = -1.0             
                noise_mW[w,l] = -1.0
                
            export_dict[(w,l,h)] = temp
            count+=1
            

print("Total points:", count)


title = [
        'Packets Transmitted (#)','Packets Received (#)', 'Packet loss (%)','Total Time (ms)',
        'Minimum Time (ms)','Maximun Time (ms)','Avarage Time (ms)','Mean Deviation (ms)',
        'Link Quality (#/70)', 'Link Quality (%)','Signal Level (dBm)','Signal Level (mW)',
        'Noise Level (dBm)', 'Noise Level (mW)'
         ]
    
data =  [
           pkg_transmit, pkg_receive, pkg_loss, pkg_time, min_val, max_val, avg_val, 
           mdev_val,link_number, link_percent, level_dBm, level_mW, noise_dBm, noise_mW
        ]

# ===========================
# Print figures
# ===========================

if fig_mode == True:
    fig = plt.figure(figsize=(16,90))
    for i in range(len(data)):
        a = fig.add_subplot(int((len(data)/2)+1), 2, i+1)
        imgplot = plt.imshow(data[i])
        a.set_title(title[i])
        plt.colorbar(ticks=[np.amin(data[i]), ((np.amax(data[i])-np.amin(data[i]))/2)+np.amin(data[i]), np.amax(data[i])], orientation='horizontal')
        imgplot.set_clim(np.amin(data[i])-(np.amax(data[i])-np.amin(data[i]))*0.02, (np.amax(data[i])-np.amin(data[i]))*0.02+np.amax(data[i]))

# ===================
# Debugging prints
# ===================

if dbg_mode == True:
    print("\n ================  DEBUGGING IS ENABLED  ==================")
    for i in range(len(data)):
        print("\n",title[i],":\n",(len(title[i])+2)*"-","\n", data[i])




