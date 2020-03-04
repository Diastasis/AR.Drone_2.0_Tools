#!/usr/bin/env python3

import subprocess
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import csv
import copy # deep copy

# =========================
# Mode selection variables:
# =========================

# Enable debugging
dbg_mode = True
# Enable figure generation
fig_mode = True
# Enable manual mode
manual_mode = False
# Save data in a CSV file
savecsv_mode = True


def terminal_capture(args):
      """subprocess allow the execution of linux commands and it captures the output"""
      return subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate()
  
def ping(ip, ping_count=3):
    """Capture ping values from terminal"""
    args = ["ping","-c", str(ping_count), str(ip)]
    output = terminal_capture(args)
    ping_values = [-999,-999,-999,-999,-999,-999,-999,-999]
    
#   if errors occure return -999 value for all ping values
    if "Name or service not known" in str(output[1]):
        print("Invalid IP address.")
        return ping_values # -999
    elif "Network is unreachable" in str(output[1]):
        print("Network is unreachable.")
        return ping_values # -999
    elif "Destination Host Unreachable" in str(output[0]):
        print("Destination Host Unreachable.")
        return ping_values # -999
    else:
#       extract useful data from the terminal output
        output = str(output[0]).split("--- " + str(ip) + " ping statistics ---")[1].replace("'","").replace("ms","").replace("%","").strip("\\n").split("\\n")
        if len(output) < 2:
            print("Ping output has not the right format.")
            return ping_values # -999
    
    ping_values = []
#   extract package data        
    for word in output[0].split(","):
        first, second = word.strip().split(" ",1)
        if first.isnumeric():
            ping_values.append(int(first))
        else:
            ping_values.append(int(second))

#   extract statistics
    words = output[1].strip(" ").split("=")
    for i, word in enumerate(words[1].split("/")):            
        try:
            ping_values.append(float(word))
        except ValueError:
            ping_values.append(-999) 
    return ping_values # Right values

def wifi_stats(stats_count=3):
    """Capture wifi values from terminal"""
    link_sum = 0
    level_sum = 0
    noise_sum = 0
        
    for i in range(stats_count):
        args = ["grep", "-i", "wlp2s0","/proc/net/wireless"]
        try:
            link, level, noise = str(terminal_capture(args))[16:35].replace(".","").strip().split()
        except:
            link = -999
            level = -900
            noise = -900
            print("Unable to retrieve wifi data. Are you connected?")
            
        link_sum += int(link)
        level_sum += int(level)
        noise_sum += int(noise)
        
#   Take several measurements and calculate the average value for every point. 
    link = link_sum / stats_count
    level = level_sum / stats_count
    noise = noise_sum / stats_count
    return [stats_count, link, int(link*(10/7)), level, round((10**(level/10.))*1000000,2), \
            noise, round((10**((noise)/10.))*1000000,2)]

def get_timestamp():
    """Return date and time as string"""
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%d%m%y.%H%M%S")
    return timestampStr

def save_dict2csv(data, headers):
    """Save a dicionary to a CSV file. Imput: a dictionary and a list of headers"""
    file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/csv/ConQlt_{}.csv".format(get_timestamp())
    with open(file_path, mode='w') as csv_file:
        temp1=copy.deepcopy(headers) # de-aliasing - Make a deep copy
        temp1.insert(0,"x")       # Add the points since they weren't included in the key list
        temp1.insert(1,"y")
        temp1.insert(2,"z")
        writer = csv.DictWriter(csv_file, fieldnames = temp1)
        writer.writeheader()
        temp2 = copy.deepcopy(data)  # de-aliasing - Make a deep copy 
        for key, value in temp2.items():
            value["x"]=key[0]       # Add the points since they weren't included in dictionary
            value["y"]=key[1]
            value["z"]=key[2]
            writer.writerow(value)
    print('\nCSV File saved!')
    
    
    

def save_pickle():
    pass

def save_png():
    pass
#
#def dict2np(data, keys):
#    """"convert a dictionary to a list o numpy arrays for figures generation"""
#    # Initialiaze all fields to np arrays filled with zeros
#    min_val, max_val, avg_val, mdev_val, pkg_transmit, pkg_receive, pkg_loss, \
#    pkg_time, link_number, link_percent, level_dBm, level_mW, noise_dBm, \
#    noise_mW, stats_count = (np.zeros((w_points,l_points)) for i in range(15))
#    
#    var_list = [ pkg_transmit, pkg_receive, pkg_loss, pkg_time, min_val, max_val, avg_val, 
#                   mdev_val, stats_count, link_number, link_percent, level_dBm, level_mW, 
#                   noise_dBm, noise_mW ]
#
#    for key,value in data.items():
#        x,y,z = key
#        for i, element in enumerate(var_list):
#            element[x,y] = value[keys[i]]
#    return var_list
    
#================================
# Start
#================================
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

count = 0
export_dict = {}

keyz = ['pkg_tx','pkg_rx', 'pkg_loss','pkg_time',
        'min_time','max_time','avg_time','mdev_time','stats_count', 'link_num', 
        'link_per', 'level_dbm', 'level_mw', 'noise_dbm', 'noise_mw']

# Initialiaze all fields to np arrays filled with zeros
min_val, max_val, avg_val, mdev_val, pkg_transmit, pkg_receive, pkg_loss, \
pkg_time, link_number, link_percent, level_dBm, level_mW, noise_dBm, \
noise_mW, stats_count = (np.zeros((w_points,l_points)) for i in range(15))

fig_data = [ pkg_transmit, pkg_receive, pkg_loss, pkg_time, min_val, max_val, avg_val, 
               mdev_val, stats_count, link_number, link_percent, level_dBm, level_mW, 
               noise_dBm, noise_mW ]


# =========================
# Put data in a Dictionary
# =========================
point_dict = {}
for h in range(h_points):
    for w in range(w_points):
        for l in range(l_points):
            
            if manual_mode == True:
                try:
                    print("\nNext point:",(w,l,h),"[Enter]")
                    input("")
                except SyntaxError:
                    pass
                
            data_list = []
            data_list += ping("localhost",3)
            data_list += wifi_stats(5)

            """"Formulate a dictionary with the connection information"""
            for i,element in enumerate(data_list):
                point_dict[keyz[i]] = element 
#            print("\nPoint values",[w,l,h],"-->", point_dict)
            export_dict[(w,l,h)] = copy.deepcopy(point_dict) # de-aliasing - Make a deep copy 
            
            """"convert a dictionary to a list o numpy arrays for figures generation"""        
            for i, el in enumerate(fig_data):
                el[w,l] = point_dict[keyz[i]]
            count+=1
print("\nTotal points:", count)

if 
save_dict2csv(export_dict, keyz)



# ===========================
# Print figures
# ===========================
titles = [
        'Packets Transmitted (#)','Packets Received (#)', 
        'Packet loss (%)','Total Time (ms)', 'Minimum Time (ms)','Maximun Time (ms)',
        'Avarage Time (ms)','Mean Deviation (ms)', 'Wifi measurements (#/Point)',
        'Link Quality (#/70)', 'Link Quality (%)','Signal Level (dBm)','Signal Level (mW)',
        'Noise Level (dBm)', 'Noise Level (mW)'
         ]

if fig_mode == True:
    fig = plt.figure(figsize=(16,90))
    for i in range(len(fig_data)):
        a = fig.add_subplot(int((len(fig_data)/2)+1), 2, i+1)
        imgplot = plt.imshow(fig_data[i])
        a.set_title(titles[i])
#        if fig_data[i] == 
        plt.colorbar(ticks=[np.amin(fig_data[i]), ((np.amax(fig_data[i])-np.amin(fig_data[i]))/2)+np.amin(fig_data[i]), np.amax(fig_data[i])], orientation='horizontal')
        imgplot.set_clim(np.amin(fig_data[i])-(np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02, (np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02+np.amax(fig_data[i]))


# ===================
# Debugging prints
# ===================
if dbg_mode == True:
    print("\n ================  DEBUGGING IS ENABLED  ==================")
    for i in range(len(fig_data)):
        print("\n",titles[i],":\n",(len(titles[i])+2)*"-","\n", fig_data[i])




