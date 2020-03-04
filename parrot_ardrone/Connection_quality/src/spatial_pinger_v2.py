#!/usr/bin/env python3
import subprocess
#from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
#from math import log10
#import csv


## Converting datetime object to string
#dateTimeObj = datetime.now()
# 
#timestampStr = dateTimeObj.strftime("%d-%b-%Y (%H:%M:%S.%f)")
# 
#print('Current Timestamp : ', timestampStr)
#
#
#
#def savetocsv():
#    file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/Con_Qt.csv"
#    with open(file_path, mode='w') as csv_file:
#        fieldnames = ['emp_name', 'dept', 'birth_month']
#        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
#
#        writer.writeheader()
#        writer.writerow({'emp_name': 'John Smith', 'dept': 'Accounting', 'birth_month': 'November'})
#        writer.writerow({'emp_name': 'Erica Meyers', 'dept': 'IT', 'birth_month': 'March'})
#

    
def terminal_cap(args):
      # subprocess allow the execution of linux commands and it captures the output
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


# gh = Connectivity("192.168.1.1",10)
# gh = Connectivity("172.18.29.1",10)
gh = Connectivity("localhost")
#gh = Connectivity("172.16.0.14")
#gh = Connectivity("178.239.173.175") # VPN Gateway
#gh = Connectivity("192.168.43.76")

#====================
width = 4
w_step = 1

length = 4
l_step = 1

height = 1
h_step = 0.5
#====================

h_points = int((height-h_step)/h_step)
w_points = int((width-w_step)/w_step)
l_points = int((length-l_step)/l_step)

min_val = np.zeros((w_points,l_points))
max_val = np.zeros((w_points,l_points))
avg_val = np.zeros((w_points,l_points))
mdev_val = np.zeros((w_points,l_points))

pkg_transmit = np.zeros((w_points,l_points))
pkg_receive = np.zeros((w_points,l_points))
pkg_loss = np.zeros((w_points,l_points))
pkg_time = np.zeros((w_points,l_points))

link_number = np.zeros((w_points,l_points))
link_percent = np.zeros((w_points,l_points))
level_dBm = np.zeros((w_points,l_points))
level_mW = np.zeros((w_points,l_points))
noise_dBm = np.zeros((w_points,l_points))
noise_mW = np.zeros((w_points,l_points))

count = 0
export_dict = {}


for h in range(h_points):
    for w in range(w_points):
        for l in range(l_points):

#            try:
#                print("\nNext point:",(w,l,h),"[Enter]")
#                input("")
#            except SyntaxError:
#                pass
            
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
            
# Debugging prints
#print("packets transmitted > max:", np.amax(pkg_transmit), "| min: ", np.amin(pkg_transmit),"\n", pkg_transmit)
#print("packets received > max:", np.amax(pkg_receive),"| min: ", np.amin(pkg_receive),"\n", pkg_receive)
#print("packet loss > max:", np.amax(pkg_loss), "| min: ",np.amin(pkg_loss), "\n", pkg_loss)
#print("time > max:", np.amax(pkg_time), "| min: ", np.amin(pkg_time), "\n", pkg_time)
#
#print("min_val > max:",np.amax(min_val),"| min: ",np.amin(min_val),"\n",min_val)
#print("max_val > max:",np.amax(max_val),"| min: ",np.amin(max_val),"\n",max_val)
#print("avg_val > max:",np.amax(avg_val),"| min: ",np.amin(avg_val),"\n",avg_val)
#print("mdev_val > max:",np.amax(mdev_val),"| min: ",np.amin(mdev_val),"\n",mdev_val)

print(link_number)
print(link_percent)
print(level_dBm)
print(level_mW)
print(noise_dBm)
print(noise_mW)

print("Total points:", count)



# Print figures
# Fig 1
fig1 = plt.figure(figsize=(20,10))
a = fig1.add_subplot(2, 2, 1)
imgplot = plt.imshow(pkg_transmit)
a.set_title('packets transmitted')
plt.colorbar(ticks=[np.amin(pkg_transmit), ((np.amax(pkg_transmit)-np.amin(pkg_transmit))/2)+np.amin(pkg_transmit), np.amax(pkg_transmit)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(pkg_transmit)+np.amin(pkg_transmit))
imgplot.set_clim(np.amin(pkg_transmit)-(np.amax(pkg_transmit)-np.amin(pkg_transmit))*0.02, (np.amax(pkg_transmit)-np.amin(pkg_transmit))*0.02+np.amax(pkg_transmit))

a = fig1.add_subplot(2, 2, 2)
imgplot = plt.imshow(pkg_receive)
a.set_title('packets received')
plt.colorbar(ticks=[np.amin(pkg_receive), ((np.amax(pkg_receive)-np.amin(pkg_receive))/2)+np.amin(pkg_receive), np.amax(pkg_receive)], orientation='horizontal')
#imgplot.set_clim(np.amin(pkg_receive)-(np.amin(pkg_receive)*0.1), (np.amax(pkg_receive)*-0.1)+np.amax(pkg_receive))
imgplot.set_clim(np.amin(pkg_receive)-(np.amax(pkg_receive)-np.amin(pkg_receive))*0.02, (np.amax(pkg_receive)-np.amin(pkg_receive))*0.02+np.amax(pkg_receive))

a = fig1.add_subplot(2, 2, 3)
imgplot = plt.imshow(pkg_loss)
a.set_title('packet loss')
plt.colorbar(ticks=[np.amin(pkg_loss), ((np.amax(pkg_loss)-np.amin(pkg_loss))/2)+np.amin(pkg_loss), np.amax(pkg_loss)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(pkg_loss)+np.amin(pkg_loss))
imgplot.set_clim(np.amin(pkg_loss)-(np.amax(pkg_loss)-np.amin(pkg_loss))*0.02, (np.amax(pkg_loss)-np.amin(pkg_loss))*0.02+np.amax(pkg_loss))

a = fig1.add_subplot(2, 2, 4)
imgplot = plt.imshow(mdev_val)
a.set_title('time')
plt.colorbar(ticks=[np.amin(pkg_time), ((np.amax(pkg_time)-np.amin(pkg_time))/2)+np.amin(pkg_time), np.amax(pkg_time)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(mdev_val)+np.amin(mdev_val))
imgplot.set_clim(np.amin(pkg_time)-(np.amax(pkg_time)-np.amin(pkg_time))*0.02, (np.amax(pkg_time)-np.amin(pkg_time))*0.02+np.amax(pkg_time))


# Fig 2
fig2 = plt.figure(figsize=(20,10))
a = fig2.add_subplot(2, 2, 1)
imgplot = plt.imshow(min_val)
a.set_title('min_val')
plt.colorbar(ticks=[np.amin(min_val), ((np.amax(min_val)-np.amin(min_val))/2)+np.amin(min_val), np.amax(min_val)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(min_val)+np.amin(min_val))
imgplot.set_clim(np.amin(min_val)-(np.amax(min_val)-np.amin(min_val))*0.02, (np.amax(min_val)-np.amin(min_val))*0.02+np.amax(min_val))

a = fig2.add_subplot(2, 2, 2)
imgplot = plt.imshow(max_val)
a.set_title('max_val')
plt.colorbar(ticks=[np.amin(max_val), ((np.amax(max_val)-np.amin(max_val))/2)+np.amin(max_val), np.amax(max_val)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(max_val)+np.amin(max_val))
imgplot.set_clim(np.amin(max_val)-(np.amax(max_val)-np.amin(max_val))*0.02, (np.amax(max_val)-np.amin(max_val))*0.02+np.amax(max_val))

a = fig2.add_subplot(2, 2, 3)
imgplot = plt.imshow(avg_val)
a.set_title('avg_val')
plt.colorbar(ticks=[np.amin(avg_val), ((np.amax(avg_val)-np.amin(avg_val))/2)+np.amin(avg_val), np.amax(avg_val)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(avg_val)+np.amin(avg_val))
imgplot.set_clim(np.amin(avg_val)-(np.amax(avg_val)-np.amin(avg_val))*0.02, (np.amax(avg_val)-np.amin(avg_val))*0.02+np.amax(avg_val))

a = fig2.add_subplot(2, 2, 4)
imgplot = plt.imshow(mdev_val)
a.set_title('mdev_val')
plt.colorbar(ticks=[np.amin(mdev_val), ((np.amax(mdev_val)-np.amin(mdev_val))/2)+np.amin(mdev_val), np.amax(mdev_val)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(mdev_val)+np.amin(mdev_val))
imgplot.set_clim(np.amin(mdev_val)-(np.amax(mdev_val)-np.amin(mdev_val))*0.02, (np.amax(mdev_val)-np.amin(mdev_val))*0.02+np.amax(mdev_val))



# Fig 3
fig3 = plt.figure(figsize=(20,10))
a = fig3.add_subplot(3, 2, 1)
imgplot = plt.imshow(link_number)
a.set_title('Link quality (#/70)')
plt.colorbar(ticks=[np.amin(link_number), ((np.amax(link_number)-np.amin(link_number))/2)+np.amin(link_number), np.amax(link_number)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(pkg_transmit)+np.amin(pkg_transmit))
imgplot.set_clim(np.amin(link_number)-(np.amax(link_number)-np.amin(link_number))*0.02, (np.amax(link_number)-np.amin(link_number))*0.02+np.amax(link_number))

a = fig3.add_subplot(3, 2, 2)
imgplot = plt.imshow(link_percent)
a.set_title('Link quality (%)')
plt.colorbar(ticks=[np.amin(link_percent), ((np.amax(link_percent)-np.amin(link_percent))/2)+np.amin(link_percent), np.amax(link_percent)], orientation='horizontal')
#imgplot.set_clim(np.amin(pkg_receive)-(np.amin(pkg_receive)*0.1), (np.amax(pkg_receive)*-0.1)+np.amax(pkg_receive))
imgplot.set_clim(np.amin(link_percent)-(np.amax(link_percent)-np.amin(link_percent))*0.02, (np.amax(link_percent)-np.amin(link_percent))*0.02+np.amax(link_percent))

a = fig3.add_subplot(3, 2, 3)
imgplot = plt.imshow(level_dBm)
a.set_title('Signal level (dBm)')
plt.colorbar(ticks=[np.amin(level_dBm), ((np.amax(level_dBm)-np.amin(level_dBm))/2)+np.amin(level_dBm), np.amax(level_dBm)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(pkg_loss)+np.amin(pkg_loss))
imgplot.set_clim(np.amin(level_dBm)-(np.amax(level_dBm)-np.amin(level_dBm))*0.02, (np.amax(level_dBm)-np.amin(level_dBm))*0.02+np.amax(level_dBm))

a = fig3.add_subplot(3, 2, 4)
imgplot = plt.imshow(level_mW)
a.set_title('Signal level (mW)')
plt.colorbar(ticks=[np.amin(level_mW), ((np.amax(level_mW)-np.amin(level_mW))/2)+np.amin(level_mW), np.amax(level_mW)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(mdev_val)+np.amin(mdev_val))
imgplot.set_clim(np.amin(level_mW)-(np.amax(level_mW)-np.amin(level_mW))*0.02, (np.amax(level_mW)-np.amin(level_mW))*0.02+np.amax(level_mW))

a = fig3.add_subplot(3, 2, 5)
imgplot = plt.imshow(noise_dBm)
a.set_title('Noise level (dBm)')
plt.colorbar(ticks=[np.amin(noise_dBm), ((np.amax(noise_dBm)-np.amin(noise_dBm))/2)+np.amin(noise_dBm), np.amax(noise_dBm)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(mdev_val)+np.amin(mdev_val))
imgplot.set_clim(np.amin(noise_dBm)-(np.amax(noise_dBm)-np.amin(noise_dBm))*0.02, (np.amax(noise_dBm)-np.amin(noise_dBm))*0.02+np.amax(noise_dBm))

a = fig3.add_subplot(3, 2, 6)
imgplot = plt.imshow(noise_mW)
a.set_title('Noise level (mW)')
plt.colorbar(ticks=[np.amin(noise_mW), ((np.amax(noise_mW)-np.amin(noise_mW))/2)+np.amin(noise_mW), np.amax(noise_mW)], orientation='horizontal')
#imgplot.set_clim(0.0, np.amax(mdev_val)+np.amin(mdev_val))
imgplot.set_clim(np.amin(noise_mW)-(np.amax(noise_mW)-np.amin(noise_mW))*0.02, (np.amax(noise_mW)-np.amin(noise_mW))*0.02+np.amax(noise_mW))


