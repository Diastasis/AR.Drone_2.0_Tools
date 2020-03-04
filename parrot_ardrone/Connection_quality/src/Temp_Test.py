
import telnetlib
import time
import re
#import json
from textwrap import dedent
import pingparsing


def drone2pc_pinger(telnet_host="192.168.1.1", target_ip="192.168.1.2", _count = 5):
    """Parse ping values from DRONE's terminal"""
#    TODO: Test if this funcion works as expected
    print("(DRONE): PINGING PC \t [{}]".format(target_ip))
    ping_pattern = re.compile(br'=\s?(\d+.?\d+)/(\d+.?\d+)/(\d+.?\d+)\s?\bms\b') #Set a regex to parse the data
    tn = telnetlib.Telnet(telnet_host)
    time.sleep(3)
    telnet_commant = "ping " + target_ip + " -c " + str(_count) + "\n"  # <-- Test if it works
    tn.write(bytes(telnet_commant, encoding='utf-8'))                   # <-- Test if it works
    OUTPUT = tn.expect([ping_pattern], 60)                                   # Time out in 60 Secs
    tn.close()                                                          # close the connection
    if OUTPUT[0] == -1:
        print("(DRONE): Unexpected ping values!")
        return { # Return error values in case of unexpected ping values
                "destination": "192.168.1.3", "packet_transmit": -999,"packet_receive": -999,
                "packet_loss_count": -999, "packet_loss_rate": -999, "rtt_min": -999,
                "rtt_avg": -999, "rtt_max": -999, "rtt_mdev": -999, "packet_duplicate_count": -999,
                "packet_duplicate_rate": -999}
    parser = pingparsing.PingParsing()
    OUTPUT = parser.parse(dedent(OUTPUT[2].decode('ascii')))
    return OUTPUT.as_dict() # return a dictionary

print(drone2pc_pinger())

#def drone_get_bitrate(HOST="192.168.1.1", interface="ath0" ):
##    TODO: Fix the redudant patterns
#    """Parse bitrate by iwlist ath0 bitrate command from the DRONE"""
##    pattern = re.compile(br'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
##    pattern2 = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
#    link_pattern = re.compile(br'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
#    link_pattern2 = re.compile(r'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
#
#    
#    # iwlist ath0 bitrate
#    tn = telnetlib.Telnet(HOST)
#    time.sleep(3)
#    tn.write(b"grep -i ath0 /proc/net/wireless\n")
#    bit_rate = tn.expect([link_pattern], 15)# Time out in 15 Secs
#    tn.close() # close the connection
#    
#    if bit_rate[0] == -1:
#        print("DRONE: Bitrate value does not have the right format.")
#        return -999
#    matches = link_pattern2.findall(bit_rate[2].decode('ascii'))
#    bit_rate = 0
#    for match in matches:
#        bit_rate = match#.group(1)
#    return round(float(bit_rate),2) #[2].decode('ascii'))
    




#def drone_wifi_stats(HOST="192.168.1.1"):
#    """Parse wifi values from DRONE. Only the link quality value is available, however."""
##    TODO: FIX THE CODE HERE
#    print("(DRONE) GETTING WIFI STATS [ath0]")
#    
##    TODO: Fix the redudant patterns
#    link_pattern = re.compile(br'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
#    link_pattern2 = re.compile(r'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
#    
#    # iwlist ath0 link quality
#    tn = telnetlib.Telnet(HOST)
#    time.sleep(3)
#    tn.write(b"grep -i ath0 /proc/net/wireless\n")
#    bit_rate = tn.expect([link_pattern], 15)# Time out in 15 Secs
#    tn.close() # close the connection
#    
#    if bit_rate[0] == -1:
#        print("DRONE: Bitrate value doesn't have the right format.")
#        return {"link_quality":-999}
#    matches = link_pattern2.findall(bit_rate[2].decode('ascii'))
#    bit_rate = 0
#    for match in matches:
#        bit_rate = match
#    return {"link_quality": round(float(bit_rate),2)} #[2].decode('ascii'))    



#    return float(bit_rate[2])
#drone_get_bitrate()
#print(drone_wifi_stats())

##=============================================================================================
#import telnetlib
#import time
#import re
#import json
#from textwrap import dedent
#import pingparsing 
#
#
#def drone_ping():
#    pattern = re.compile(br'=\s?(\d+.?\d+)/(\d+.?\d+)/(\d+.?\d+)\s?\bms\b')
##    issue = re.compile(br'$=\s?(\d+.?\d+)/(\d+.?\d+)/(\d+.?\d+)\s?\bms\b')
#    #print(type(pattern))
#    
#    
#    HOST = "192.168.1.1"
#    
#    tn = telnetlib.Telnet(HOST)
#    time.sleep(3)
#    tn.write(b"ping 192.168.1.2 -c 3\n")
#    #time.sleep(5)
#    OUTPUT = tn.expect([pattern], 15)# Time out in 15 Secs
#    tn.close()
#    
#    if OUTPUT[0] == -1:
#        print("DRONE: Ping does not return the right values.")
#        return {"destination": "192.168.1.3",
#                "packet_transmit": -999,
#                "packet_receive": -999,
#                "packet_loss_count": -999,
#                "packet_loss_rate": -999,
#                "rtt_min": -999,
#                "rtt_avg": -999,
#                "rtt_max": -999,
#                "rtt_mdev": -999,
#                "packet_duplicate_count": -999,
#                "packet_duplicate_rate": -999
#            }
#    
##    print('\nOUTPUT: ',OUTPUT[2].decode('ascii'))
#    parser = pingparsing.PingParsing()
#    OUTPUT = parser.parse(dedent(OUTPUT[2].decode('ascii')))
##    print(json.dumps(OUTPUT.as_dict(), indent=4))
#    return OUTPUT.as_dict()
#
#print(drone_ping())
#============================================================================================================
#matches = pattern.finditer(OUTPUT[1]) #.decode('ascii')
##
##
#for match in matches:
#    print(match)

#print(str(OUTPUT[2]).decode('ascii'))




#======================================================================================
#import subprocess
#
#import re
#
#args = ["iwlist","wlp2s0","bitrate"]
#my_string = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())
#print(my_string,"\n")
#
#pattern = re.compile(r'Current Bit Rate:(\d+\.?\d+)')
#
#matches = pattern.finditer(my_string)
#
#for match in matches:
#    print(match.group(1))

#
#iwlist wlp2s0 bitrate | grep "Current Bit Rate"
#
#def wifi_statistics(stats_count=3):
#    """Parse wifi values from terminal"""
#    link_sum,level_sum, noise_sum = 0, 0, 0
#    for i in range(stats_count):
#        args = ["grep", "-i", "wlp2s0","/proc/net/wireless"]
#        try:
#            link, level, noise = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())[16:35].replace(".","").strip().split()
#        except:
#            link, level, noise = -999, -999, -999
#            print("Unable to retrieve wifi data. Are you connected?") 
#        link_sum += int(link)
#        level_sum += int(level)
#        noise_sum += int(noise)
##   Take several measurements and calculate the average value for every point. 
#    link = link_sum / stats_count
#    level = level_sum / stats_count
#    noise = noise_sum / stats_count
#    return {
#            "measurement_count": stats_count,
#            "link_quality": link,
#            "link_quatity_amount": int(link*(10/7)),
#            "signal_dbm": level,
#            "signal_mw": round((10**(level/10.))*1000000,2),
#            "noise_dbm": noise,
#            "noise_mw": round((10**((noise)/10.))*1000000,2)
#            }

#======================================================================================================
##https://github.com/iancoleman/python-iwlist
#
#import iwlist
#import json
#
#content = iwlist.scan(interface='wlp2s0')
#cells = iwlist.parse(content)
#print(type(cells[0]))
#print(json.dumps(cells, indent=4))
#

# =====================================================================================================
#print(cells)

#import json
#import pingparsing
#import numpy as np
#import data_viewer as dv
#
## define the rooms diamensions (x,y,z -> width,length, height)
#width = 3
#w_step = 1
#
#length = 3
#l_step = 1
#
#height = 1
#h_step = 0.5
#
## Pre-calculate how many points required with the given dimentions
#h_points = int((height-h_step)/h_step)
#w_points = int((width-w_step)/w_step)
#l_points = int((length-l_step)/l_step)
#
## Initialiaze all fields to np arrays filled with zeros
#packet_transmit, packet_receive, packet_loss_count, packet_loss_rate, rtt_min, rtt_avg, rtt_max, rtt_mdev,  \
#packet_duplicate_count, packet_duplicate_rate, link_percent, level_dBm, level_mW, noise_dBm, \
#noise_mW = (np.zeros((w_points,l_points)) for i in range(15))
#
#point_data = [packet_transmit, packet_receive, packet_loss_count, packet_loss_rate, rtt_min, rtt_avg, rtt_max, rtt_mdev,  \
#packet_duplicate_count, packet_duplicate_rate, link_percent, level_dBm, level_mW, noise_dBm, noise_mW]
#
## PING
#ping_parser = pingparsing.PingParsing()
#transmitter = pingparsing.PingTransmitter()
#transmitter.destination = "178.239.173.175"
#transmitter.count = 3
#
## =========================
## Put data in a Dictionary
## =========================
#for h in range(h_points):
#    for w in range(w_points):
#        for l in range(l_points):
##            retreive the ping stats and put them in a dictionary
#            result = ping_parser.parse(transmitter.ping()).as_dict()
#            # Include the cordinates in the dict file
#            result['x'], result['y'],result['z'] = w, l, h
#            
#            packet_transmit[w,l] = result['packet_transmit']
#            packet_receive[w,l] = result['packet_receive']
#            packet_loss_count[w,l] = result['packet_loss_count']
#            packet_loss_rate[w,l] = result['packet_loss_rate']
#            rtt_min[w,l] = result['rtt_min']
#            rtt_avg[w,l] = result['rtt_avg']
#            rtt_max[w,l] = result['rtt_max']
#            rtt_mdev[w,l] = result['rtt_mdev']
#            packet_duplicate_count[w,l] = result['packet_duplicate_count']
#            packet_duplicate_rate[w,l] = result['packet_duplicate_rate']
#            
#            print(json.dumps(result, indent=4))
#
#dv.img_graphs(point_data)
#            stats_count[w,l] =  result['']
#            link_number[w,l] =  result['']
#            link_percent[w,l] =   result['']
#            level_dBm[w,l] =  result['']
#            level_mW[w,l] =  result['']
#            noise_dBm[w,l] =   result['']
#            noise_mW[w,l] =  result['']
            
#            for i,variable in enumerate(point data):
#                variable[w,l] = result['packet_transmit']
#                
#            
#            print(json.dumps(result, indent=4))
            
            
            
            
#            point_dict = {}
#            data_list = []
#            data_list += ping(str(ip_addr), count_ping)
#            data_list += wifi_stats(count_wifi)
#            """"Formulate a dictionary with the connection information"""
#            for i,element in enumerate(data_list):
#                point_dict[keyz[i]] = round(element,2) 
##            export_dict[(w,l,h)] = copy.deepcopy(point_dict) # de-aliasing - Make a deep copy 
#            export_dict[(w,l,h)] = copy.deepcopy(point_dict) # de-aliasing - Make a deep copy
##            print(export_dict)
#            """"convert a dictionary to a list o numpy arrays for figures generation"""        
#            for i, element in enumerate(fig_data):
#                element[w,l] = point_dict[keyz[i]]
#            count+=1







#=================================================================
#import numpy as np
#import matplotlib.pyplot as plt
#
## define the rooms diamensions (x,y,z -> width,length, height)
#width = 5
#w_step = 1
#
#length = 5
#l_step = 1
#
#height = 1
#h_step = 0.5
#
## Pre-calculate how many points required with the given dimentions
#h_points = int((height-h_step)/h_step)
#w_points = int((width-w_step)/w_step)
#l_points = int((length-l_step)/l_step)
#
##   Define the titles of the figures
#titles = ['Packets Transmitted (#)','Packets Received (#)', 
#        'Packet loss (%)','Total Time (ms)', 'Minimum Time (ms)','Maximun Time (ms)',
#        'Avarage Time (ms)','Mean Deviation (ms)', 'Wifi measurements (#/Point)',
#        'Link Quality (#/70)', 'Link Quality (%)','Signal Level (dBm)','Signal Level (mW)',
#        'Noise Level (dBm)', 'Noise Level (mW)']
#
#def img_graphs(fig_data, titles, exclude=[]):
#    """Generate graphs from numpy arrays"""
#    
##   ================================================================================================
#        # Initialiaze all fields to np arrays filled with zeros
#    axX = np.zeros((w_points))
#    axY = np.zeros((l_points))
#    
#    for w in range(w_points):
#        axX[w] = int(w)
#        for l in range(l_points):
#            axY[l] = int(l)
##    fig, ((ax, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 90))
#    fig = plt.figure(figsize=(16,90))
#    fig.tight_layout()
#    
#    for i in range(len(fig_data)):     
#        a = fig.add_subplot(int((len(fig_data)/2)+1), 2, i+1)
#        
#        imgplot = plt.imshow(fig_data[i], interpolation='catrom') # cmap="hot",  cmap='nipy_spectral',interpolation="nearest", "bicubic"
#        a.set_title(titles[i])
#        
#        #   fig, ax = plt.subplots()
##        im = ax.imshow(harvest,interpolation='catrom') # 'gaussian'
#        
#        # We want to show all ticks...
#        a.set_xticks(np.arange(len(axX)))
#        a.set_yticks(np.arange(len(axY)))
#        # ... and label them with the respective list entries
#        a.set_xticklabels(axX)
#        a.set_yticklabels(axY)
#        
#        ## Rotate the tick labels and set their alignment.
#        #plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
#        #         rotation_mode="anchor")
#        
#        # Loop over data dimensions and create text annotations.
#        for ii in range(len(axY)):
#            for j in range(len(axX)):
#                text = a.text(j, ii, fig_data[i][ii, j], ha="center", va="center", color="w")
#        
##        a.set_title("Harvest of local farmers (in tons/year)")
#        
##        plt.show() 
##   ================================================================================================ 
#        
#        fig.suptitle('Level 1', fontsize=16) # +str(level)+"/"+str(l_points)
#        if i == 2:      # Packet loss(%) 
#            plt.colorbar(ticks=[0, 50, 100], orientation='horizontal')
#            imgplot.set_clim(0, 100)  
#        elif i == 9:    # Link Quality (#/70)
#            plt.colorbar(ticks=[0, 35, 70], orientation='horizontal')
#            imgplot.set_clim(0, 70)
#        elif i == 10:   # Link Quality (%) OR Packet loss(%)
#            plt.colorbar(ticks=[0, 50, 100], orientation='horizontal')
#            imgplot.set_clim(0, 100)
#        elif i == 11:   # Link Quality (%) OR Packet loss(%)
#            plt.colorbar(ticks=[-30, -65, -100], orientation='horizontal')
#            imgplot.set_clim(-30, -100)
#        elif i == 13:   # Noise Level (dBm)
#            plt.colorbar(ticks=[-20, -138, -256], orientation='horizontal')
#            imgplot.set_clim(-20, -256)
#        else:           
#            plt.colorbar(ticks=[np.amin(fig_data[i]), ((np.amax(fig_data[i])-np.amin(fig_data[i]))/2)+np.amin(fig_data[i]), np.amax(fig_data[i])], orientation='horizontal')
#            imgplot.set_clim(np.amin(fig_data[i])-(np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02, (np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02+np.amax(fig_data[i]))
#    plt.show()
#
#
#file_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/ConQlt_200220.153036.npy'
#fig_data = np.load(file_path)
#img_graphs(fig_data,titles)


#===========================================================================
#import matplotlib.pyplot as plt
#import numpy as np
#
#
#def func3(x, y):
#    return (1 - x / 2 + x**5 + y**3) * np.exp(-(x**2 + y**2))
#
#
## make these smaller to increase the resolution
#dx, dy = 0.05, 0.05
#
#x = np.arange(-3.0, 3.0, dx)
#y = np.arange(-3.0, 3.0, dy)
#X, Y = np.meshgrid(x, y)
#
## when layering multiple images, the images need to have the same
## extent.  This does not mean they need to have the same shape, but
## they both need to render to the same coordinate system determined by
## xmin, xmax, ymin, ymax.  Note if you use different interpolations
## for the images their apparent extent could be different due to
## interpolation edge effects
#
#extent = np.min(x), np.max(x), np.min(y), np.max(y)
#fig = plt.figure(frameon=False)
#
#Z1 = np.add.outer(range(8), range(8)) % 2  # chessboard
#im1 = plt.imshow(Z1, cmap=plt.cm.gray, interpolation='nearest',
#                 extent=extent)
#
#Z2 = func3(X, Y)
#
#im2 = plt.imshow(Z2, cmap=plt.cm.viridis, alpha=.9, interpolation='bilinear',
#                 extent=extent)
#
#plt.show()

#==================================================================================================
#import numpy as np
##import matplotlib
#import matplotlib.pyplot as plt
## sphinx_gallery_thumbnail_number = 2
#
## define the rooms diamensions (x,y,z -> width,length, height)
#width = 8
#w_step = 1
#
#length = 8
#l_step = 1
#
#height = 1
#h_step = 0.5
#
#count = 0
#export_dict = {}
#
#
## Pre-calculate how many points required with the given dimentions
#w_points = int((width-w_step)/w_step)
#l_points = int((length-l_step)/l_step)
#h_points = int((height-h_step)/h_step)
#
## Initialiaze all fields to np arrays filled with zeros
#axX = np.zeros((w_points))
#axY = np.zeros((l_points))
#
#
#for h in range(h_points):
#    for w in range(w_points):
#        axX[w] = int(w)
#        for l in range(l_points):
#            axY[l] = int(l)
#
#
#
#harvest = np.array([[0.8, 2.4, 2.5, 3.9, 0.0, 4.0, 0.0],
#                    [2.4, 0.0, 4.0, 1.0, 2.7, 0.0, 0.0],
#                    [1.1, 2.4, 0.8, 4.3, 1.9, 4.4, 0.0],
#                    [0.6, 0.0, 0.3, 0.0, 3.1, 0.0, 0.0],
#                    [0.7, 1.7, 0.6, 2.6, 2.2, 6.2, 0.0],
#                    [1.3, 1.2, 0.0, 0.0, 0.0, 3.2, 5.1],
#                    [0.1, 2.0, 0.0, 1.4, 0.0, 1.9, 6.3]])
#
#
#fig, ax = plt.subplots()
#im = ax.imshow(harvest,interpolation='catrom') # 'gaussian'
#
## We want to show all ticks...
#ax.set_xticks(np.arange(len(axX)))
#ax.set_yticks(np.arange(len(axY)))
## ... and label them with the respective list entries
#ax.set_xticklabels(axX)
#ax.set_yticklabels(axY)
#
### Rotate the tick labels and set their alignment.
##plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
##         rotation_mode="anchor")
#
## Loop over data dimensions and create text annotations.
#for i in range(len(axY)):
#    for j in range(len(axX)):
#        text = ax.text(j, i, harvest[i, j],
#                       ha="center", va="center", color="w")
#
#ax.set_title("Harvest of local farmers (in tons/year)")
#fig.tight_layout()
#plt.show()


##=========================================
#from pydub import AudioSegment
#from pydub.playback import play
#
#sound = AudioSegment.from_file("siren.wav", format="wav")
#play(sound)
#

#import numpy as np
#
#w_points = 3
#l_points = 3
#
#
#min_val, max_val, avg_val, mdev_val = (np.zeros((int(w_points),int(l_points))) for i in range(4)) 
#
#
#
#
#print('min: \n',min_val)
#min_val[0,0] = 99
#print('min: \n',min_val)
#print('max: \n',max_val)
#max_val[0,2] = 99
#print('max: \n',max_val)
#print('max: \n',avg_val)
#print('max: \n',mdev_val)
#=======================================================================
#import matplotlib.pyplot as plt
#import numpy as np
#
#
#def func3(x, y):
#    return (1 - x / 2 + x**5 + y**3) * np.exp(-(x**2 + y**2))
#
## make these smaller to increase the resolution
#dx, dy = 0.05, 0.05
#
#x = np.arange(-3.0, 3.0, dx)
#y = np.arange(-3.0, 3.0, dy)
#X, Y = np.meshgrid(x, y)
#
#extent = np.min(x), np.max(x), np.min(y), np.max(y)
#fig = plt.figure(frameon=False)
#
#Z1 = np.add.outer(range(8), range(8)) % 2  # chessboard
#
#im1 = plt.imshow(Z1, cmap=plt.cm.gray, interpolation='nearest',
#                 extent=extent)
#
#Z2 = func3(X, Y)
#
#im2 = plt.imshow(Z2, cmap=plt.cm.viridis, alpha=.9, interpolation='bilinear',
#                 extent=extent)
#
#plt.show()
#==========================================================================
#import matplotlib.pyplot as plt
#import numpy as np
#
#Z = np.random.rand(6, 10)
#
#fig, (ax0, ax1) = plt.subplots(2, 1)
#
#c = ax0.pcolor(Z)
#ax0.set_title('default: no edges')
#
#c = ax1.pcolor(Z, edgecolors='k', linewidths=4)
#ax1.set_title('thick edges')
#
#fig.tight_layout()
#plt.show()