
# https://pypi.org/project/pingparsing/

import json
import pingparsing
import numpy as np
import subprocess
from pydub import AudioSegment # play 
from pydub.playback import play
from datetime import datetime
import csv
import iwlist # https://github.com/iancoleman/python-iwlist
import re
from textwrap import dedent
import time
import telnetlib
#import data_viewer_v2
#import copy # deep copy


#================================
# Variables definition
#================================
# Flag variables:
dbg_mode = False     # Enable debugging
fig_mode = True      # Enable figure generation
manual_mode = False  # Enable manual mode
savecsv_mode = True  # Save data in a CSV file
sound_mode = False   # Enable sound notification
savejson_mode = True # Save data to a JSON file
savenpy_mode = True  # Save data to a npy binary file

# =============================================
# Function Definition
# =============================================
def pc_wifi_stats(stats_count=3):
    """Parse wifi values from terminal"""
#    link_sum,level_sum, noise_sum = 0,0,0
    link_sum2,level_sum2, noise_sum2 = [], [], [] #0,0,0
    args = ["grep", "-i", "wlp2s0","/proc/net/wireless"]
    for _ in range(stats_count):
        try:
            link, level, noise = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())[16:35].replace(".","").strip().split()
        except:
            print("Unable to retrieve wifi data. Are you connected?") 
            return {"measurement_count": stats_count, "link_quality": -999,"link_quatity_amount": -999,
            "signal_dbm": -999, "signal_mw": -999, "noise_dbm": -999, "noise_mw": -999}
        
#        # Sum up the returned values:    
#        link_sum += int(link)
#        level_sum += int(level)
#        noise_sum += int(noise)

        # Sum up the returned values:    
        link_sum2.append(int(link))
        level_sum2.append(int(level))
        noise_sum2.append(int(noise))
        
##   Calculate the average value for every point. 
#    link = link_sum / stats_count
#    print("link1: ", link)
#    level = level_sum / stats_count
#    noise = noise_sum / stats_count

#   Calculate the average value for every point. 
    link2 = sum(link_sum2)/len(link_sum2)
    print("link2: ", link2)
    
    level2 = sum(level_sum2)/len(level_sum2)
    print("level2: ", level2) #level_sum / stats_count
    
    noise2 = sum(noise_sum2)/len(noise_sum2)
    print("noise2: ", noise2) #noise_sum / stats_count
    
    
    return {
            "measurement_count": stats_count,
            "link_quality": link,
            "link_quatity_amount": int(link*(10/7)),
            "signal_dbm": level,
            "signal_mw": round((10**(level/10.))*1000000,2),
            "noise_dbm": noise,
            "noise_mw": round((10**((noise)/10.))*1000000,2)
            }
def pc_get_bitrate(interface="wlp2s0" ):
    """Parse bitrate by iwlist wlp2s0 bitrate command from the PC"""
    pattern = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
    
    args = ["iwlist",interface,"bitrate"]
    my_string = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())   
    matches = pattern.finditer(my_string)
    bit_rate = 0
    for match in matches:
        bit_rate = match.group(1)
    return float(bit_rate)

def drone_get_bitrate(HOST="192.168.1.1", interface="ath0" ):
#    TODO: Fix the redudant patterns
    """Parse bitrate by iwlist wlp2s0 bitrate command from the DRONE"""
    pattern = re.compile(br'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
    pattern2 = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
    
    # iwlist ath0 bitrate
    tn = telnetlib.Telnet(HOST)
    time.sleep(3)
    tn.write(b"iwlist ath0 bitrate\n")
    bit_rate = tn.expect([pattern], 15)# Time out in 15 Secs
    tn.close() # close the connection
    
    if bit_rate[0] == -1:
        print("DRONE: Bitrate value does not have the right format.")
        return -999
    matches = pattern2.findall(bit_rate[2].decode('ascii'))
    bit_rate = 0
    for match in matches:
        bit_rate = match#.group(1)
    return round(float(bit_rate),2) # returns just the 
        
def drone2pc_ping(HOST="192.168.1.1"):
    pattern = re.compile(br'=\s?(\d+.?\d+)/(\d+.?\d+)/(\d+.?\d+)\s?\bms\b') #Set a regex to parse the data
    tn = telnetlib.Telnet(HOST)
    time.sleep(3)
    tn.write(b"ping 192.168.1.2 -c 3\n")
    OUTPUT = tn.expect([pattern], 15)# Time out in 15 Secs
    tn.close() # close the connection
    if OUTPUT[0] == -1:
        print("DRONE: Ping does not return the right values.")
        return {"destination": "192.168.1.3",
                "packet_transmit": -999,
                "packet_receive": -999,
                "packet_loss_count": -999,
                "packet_loss_rate": -999,
                "rtt_min": -999,
                "rtt_avg": -999,
                "rtt_max": -999,
                "rtt_mdev": -999,
                "packet_duplicate_count": -999,
                "packet_duplicate_rate": -999
            }
    parser = pingparsing.PingParsing()
    OUTPUT = parser.parse(dedent(OUTPUT[2].decode('ascii')))
#    return json.dumps(OUTPUT.as_dict(), indent=4) # return data in JSON format 
    return OUTPUT.as_dict() # return a dictionary
    
    
#  ==========[ Main ]==========
def main():  
    # define the rooms diamensions (x,y,z -> width,length, height)
    # TODO: Add user promption in case of zero value in w_point/h_point/l_point 
    width = 3.2
    w_step = 0.7
    
    length = 2.2
    l_step = 0.7
    
    height = 1
    h_step = 0.5
    
    # Pre-calculate how many points required with the given dimentions
    if width < w_step:
        w_points = 0
    elif (width - 0.5) < w_step: # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        w_points = 1
    else:
        w_points = int((width - 0.5)/w_step)+1# The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        
    if length < l_step:
        l_points = 0
    elif (length - 0.5) < l_step: # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        l_points = 1
    else:
        l_points = int((length - 0.5)/l_step)+1 # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        
    if height < h_step:
        h_points = 0
    elif (height - 0.5) < h_step: # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        h_points = 1
    else:
        h_points = int((height - 0.5)/h_step)+1 # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        
        
        # Initialiaze all fields to np arrays filled with zeros
    packet_transmit, packet_receive, packet_loss_count, packet_loss_rate, \
    rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_duplicate_count, packet_duplicate_rate, \
    measurement_count, link_quality, link_quatity_amount, signal_dbm, signal_mw, noise_dbm,\
    noise_mw, bitrate = (np.zeros((w_points,l_points)) for i in range(18))
    
    # Create a list of all np.array variables
    point_data = [packet_transmit, packet_receive, packet_loss_count, packet_loss_rate, \
                  rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_duplicate_count, packet_duplicate_rate, \
                  measurement_count, link_quality, link_quatity_amount, signal_dbm, signal_mw, noise_dbm, noise_mw, bitrate]
    
    #   Define the titles of the figures
    titles = ['Packets Transmitted (#)','Packets Received (#)', 
            'Packet Loss (#)','Packet Loss (%)', 'Minimum Time (ms)','Avarage Time (ms)','Maximun Time (ms)',
            'Mean Deviation (ms)','Duplicate Packets (#)', 'Duplicate Packets (%)', 'Wifi Measurements (#/Point)',
            'Link Quality (#/70)', 'Link Quality (%)','Signal Level (dBm)','Signal Level (mW)',
            'Noise Level (dBm)', 'Noise Level (mW)','Bit rate (Mb/s)']
    
    # Creat ping objects
    ping_parser = pingparsing.PingParsing()
    transmitter = pingparsing.PingTransmitter()
    transmitter.destination ="178.239.173.175" # "178.239.173.175" #'192.168.1.1' 
    transmitter.count = 3
    
    # Load sound file
    sound = AudioSegment.from_file("siren.wav", format="wav")
    # File paths
    time_stamp = datetime.now().strftime("%d%m%y_%H%M%S")
    csv_file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/csv/ConQlt_{}.csv".format(time_stamp)
    json_file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/JSON/ConQlt_{}.json".format(time_stamp)
    #npy_file_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/ConQlt_{}.npy'.format(time_stamp)
    npy = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/'
    
    content = iwlist.scan(interface='wlp2s0') # Wifi module
    
    pc_output_list = []
    drone_output_list = []
    output_dict = {}
    
    for h in range(h_points):
        for w in range(w_points):
            for l in range(l_points):
                if sound_mode:
                    play(sound)
                if manual_mode:
                    try:
                        print("\nNext point:",(w,l,h),"Press [Enter] to start")
                        input("")
                    except SyntaxError:
                        pass
                result = {'z':round((h*h_step)+h_step,2),'x':round((w*w_step)+w_step,2),'y':round((l*l_step)+l_step,2)} # Add coordinate points    
                result.update(ping_parser.parse(transmitter.ping()).as_dict())    # Retreive the ping stats and put them in a dictionary
                result.update(pc_wifi_stats())                                  # Retreive wifi statistics
                result['bitrate'] = pc_get_bitrate()                                 # Retrieve bitrate
                
    #            Save ping data in the coresponding variables
                packet_transmit[w,l] =      round(result['packet_transmit'],2)
                packet_receive[w,l] =       round(result['packet_receive'],2)
                packet_loss_count[w,l] =    round(result['packet_loss_count'],2)
                packet_loss_rate[w,l] =     round(result['packet_loss_rate'],2)
                rtt_min[w,l] =              round(result['rtt_min'],2)
                rtt_avg[w,l] =              round(result['rtt_avg'],2)
                rtt_max[w,l] =              round(result['rtt_max'],2)
                rtt_mdev[w,l] =             round(result['rtt_mdev'],2)
                packet_duplicate_count[w,l] = round(result['packet_duplicate_count'],2)
                packet_duplicate_rate[w,l] = round(result['packet_duplicate_rate'],2)
                
    #            Save WIFI stats data in the coresponding variables
                measurement_count[w,l] =    round(result['measurement_count'],2)
                link_quality[w,l] =         round(result['link_quality'],2)
                link_quatity_amount[w,l] =  round(result['link_quatity_amount'],2)
                signal_dbm[w,l] =           round(result['signal_dbm'],2)
                signal_mw[w,l] =            round(result['signal_mw'],2)
                noise_dbm[w,l] =            round(result['noise_dbm'],2)
                noise_mw[w,l] =             round(result['noise_mw'],2)
                bitrate[w,l] =              round(result['bitrate'],2)
                
                result['networks'] = iwlist.parse(content)
#                print(json.dumps(result, indent=4))
                
                pc_output_list.append(result)
                output_dict['pc2drone'] = pc_output_list
                
                
                output_dict['drone2pc'] = drone_output_list
                
                
        #  ==========[ Export NPY file  ]==========
        if savenpy_mode:
            npy_file_path = npy + 'Ping_{}_({}m).npy'.format(time_stamp,(h*h_step)+h_step)
            np.save(npy_file_path, point_data)
            print('\nNPY file for layer {}/{} is saved!'.format(h+1,h_points))

#  ==========[ Export dictionary to a CSV file ]==========
    if savecsv_mode:
        headers = []
        for item in output_dict['pc2drone'][0].keys():
            headers.append(item)
        with open(csv_file_path, mode='w') as csv_file:
            writer = csv.DictWriter(csv_file, headers)
            writer.writeheader()
            for item in output_dict['pc2drone']:
                writer.writerow(item)       
        print('\nCSV File saved!')

#  ==========[ Export dictionary to a JSON file ]==========
    if savejson_mode:
        with open(json_file_path, 'w') as f:
            json.dump(output_dict, f, indent=4)
        print('\nJSON File saved!')

##  ==========[ Show output graphs  ]==========        
#     if fig_mode:
#        dict2fig(output_dict,titles)
##        nparray2fig(fig_data,titles)
    
#  ==========[ Debugging prints ]==========
    if dbg_mode:
        print("\n ================  DEBUGGING IS ENABLED  ==================")
        print(json.dumps(output_dict, indent=4))
        for i,var in enumerate(point_data):
            for obj in var:
                print("\n",titles[i],":\n",(len(titles[i])+2)*"-","\n", var)
        
if __name__ == '__main__':
    main()