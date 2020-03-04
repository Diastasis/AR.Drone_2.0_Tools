
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
# Flag variables
#================================
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
def pc2drone_pinger(target_ip="192.168.1.1", _count=5): # "178.239.173.175" #'192.168.1.1' 
    """Parse ping values from PC's terminal"""
    print("(PC): \t PINGING DRONE \t\t [{}]".format(target_ip))
    ping_parser = pingparsing.PingParsing()
    transmitter = pingparsing.PingTransmitter()
    transmitter.destination = target_ip
    transmitter.count = _count
    try:
        return ping_parser.parse(transmitter.ping()).as_dict()
    except:
        print("(PC): Unable to ping the target IP!")
        return {"destination": target_ip, "packet_transmit": -999, "packet_receive": -999,
            "packet_loss_count": -999, "packet_loss_rate": -999, "rtt_min": -999, "rtt_avg": -999,
            "rtt_max": -999, "rtt_mdev": -999, "packet_duplicate_count": -999,"packet_duplicate_rate": -999}

def pc_wifi_stats(interface="wlp2s0", _count=5):
    """Parse wifi values from PC's terminal"""
    print("(PC): \t GETTING WIFI STATS \t [{}]".format(interface))
    args = ["grep", "-i", interface,"/proc/net/wireless"]
    link_,level_, noise_ = [], [], []
    for _ in range(_count):
        try:
            link, level, noise = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())[16:35].replace(".","").strip().split()
        except:
            print("(PC): Unable to retrieve wifi data. Are you connected?") 
            return {"measurement_count": _count, "link_quality": -999,"link_quatity_amount": -999,
            "signal_dbm": -999, "signal_mw": -999, "noise_dbm": -999, "noise_mw": -999}
#       Add every value to a list
        link_.append(int(link))
        level_.append(int(level))
        noise_.append(int(noise))
    return {  #   Calculate the average values when neccessary and return a dictionary for every point. 
            "measurement_count": _count,
            "link_quality": sum(link_)/len(link_), 
            "link_quatity_amount": int((sum(link_)/len(link_))*(10/7)),
            "signal_dbm": sum(level_)/len(level_),
            "signal_mw": round((10**((sum(level_)/len(level_))/10.))*1000000,2),
            "noise_dbm": sum(noise_)/len(noise_),
            "noise_mw": round((10**((sum(noise_)/len(noise_))/10.))*1000000,2)}

def pc_bitrate(interface="wlp2s0", _count=5):
    """Parse bitrate by iwlist wlp2s0 bitrate command from PC"""
    print("(PC): \t GETTING BITRATE \t [{}]".format(interface))
    bitrate_pattern = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
    args = ["iwlist",interface,"bitrate"]
    bitrate_ = []
    for _ in range(_count):
        try:
            my_string = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())
            matches = bitrate_pattern.finditer(my_string)
        except:
            print("(PC) Unable to retrieve bitrate.")
            return {'bitrate' : -999}
        bit_rate = -999
        for match in matches:
            bit_rate = match.group(1)
        bitrate_.append(float(bit_rate))
    return {'bitrate' : round(sum(bitrate_)/len(bitrate_),2)}

def drone2pc_pinger(telnet_host="192.168.1.1", target_ip="192.168.1.3", _count = 5):
    """Parse ping values from DRONE's terminal"""
#    TODO: Test if this funcion works as expected
    print("(DRONE): PINGING PC \t\t [{}]".format(target_ip))
    ping_pattern = re.compile(br'=\s?(\d+.?\d+)/(\d+.?\d+)/(\d+.?\d+)\s?\bms\b') #Set a regex to parse the data
    false_output = { # Return error values in case of unexpected ping values
                "destination": "192.168.1.3", "packet_transmit": -999,"packet_receive": -999,
                "packet_loss_count": -999, "packet_loss_rate": -999, "rtt_min": -999,
                "rtt_avg": -999, "rtt_max": -999, "rtt_mdev": -999, "packet_duplicate_count": -999,
                "packet_duplicate_rate": -999}
    try:
        tn = telnetlib.Telnet(telnet_host)
    except OSError:
        print("(PC): \t Drone's telnet is unreachable!")
    except:
        print("(PC): \t Unknown error!")
    finally:     
        return false_output    
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

def drone_wifi_stats(telnet_host="192.168.1.1"):
    """Parse wifi values from DRONE. Only the link quality value is available, however."""
#    TODO: FIX THE CODE HERE
    print("(DRONE): GETTING WIFI STATS \t [ath0]")
#    TODO: Fix the redudant patterns
    link_pattern = re.compile(br'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
    link_pattern2 = re.compile(r'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
    false_output = {"link_quality":-999}
    try:
        tn = telnetlib.Telnet(telnet_host)
    except OSError:
        print("(PC): \t Drone's telnet is unreachable!")
    except:
        print("(PC): \t Unknown error!")
    finally:     
        return false_output  
    time.sleep(3)
    tn.write(b"grep -i ath0 /proc/net/wireless\n")
    bit_rate = tn.expect([link_pattern], 15)# Time out in 15 Secs
    tn.close() # close the connection
    if bit_rate[0] == -1:
        print("(DRONE): Bitrate value doesn't have the right format.")
        return false_output
    matches = link_pattern2.findall(bit_rate[2].decode('ascii'))
    bit_rate = 0
    for match in matches:
        bit_rate = match
    return {"link_quality": round(float(bit_rate),2)} #[2].decode('ascii'))    

def drone_bitrate(telnet_host="192.168.1.1", interface="ath0" ):
    """Parse bitrate by iwlist wlp2s0 bitrate command from the DRONE"""
#    TODO: Test if it works 
    print("(DRONE): GETTING BITRATE \t [{}]".format(interface))
    bitrate_pattern_bin = re.compile(br'Current Bit Rate[:=](\d+\.?\d+)')   # Regular expression to parse the terminal output
    bitrate_pattern = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)')        # Regular expression to parse the terminal output
    false_output = {"bitrate" : -999}
    try:
        tn = telnetlib.Telnet(telnet_host)
    except OSError:
        print("(PC): \t Drone's telnet is unreachable!")
    except:
        print("(PC): \t Unknown error!")
    finally:     
        return false_output 
    time.sleep(3)
#    tn.write(b"iwlist ath0 bitrate\n")   
    telnet_commant = "iwlist " + interface + " bitrate\n"                   # <-- Test if it works
    tn.write(bytes(telnet_commant, encoding='utf-8'))                       # <-- Test if it works   
    bit_rate_result = tn.expect([bitrate_pattern_bin], 60)                         # Time out in 60 Secs
    tn.close()                                                              # close the connection
    bit_rate = -999
    if bit_rate_result[0] == -1:
        print("(DRONE): Unexpected bitrate values!")
        return false_output
    matches = bitrate_pattern.findall(bit_rate_result[2].decode('ascii'))
    for match in matches:
        bit_rate = match#.group(1)
    return {"bitrate" : round(float(bit_rate),2)}                           # returns bitrate dictionary 
    
#  ==========[ Main ]==========
def main():  
#     TODO: Add user promption in case of zero value in w_point/h_point/l_point 
#     TODO: Save PC,drone and networks in 3 different files
#     TODO: Fix the way the data are stored in the numpy arrays

#     define the rooms diamensions (x,y,z -> width,length, height)
    width = 3
    w_step = 1
    length = 3
    l_step = 1
    height = 1
    h_step = 1
    
#    Pre-calculate how many points required for width
    if width < w_step:
        w_points = 0
    elif (width - 0.5) < w_step: # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        w_points = 1
    else:
        w_points = int((width - 0.5)/w_step)+1 # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
    
#    Pre-calculate how many points required for length
    if length < l_step:
        l_points = 0
    elif (length - 0.5) < l_step: # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        l_points = 1
    else:
        l_points = int((length - 0.5)/l_step)+1 # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset

#    Pre-calculate how many points required for height
    if height < h_step:
        h_points = 0
    elif (height - 0.5) < h_step: # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        h_points = 1
    else:
        h_points = int((height - 0.5)/h_step)+1 # The radius of the drone stand is 0.25 for one sides is 0.25*2=0.5m offset
        
        
#    Initialiaze all fields to np arrays filled with zeros
    packet_transmit, packet_receive, packet_loss_count, packet_loss_rate, \
    rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_duplicate_count, packet_duplicate_rate, \
    measurement_count, link_quality, link_quatity_amount, signal_dbm, signal_mw, noise_dbm,\
    noise_mw, bitrate = (np.zeros((w_points,l_points)) for i in range(18))
    
#    Create a list of all np.array variables
    point_data = [packet_transmit, packet_receive, packet_loss_count, packet_loss_rate, \
                  rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_duplicate_count, packet_duplicate_rate, \
                  measurement_count, link_quality, link_quatity_amount, signal_dbm, signal_mw, noise_dbm, noise_mw, bitrate]
    
#    Define the titles of the figures
    titles = ['Packets Transmitted (#)', 'Packets Received (#)', 'Packet Loss (#)','Packet Loss (%)',
            'Minimum Time (ms)','Avarage Time (ms)','Maximun Time (ms)', 'Mean Deviation (ms)',
            'Duplicate Packets (#)', 'Duplicate Packets (%)', 'Wifi Measurements (#/Point)',
            'Link Quality (#/70)', 'Link Quality (%)','Signal Level (dBm)','Signal Level (mW)',
            'Noise Level (dBm)', 'Noise Level (mW)','Bit rate (Mb/s)']
    
#    Load sound file
    sound = AudioSegment.from_file("siren.wav", format="wav")
    
#    File paths
    time_stamp = datetime.now().strftime("%d%m%y_%H%M%S")
    csv_file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/csv/ConQlt_{}.csv".format(time_stamp)
    json_file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/JSON/ConQlt_{}.json".format(time_stamp)
    #npy_file_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/ConQlt_{}.npy'.format(time_stamp)
    npy = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/'
    
    pc_output_list = []
    drone_output_list = []
    wifi_output_list = []
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
                    
                pc_result =         {'z':round((h*h_step)+h_step,2),'x':round((w*w_step)+w_step,2),'y':round((l*l_step)+l_step,2)} # Add coordinate points
                drone_result =      {'z':round((h*h_step)+h_step,2),'x':round((w*w_step)+w_step,2),'y':round((l*l_step)+l_step,2)} # Add coordinate points
                pc_wifi_result =    {'z':round((h*h_step)+h_step,2),'x':round((w*w_step)+w_step,2),'y':round((l*l_step)+l_step,2)} # Add coordinate points
              
#                WiFi network results
                print("(PC): \t WIFI SCANNING...")
                content = iwlist.scan(interface='wlp2s0') # Wifi module
                pc_wifi_result['networks'] = iwlist.parse(content)
                
#                PC Results
                pc_result.update(pc2drone_pinger())
                pc_result.update(pc_wifi_stats())                                  # Retreive wifi statistics
                pc_result.update(pc_bitrate())                                     # Retrieve bitrate
                
#                Drone results
                drone_result.update(drone2pc_pinger())
                drone_result.update(drone_wifi_stats())
                drone_result.update(drone_bitrate())
                
#                Save ping data in the coresponding variables
                packet_transmit[w,l] =      round(pc_result['packet_transmit'],2)
                packet_receive[w,l] =       round(pc_result['packet_receive'],2)
                packet_loss_count[w,l] =    round(pc_result['packet_loss_count'],2)
                packet_loss_rate[w,l] =     round(pc_result['packet_loss_rate'],2)
                rtt_min[w,l] =              round(pc_result['rtt_min'],2)
                rtt_avg[w,l] =              round(pc_result['rtt_avg'],2)
                rtt_max[w,l] =              round(pc_result['rtt_max'],2)
                rtt_mdev[w,l] =             round(pc_result['rtt_mdev'],2)
                packet_duplicate_count[w,l] = round(pc_result['packet_duplicate_count'],2)
                packet_duplicate_rate[w,l] = round(pc_result['packet_duplicate_rate'],2)
                
#                Save WIFI stats data in the coresponding variables
                measurement_count[w,l] =    round(pc_result['measurement_count'],2)
                link_quality[w,l] =         round(pc_result['link_quality'],2)
                link_quatity_amount[w,l] =  round(pc_result['link_quatity_amount'],2)
                signal_dbm[w,l] =           round(pc_result['signal_dbm'],2)
                signal_mw[w,l] =            round(pc_result['signal_mw'],2)
                noise_dbm[w,l] =            round(pc_result['noise_dbm'],2)
                noise_mw[w,l] =             round(pc_result['noise_mw'],2)
                bitrate[w,l] =              round(pc_result['bitrate'],2)
                
#                pc_result['networks'] = iwlist.parse(content)
#                print(json.dumps(result, indent=4))
                
                pc_output_list.append(pc_result)
                drone_output_list.append(drone_result)
                wifi_output_list.append(pc_wifi_result)
                output_dict['pc2drone'] = pc_output_list
                output_dict['drone2pc'] = drone_output_list
                output_dict['networks'] = wifi_output_list
                
                
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