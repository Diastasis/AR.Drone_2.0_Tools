
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

#================================
# Flag variables
#================================
dbg_mode = False                    # Enable debugging
fig_mode = False                     # Enable figure generation
manual_mode = True                 # Enable manual mode
savecsv_mode = True                 # Save data in a CSV file
sound_mode = True                  # Enable sound notification
savejson_mode = True                # Save data to a JSON file
savenpy_mode = True                 # Save data to a npy binary file
welcome_mode = True                 # Enable welcome information message 


# =============================================
# Function Definition
# =============================================
def pc2drone_pinger(target_ip="192.168.1.1", count=5): # "178.239.173.175" #'192.168.1.1' # '127.0.0.1'
    """Parse ping values from PC's terminal"""
    print("(PC): \t PINGING DRONE \t\t [{}]".format(target_ip))
    ping_parser = pingparsing.PingParsing()
    transmitter = pingparsing.PingTransmitter()
    transmitter.destination = target_ip
    transmitter.count = count
    try:
        return ping_parser.parse(transmitter.ping()).as_dict()
    except:
        print("(PC): Unable to ping the target IP!")
        return {"destination": target_ip, "packet_transmit": -999, "packet_receive": -999,
            "packet_loss_count": -999, "packet_loss_rate": -999, "rtt_min": -999, "rtt_avg": -999,
            "rtt_max": -999, "rtt_mdev": -999, "packet_duplicate_count": -999,"packet_duplicate_rate": -999}

def pc_wifi_stats(interface="wlp2s0", count=5):
    """Parse wifi values from PC's terminal"""
    print("(PC): \t GETTING WIFI STATS \t [{}]".format(interface))
    args = ["grep", "-i", interface,"/proc/net/wireless"]
    link_,level_, noise_ = [], [], []
    for _ in range(count):
        try:
            link, level, noise = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())[16:35].replace(".","").strip().split()
        except:
            print("(PC): Unable to retrieve wifi data. Are you connected?") 
            return {"measurement_count": count, "link_quality": -999,"link_quatity_amount": -999,
            "signal_dbm": -999, "signal_mw": -999, "noise_dbm": -999, "noise_mw": -999}
#       Add every value to a list
        link_.append(int(link))
        level_.append(int(level))
        noise_.append(int(noise))
    return {  #   Calculate the average values when neccessary and return a dictionary for every point. 
            "measurement_count": count,
            "link_quality": sum(link_)/len(link_), 
            "link_quatity_amount": int((sum(link_)/len(link_))*(10/7)),
            "signal_dbm": sum(level_)/len(level_),
            "signal_mw": round((10**((sum(level_)/len(level_))/10.))*1000000,2),
            "noise_dbm": sum(noise_)/len(noise_),
            "noise_mw": round((10**((sum(noise_)/len(noise_))/10.))*1000000,2)}

def pc_bitrate(interface="wlp2s0", count=5):
    """Parse bitrate by iwlist wlp2s0 bitrate command from PC"""
    print("(PC): \t GETTING BITRATE \t [{}]".format(interface))
    bitrate_pattern = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
    args = ["iwlist",interface,"bitrate"]
    bitrate_ = []
    for _ in range(count):
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

def drone2pc_pinger(telnet_host="192.168.1.1", target_ip="192.168.1.3", telnet_timeout=60, count = 5):
    """Parse ping values from DRONE's terminal"""
    print("(DRONE): PINGING PC \t\t [{}]".format(target_ip))
    ping_pattern = re.compile(br'=\s?(\d+.?\d+)/(\d+.?\d+)/(\d+.?\d+)\s?\bms') #Set a regex to parse the data
    false_output = { # Return error values in case of unexpected ping values
                "destination": target_ip, "packet_transmit": -999,"packet_receive": -999,
                "packet_loss_count": -999, "packet_loss_rate": -999, "rtt_min": -999,
                "rtt_avg": -999, "rtt_max": -999, "rtt_mdev": -999, "packet_duplicate_count": -999,
                "packet_duplicate_rate": -999}
    try:
        tn = telnetlib.Telnet(telnet_host,23,telnet_timeout)
    except OSError:
        print("(PC): \t Timeout. Drone's telnet is unreachable!")
        return false_output
    except:
        print("(PC): \t Unknown error!")
        return false_output    
    time.sleep(3)
    telnet_commant = "ping " + target_ip + " -c " + str(count) + "\n"  # <-- Test if it works
    tn.write(bytes(telnet_commant, encoding='utf-8'))                   # <-- Test if it works
    OUTPUT = tn.expect([ping_pattern], telnet_timeout)                              # Time out in 60 Secs
    tn.close()                                                          # close the connection
#    print("\nTELNET OUTPUT:", OUTPUT)
    if OUTPUT[0] == -1:
        print("(DRONE): Unexpected ping values!")
        return { # Return error values in case of unexpected ping values
                "destination": target_ip, "packet_transmit": -999,"packet_receive": -999,
                "packet_loss_count": -999, "packet_loss_rate": -999, "rtt_min": -999,
                "rtt_avg": -999, "rtt_max": -999, "rtt_mdev": -999, "packet_duplicate_count": -999,
                "packet_duplicate_rate": -999}
    parser = pingparsing.PingParsing()
    # -------------------
#    OUTPUT = OUTPUT[2].decode('ascii')
#    print("\nOUTPUT AFTER DECODING:", OUTPUT)
    #--------------------
    OUTPUT = parser.parse(dedent(OUTPUT[2].decode('ascii')))
#    OUTPUT = parser.parse(dedent(OUTPUT))
#    print("\nOUTPUT AFTER PARSER:", OUTPUT)
    return OUTPUT.as_dict() # return a dictionary

def drone_wifi_stats(telnet_host="192.168.1.1", telnet_timeout=60):
    """Parse wifi values from DRONE. Only the link quality value is available, however."""
    print("(DRONE): GETTING WIFI STATS \t [ath0]")
#    TODO: Fix the redudant patterns
    link_pattern = re.compile(br'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
    link_pattern2 = re.compile(r'ath0:\s\d{4}\s*(\d+)') # Regular expression to parse the terminal output
    false_output = {"link_quality":-999}
    try:
        tn = telnetlib.Telnet(telnet_host,23,telnet_timeout)
    except OSError:
        print("(PC): \t Timeout. Drone's telnet is unreachable!")
        return false_output
    except:
        print("(PC): \t Unknown error!")
        return false_output  
    time.sleep(3)
    tn.write(b"grep -i ath0 /proc/net/wireless\n")
    bit_rate = tn.expect([link_pattern], 10)# Time out in 15 Secs
    tn.close() # close the connection
    if bit_rate[0] == -1:
        print("(DRONE): Link value doesn't have the right format.")
        return false_output
    matches = link_pattern2.findall(bit_rate[2].decode('ascii'))
    bit_rate = 0
    for match in matches:
        bit_rate = match
    return {"link_quality": round(float(bit_rate),2)} #[2].decode('ascii'))    

def drone_bitrate(telnet_host="192.168.1.1", telnet_timeout=60, interface="ath0" ):
    """Parse bitrate by iwlist wlp2s0 bitrate command from the DRONE"""
    print("(DRONE): GETTING BITRATE \t [{}]".format(interface))
    bitrate_pattern_bin = re.compile(br'Current Bit Rate[:=](\d+\.?\d+)')   # Regular expression to parse the terminal output
    bitrate_pattern = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)')        # Regular expression to parse the terminal output
    false_output = {"bitrate" : -999}
    try:
        tn = telnetlib.Telnet(telnet_host,23,telnet_timeout)
    except OSError:
        print("(PC): \t Timeout. Drone's telnet is unreachable!")
        return false_output
    except:
        print("(PC): \t Unknown error!")
        return false_output
    time.sleep(3)
#    tn.write(b"iwlist ath0 bitrate\n")   
    telnet_commant = "iwlist " + interface + " bitrate\n"                   # <-- Test if it works
    tn.write(bytes(telnet_commant, encoding='utf-8'))                       # <-- Test if it works   
    bit_rate_result = tn.expect([bitrate_pattern_bin], 10)                  # Time out in 60 Secs
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
#     TODO: Fix the way the data are stored in NPY (Save file in every interation)
#     TODO: Fix the way the data are stored in CSV (Save file in every interation)
#     TODO: Fix the way the data are stored in JSON (Save file in every interation)

    ping_count_pc = ping_count_dr = 25  # number of ping packages
    telnet_timeout = 10                 # telnet timeout in secs
    dr_ip = dr_telnet = "192.168.1.1"   # IP of the drone (HOST)
    pc_ip = "192.168.1.3"               # IP of the pc when connected to drone (CLIENT)    
    
#     define the rooms diamensions in meters (x,y,z OR w,l,h -> width,length, height)
    width = 11 #11
    w_step = 1
    length = 11 #11
    l_step = 1
    height = 3
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
    
    if welcome_mode:
        print("\n***************************************************\n\tDrone connection tester\n***************************************************\n")    
        print("Room diamensions:\n\t Width  (x) = {}m \n\t Length (y) = {}m \n\t Height (z) = {}m\n".format(width,length,height))
        print("Please move manually the drone in different \npositions using the following steps:")
        print("\t Width  (x) = {}m \n\t Length (y) = {}m \n\t Height (z) = {}m\n".format(w_step,l_step,h_step))
        print("The drone should be moved:\n\t {} times ({}x{} points) in the 2D space\n\t {} times ({}x{}x{} points) in the 3D space\n".format(w_points*l_points,w_points,l_points, w_points*l_points*h_points,w_points,l_points,h_points))
        print("Please relax, this process is going to take \nquite some time.  Lets start! :D")
        print("***************************************************\n***************************************************\n")
    
    
#    Initialiaze all fields to np arrays filled with zeros
    packet_transmit, packet_receive, packet_loss_count, packet_loss_rate, \
    rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_duplicate_count, packet_duplicate_rate, \
    measurement_count, link_quality, link_quatity_amount, signal_dbm, signal_mw, noise_dbm,\
    noise_mw, bitrate = (np.zeros((h_points, w_points, l_points)) for i in range(18))
    
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
    
    output_keys = ["packet_transmit", "packet_receive", "packet_loss_count", "packet_loss_rate", \
                  "rtt_min", "rtt_avg", "rtt_max", "rtt_mdev", "packet_duplicate_count", "packet_duplicate_rate", \
                  "measurement_count", "link_quality", "link_quatity_amount", "signal_dbm", "signal_mw", "noise_dbm", "noise_mw", "bitrate"]
    
#    Load sound file
    sound = AudioSegment.from_file("siren.wav", format="wav")
    
#    File paths
    time_stamp = datetime.now().strftime("%d%m%y_%H%M%S")
    commonPath = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/"
    json_file_path = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/JSON/ConQlt_{}.json".format(time_stamp)
    
    pc_output_list = []
    drone_output_list = []
    wifi_output_list = []
    output_dict = {}
    
    for h in range(h_points):
        npyPath = commonPath + 'NPY/PC2DR_{}_({}m).npy'.format(time_stamp,(h*h_step)+h_step) # when the height changes creat a new npy file (change path)
        for w in range(w_points):
            for l in range(l_points):
 #=====================================================================================================================
 # It seems to work
                if sound_mode:
                    play(sound)
                    
                repeat = True
                while repeat:
                    repeat = False
                    invalid_input = True
                    while invalid_input:
                        invalid_input = False
                        print("\n\t --> [ Next point:",(w,l,h),"] <-- \n")
                        if manual_mode:
                            if h == 0 and w == 0 and l == 0:
                                user = input("Press [s] to start followed by [Enter] to start.")
                            else:
                                user = input("Press [s] to start or [r] followed by [Enter] to repeat the measurement on this specific point.")
                            if user == "s":
                                repeat = False
                                invalid_input = False
                            elif user == "r":
                                if h == 0 and w == 0 and l == 0:
                                    invalid_input = True
                                    print("You haven't started the measurement yet. Please press [s] to continue.")
                                else:
                                    repeat = True
                                    invalid_input = False
                            else:
                                print("Acceptable values are 's' to start and 'r' to repeat.")
                                invalid_input = True
                                
 #=====================================================================================================================                       
                    pc_result =         {'z':round((h*h_step)+h_step,2),'x':round((w*w_step)+w_step,2),'y':round((l*l_step)+l_step,2)} # Add coordinate points
                    drone_result =      {'z':round((h*h_step)+h_step,2),'x':round((w*w_step)+w_step,2),'y':round((l*l_step)+l_step,2)} # Add coordinate points
                  
    #                WiFi network results
                    print("(PC): \t WIFI SCANNING...")
                    content = iwlist.scan(interface='wlp2s0') # Wifi module
                    for item in iwlist.parse(content):
                        item.update({'z':round((h*h_step)+h_step,2),'x':round((w*w_step)+w_step,2),'y':round((l*l_step)+l_step,2)})# Add coordinate points
                        wifi_output_list.append(item)
                    
    #                PC results
                    pc_result.update(pc2drone_pinger("192.168.1.1",25))                   # ("127.0.0.1"
                    pc_result.update(pc_wifi_stats(interface="wlp2s0"))                                  # Retreive wifi statistics
                    pc_result.update(pc_bitrate(interface="wlp2s0"))                                     # Retrieve bitrate
                    
    #                Drone results
                    drone_result.update(drone2pc_pinger(count=25))
                    drone_result.update(drone_wifi_stats())
                    drone_result.update(drone_bitrate())
                    
#                    Store the returned values from every point to a list of numpy arrays
                    for var,key in zip(point_data, output_keys):
                        var[h,w,l] = round(pc_result[key],4)
                    
                    #  ==========[ Export NPY file  ]==========
                    if savenpy_mode:
    #                    npyPath = commonPath + 'NPY/PC2DR_{}.npy'.format(time_stamp)
                        np.save(npyPath, point_data)
                        print('\nNPY file for point ({},{},{}) is saved!'.format(w,l,h))
                                  
                    pc_output_list.append(pc_result)
                    output_dict['pc2drone'] = pc_output_list
                    drone_output_list.append(drone_result)
                    output_dict['drone2pc'] = drone_output_list
                    output_dict['networks'] = wifi_output_list
                    
                    #  ==========[ Export dictionary to a CSV file ]==========
                    if savecsv_mode:
                        csvPath_pc = commonPath + "csv/PC2DR_{}.csv".format(time_stamp)
                        csvPath_drone = commonPath + "csv/DR2PC_{}.csv".format(time_stamp)
                        csvPath_networks = commonPath + "csv/NET_{}.csv".format(time_stamp)
                #        Save 'pc2drone' values to CSV
                        headers = []
                        for item in output_dict['pc2drone'][0].keys():
                            headers.append(item)
                        with open(csvPath_pc, mode='w') as csv_file:
                            writer = csv.DictWriter(csv_file, headers)
                            writer.writeheader()
                            for item in output_dict['pc2drone']:
                                writer.writerow(item)
                        print('(CSV): Save pc2drone values until point ({},{},{}).'.format(w,l,h))
                        
                #        Save 'drone2pc' values to CSV        
                        headers = []
                        for item in output_dict['drone2pc'][0].keys():
                            headers.append(item)
                        with open(csvPath_drone, mode='w') as csv_file:
                            writer = csv.DictWriter(csv_file, headers)
                            writer.writeheader()
                            for item in output_dict['drone2pc']:
                                writer.writerow(item)
                        print('(CSV): Save drone2pc values until point ({},{},{}).'.format(w,l,h))
                        
                #        Save 'networks' to CSV
                        headers = []       
                        if len(output_dict['networks']) == 0:
                            print("No Wifi scanning data. Are you on flight mode?")
                        else:
                            for item in output_dict['networks'][0].keys():
                                headers.append(item)
                            with open(csvPath_networks, mode='w') as csv_file:
                                writer = csv.DictWriter(csv_file, headers)
                                writer.writeheader()
                                for item in output_dict['networks']:
                                    writer.writerow(item)
                        print('(CSV): Save network values until point ({},{},{}).'.format(w,l,h))
                        
                        #  ==========[ Export dictionary to a JSON file ]==========
                        if savejson_mode:
                            with open(json_file_path, 'w') as f:
                                json.dump(output_dict, f, indent=4)
                            print('\n(JSON:) Save all values until point ({},{},{}).'.format(w,l,h))
                
#        #  ==========[ Export NPY file  ]==========
#        if savenpy_mode:
#            npyPath = commonPath + 'NPY/PC2DR_{}_({}m).npy'.format(time_stamp,(h*h_step)+h_step)
#            np.save(npyPath, point_data)
#            print('\nNPY file for layer {}/{} is saved!'.format(h+1,h_points))


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