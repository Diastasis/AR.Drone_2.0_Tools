#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun  6 22:50:47 2020

@author: ros
"""

import ARDroneLib.ARDroneLib as ARDroneLib
from time import sleep
import pingparsing # https://pypi.org/project/pingparsing/
import iwlist # https://github.com/iancoleman/python-iwlist
import numpy as np
import subprocess
import re
from pydub import AudioSegment # play 
from pydub.playback import play
from datetime import datetime
import json
import csv
#import sys
#np.set_printoptions(threshold=sys.maxsize)


#XXX:
class Drone():
    '''This class creates drone objects'''
    def __init__(self,drone_name,drone_ip,drone_type=None,pc_interface='wlp2s0'):
        self.name = drone_name
        self.ip = drone_ip
        self.pc_interface = pc_interface
        self.drone_type = drone_type
        print(self.name,"is created!")

#XXX:
# TODO:
#        Fix the save to JSON,CSV and npy functionality
#        Implement the propeller functionality
        
class RadiationTracker():
    '''This class is estublishes all the neccessary functionality for the radiation pattern measurements'''
    def __init__(self,drone, degrees=360, step=15, radius=1,axis='Yaw',dbg_mode=True,manual_mode=True,savecsv_mode=True,figure_mode=False,sound_mode=True,savejson_mode=True,savenpy_mode=True,welcome_mode=True,propeller_mode=False):
        self.drone = drone
        self.degrees = degrees
        self.step = step
        self.points = np.arange(0,degrees,step)
        self.radius=radius
        self.axis = axis # Pitch/Roll/Yaw (X,Y,Z axis rotation)
        self.degree_sign = u'\N{DEGREE SIGN}' # This is the unicode representations of the degree symbol
        
        #        Set Modes
        self.dbg_mode = dbg_mode
        self.manual_mode = manual_mode
        self.savecsv_mode = savecsv_mode
        self.savejson_mode = savejson_mode
        self.figure_mode = figure_mode
        self.savenpy_mode = savenpy_mode
        self.welcome_mode = welcome_mode
        self.sound_mode = sound_mode
        self.propeller_mode = propeller_mode

        if self.propeller_mode:
            self.propellers='ON'
        else:
            self.propellers='OFF'
            
        #    File paths   
        self.time_stamp = datetime.now().strftime("%d%m%y_%H%M%S")
        self.commonPath = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern"
        self.jsonPath = self.commonPath+"/JSON/{}_{}_{}@{}m_{}.json".format(self.drone.name,self.time_stamp,self.axis,self.radius,self.propellers)
        self.npyPath = self.commonPath+"/NPY/{}_{}_{}@{}m_{}.npy".format(self.drone.name,self.time_stamp,self.axis,self.radius,self.propellers)
        self.csvPath = self.commonPath+"/CSV/{}_{}_{}@{}m_{}.csv".format(self.drone.name,self.time_stamp,self.axis,self.radius,self.propellers)
        
        #        Load sound file
        self.soundFile_path = "siren.wav"
        self.soundFile = AudioSegment.from_file(self.soundFile_path, format="wav")
        
        #        Variable text descriptions
        self.output_keys = ["packet_transmit", "packet_receive", "packet_loss_count", "packet_loss_rate", \
                      "rtt_min", "rtt_avg", "rtt_max", "rtt_mdev", "packet_duplicate_count", "packet_duplicate_rate", \
                      "measurement_count", "link_quality", "link_quatity_amount", "signal_dbm", "noise_dbm", "bitrate"]
        
        #        Initialiaze all fields to np arrays filled with zeros
        self.packet_transmit, self.packet_receive, self.packet_loss_count, self.packet_loss_rate, \
        self.rtt_min, self.rtt_avg, self.rtt_max, self.rtt_mdev, self.packet_duplicate_count, self.packet_duplicate_rate, \
        self.measurement_count, self.link_quality, self.link_quatity_amount, self.signal_dbm, self.noise_dbm,\
        self.bitrate_var = (np.zeros((len(self.points),1)) for i in range(16))

        #        Create a list of all np.array variables
        self.point_data = [self.packet_transmit, self.packet_receive, self.packet_loss_count, self.packet_loss_rate, \
                      self.rtt_min, self.rtt_avg, self.rtt_max, self.rtt_mdev, self.packet_duplicate_count, self.packet_duplicate_rate, \
                      self.measurement_count, self.link_quality, self.link_quatity_amount, self.signal_dbm, self.noise_dbm, self.bitrate]
        print("New RadiationTracker object is created!")


    def ping(self, count=25):
        """Parse ping output from PC's terminal"""
        print("(PC): \t PINGING DRONE \t\t [{}]".format(self.drone.ip))
        ping_parser = pingparsing.PingParsing()
        transmitter = pingparsing.PingTransmitter()
        transmitter.destination = self.drone.ip
        transmitter.count = count
        try:
            return ping_parser.parse(transmitter.ping()).as_dict()
        except:
            print("(PC): Unable to ping the target IP!")
            return {"destination": self.drone.ip, "packet_transmit": -999, "packet_receive": -999,
                "packet_loss_count": -999, "packet_loss_rate": -999, "rtt_min": -999, "rtt_avg": -999,
                "rtt_max": -999, "rtt_mdev": -999, "packet_duplicate_count": -999,"packet_duplicate_rate": -999}


    def signalStrength(self, readings=1):
        """Parse wifi output from PC's terminal"""
        print("(PC): \t GETTING WIFI STATS \t [{}]".format(self.drone.pc_interface))
        args = ["grep", "-i", self.drone.pc_interface,"/proc/net/wireless"]
        link_,level_, noise_ = [], [], []
        for _ in range(readings):
            try:
                link, level, noise = str(subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate())[16:35].replace(".","").strip().split()
            except:
                print("(PC): Unable to retrieve wifi data. Are you connected?") 
                return {"measurement_count": readings, "link_quality": -999,"link_quatity_amount": -999,
                "signal_dbm": -999, "signal_mw": -999, "noise_dbm": -999, "noise_mw": -999}
    #       Add every value to a list
            link_.append(int(link))
            level_.append(int(level))
            noise_.append(int(noise))
        return {  #   Calculate the average values when neccessary and return a dictionary for every point. 
                "measurement_count": readings,
                "link_quality": sum(link_)/len(link_), 
                "link_quatity_amount": int((sum(link_)/len(link_))*(10/7)),
                "signal_dbm": sum(level_)/len(level_),
                "noise_dbm": sum(noise_)/len(noise_)}
                
    def bitrate(self, readings=1):
        '''Parse bitrate by iwlist wlp2s0 bitrate command from PC'''
        print("(PC): \t GETTING BITRATE \t [{}]".format(self.drone.pc_interface))
        bitrate_pattern = re.compile(r'Current Bit Rate[:=](\d+\.?\d+)') # Regular expression to parse the terminal output
        args = ["iwlist",self.drone.pc_interface,"bitrate"]
        bitrate_ = []
        for _ in range(readings):
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
    
    def wifiScan(self,angle_index):
#            WiFi network results
            wifi_output_list = []
            print("(PC): \t WIFI SCANNING...")
            content = iwlist.scan(interface='wlp2s0') # Wifi module
            for item in iwlist.parse(content):
                item.update({'angle':self.points[angle_index]})# Add coordinate points
                wifi_output_list.append(item)
            return wifi_output_list
    
    def exportCSV(self,output_dict,angle_index):
        '''Exprot CSV'''
        headers = []
        for item in output_dict['values'][0].keys():
            headers.append(item)
        with open(self.csvPath, mode='w') as csv_file:
            writer = csv.DictWriter(csv_file, headers)
            writer.writeheader()
            for item in output_dict['values']:
                writer.writerow(item)
        print('(CSV): Save values until angle ({}{}).'.format(self.points[angle_index],self.degree_sign))
    
    def exportNPY(self,angle_index):
       '''Export NPY file '''
       np.save(self.npyPath, self.point_data)
       print('\nNPY file for angle {}{} is updated!'.format(self.points[angle_index],self.degree_sign))
    
    def exportJSON(self,output_dict,angle_index):
        '''Export JSON file '''
        with open(self.jsonPath, 'w') as f:
            json.dump(output_dict, f, indent=4)
        print('\n(JSON:) Save all values until angle ({}{}).'.format(self.points[angle_index],self.degree_sign))
    
    def welcome(self):
        #    Welcome message
        print("\n***************************************************\n\tDrone Radiation Pattern\n***************************************************\n")           
        print("Please rotate the drone manually in different \npositions using the step of: {}".format(self.step),self.degree_sign)
        print("The drone should be rotated around X,Y and Z axis \nstarting from 0{} to {}{} excluding the last value \n({} points pes axis and {} total points).".format(self.degree_sign,self.degrees,self.degree_sign,int(self.degrees/self.step),int((self.degrees/self.step)*2)))
        print("Please relax, this process is going to take \nquite some time.  Lets start! :D\n")
        print("***************************************************\n***************************************************\n")

    def start(self):
        self.welcome()
        pc_output_list = []
        output_dict = {}
        for index, point in enumerate(self.points):
            if self.sound_mode: # play sound
                play(self.soundFile)
            repeat = True
            while repeat:
                repeat = False
                invalid_input = True
                while invalid_input:
                    invalid_input = False
                    # The next 3 lines are creating the angle graphic using text chars
                    print("\t\t\t|\n\t\t    _ - | - _\n\t\t -      |      -\n\t       '        |        '\n\t     '          |          '\n\t   /            |            \ \n\t  ,             |             ,\n\t<---------------+--------------->")
                    print("\n\t  --> [ Next point:",str(self.points[index])+self.degree_sign,"] <-- \n")                    
                    if self.manual_mode:
                        if index == 0:
                            user = input("Press [s] to start followed by [Enter] to start.")
                        else:
                            user = input("Press [s] to start or [r] followed by [Enter] to repeat the measurement on this specific point.")
                        if user == "s":
                            repeat = False
                            invalid_input = False
                        elif user == "r":
                            if index == 0:
                                invalid_input = True
                                print("You haven't started the measurement yet. Please press [s] to continue.")
                            else:
                                repeat = True
                                invalid_input = False
                        else:
                            print("Acceptable values are 's' to start and 'r' to repeat.")
                            invalid_input = True
                
#                Combine Results
                pc_result = {'radius':self.radius,'angle':self.points[index], 'axis':self.axis, 'propellers':self.propellers}    # add the angle
                pc_result.update(self.ping(count=3))        # "127.0.0.1",'192.168.42.98'
                pc_result.update(self.signalStrength())     # Retreive wifi statistics
                pc_result.update(self.bitrate())            # Retrieve bitrate
                pc_output_list.append(pc_result)
                
                output_dict['values'] = pc_output_list
                
                if self.savecsv_mode:
                    self.exportCSV(output_dict,index)
                if self.savejson_mode:
                    print(self.bitrate_var.shape)
                    self.exportJSON(output_dict,index)
                if self.savenpy_mode:
                    pass
#                print(output_dict)
#                self.exportJSON(output_dict,index)
#                
##                Store the returned values from every point to a list of numpy arrays
#                for var,key in zip(self.point_data, self.output_keys):
#                    var[index] = round(pc_result[key],4)
#                    
#                self.exportNPY(index)
    

#  ############################
#  ########### MAIN ###########
#  ############################
def main():  
    
    drone = Drone('Router','192.168.1.1')
    rad1m = RadiationTracker(drone,degrees=60,axis='Roll',sound_mode=False)
    rad1m.start()

        
if __name__ == '__main__':
    main()