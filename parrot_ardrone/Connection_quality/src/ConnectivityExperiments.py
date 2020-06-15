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
import ARDroneLib.ARDroneLib as ARDroneLib
from time import sleep

import sys
np.set_printoptions(threshold=sys.maxsize)


#XXX:
class Drone():
    '''This class creates drone objects'''
    def __init__(self,drone_name,drone_ip,drone_type=None,pc_interface='wlp2s0'):
        self.name = drone_name
        self.ip = drone_ip
        self.pc_interface = pc_interface
        self.drone_type = drone_type
        
        if self.drone_type == 'Ardrone':
#            self.drone = ARDroneLib.Drone()
            pass
        elif self.drone_type == 'Anafi':
            pass
        else:
            print('The drone type is not specified!')
        
        print(self.name,"is created!")

# XXX:
class Environment():
    '''Class for holding all room's properties in one object'''
    def __init__(self,name='None',x=0,x_num=0,x_padding=0,y=0,y_num=0,y_padding=0,z=0,z_num=0,z_padding=0):
        self.name = name
        self.x = x
        self.x_num = x_num
        self.x_padding = x_padding
        self.y = y
        self.y_num = y_num
        self.y_padding = y_padding
        self.z = z
        self.z_num = z_num
        self.z_padding = z_padding
        
        self.x_points = np.linspace(0.0,x-x_padding*2,x_num)
        self.y_points = np.linspace(0.0,y-y_padding*2,y_num)
        self.z_points = np.linspace(0.0,z-z_padding*2,z_num)
        
        print("New Environment object is created")

class Experiments():
    '''This is the the common class that the other classes will be inherited'''
    pass

#XXX:
class RangeFinger():
    pass

#XXX:
class SpatialPinger():
    #TODO:
#    1) Fix the debug messages
#    2) Fix the welcome message - Show adsolute and relative distance in addition to points
#    3) Add on the Json file the relative, the absolute and the points
#    4) Add on the CSV file the relative, the absolute and the points

    '''Class for taking different type of ping measurements'''
    def __init__(self,drone, environment, dbg_mode=True,manual_mode=True,savecsv_mode=True,figure_mode=False,sound_mode=True,savejson_mode=True,savenpy_mode=True,welcome_mode=True,propeller_mode=False):
        self.drone = drone
#        self.note = note
##        define the rooms diamensions in meters (x,y,z OR w,l,h -> width,length, height)
#        self.x = x
#        self.y = y
#        self.z = z
#        self.xz_padding = xy_padding
#        
#        self.x_points = np.linspace(0.0,x-xy_padding*2,x_num)
#        self.y_points = np.linspace(0.0,y-xy_padding*2,y_num)
#        self.z_points = np.linspace(0.5,z,z_num)
        
        self.environment = environment
        self.point_data = np.zeros((16,len(self.environment.z_points),len(self.environment.x_points),len(self.environment.y_points)))
   
#        Set Modes
        self.dbg_mode = dbg_mode
        self.manual_mode = manual_mode
        self.savecsv_mode = savecsv_mode
        self.savejson_mode = savejson_mode
        self.figure_mode = figure_mode
        self.savenpy_mode = savenpy_mode
        self.welcome_mode = welcome_mode
        self.propeller_mode = propeller_mode
        self.sound_mode = sound_mode
        if self.propeller_mode:
            self.propellers='ON'
        else:
            self.propellers='OFF'
            
#       File paths   
        self.time_stamp = datetime.now().strftime("%d%m%y_%H%M%S")
        self.commonPath = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/SpatialPinger"
        self.npyPath = self.commonPath+"/NPY/{}_{}@{}heights_{}.npy".format(self.drone.name,self.time_stamp,len(self.environment.z_points),self.propellers)
        self.jsonPath = self.commonPath+"/JSON/{}_{}@{}heights_{}.json".format(self.drone.name,self.time_stamp,len(self.environment.z_points),self.propellers)
        self.csvPath = self.commonPath+"/CSV/{}_{}@{}heights_{}.csv".format(self.drone.name,self.time_stamp,len(self.environment.z_points),self.propellers)
        
#        Load sound file
        self.soundFile_path = "siren.wav"
        self.soundFile = AudioSegment.from_file(self.soundFile_path, format="wav")
        
#        Variable text descriptions
        self.output_keys = ["packet_transmit", "packet_receive", "packet_loss_count", "packet_loss_rate", \
                      "rtt_min", "rtt_avg", "rtt_max", "rtt_mdev", "packet_duplicate_count", "packet_duplicate_rate", \
                      "measurement_count", "link_quality", "link_quatity_amount", "signal_dbm", "noise_dbm", "bitrate"]
        
        print("New DronePinger object is created")

        
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
#    
#    def wifiScan(self,index):
##            WiFi network results
#            wifi_output_list = []
#            print("(PC): \t WIFI SCANNING...")
#            content = iwlist.scan(interface='wlp2s0') # Wifi module
#            for item in iwlist.parse(content):
#                item.update({'angle':self.points[index]})# Add angles
#                wifi_output_list.append(item)
#            return wifi_output_list
#
    def exportCSV(self,output_dict,x,y,z):
        '''Exprot CSV'''
        headers = []
        for item in output_dict[self.drone.name][0].keys():
            headers.append(item)
        with open(self.csvPath, mode='w') as csv_file:
            writer = csv.DictWriter(csv_file, headers)
            writer.writeheader()
            for item in output_dict[self.drone.name]:
                writer.writerow(item)
        print('(CSV): \tSave values till point ({},{},{}).'.format(x,y,z))
    
    def exportNPY(self,output_dict,x,y,z):
       '''Export NPY file '''
#       Store the returned values from every point to a list of numpy arrays
       for i,key in enumerate(self.output_keys):
           self.point_data[i,z,x,y] = round(output_dict[key],4)
       np.save(self.npyPath, self.point_data)
       print('(NPY): \tFile is updated till point ({},{},{})'.format(x,y,z))
    
    def exportJSON(self,output_dict,x,y,z):
        '''Export JSON file '''
        with open(self.jsonPath, 'w') as f:
            json.dump(output_dict, f, indent=4)
        print('(JSON):\tSave all values till point ({},{},{}).'.format(x,y,z))
        
    def welcome(self):
        #    Welcome message
        print("\n***************************************************\n\tDrone connection tester\n***************************************************\n")    
        print("Room diamensions:\n\t Width  (x) = {}m \n\t Length (y) = {}m \n\t Height (z) = {}m\n".format(self.environment.x,self.environment.y,self.environment.z))
        print("The drone should be moved:\n\t {} times ({}x{} points) in the 2D space\n\t {} times ({}x{}x{} points) in the 3D space\n".format(len(self.environment.x_points)*len(self.environment.y_points),len(self.environment.x_points),len(self.environment.y_points),len(self.environment.x_points)*len(self.environment.y_points)*len(self.environment.z_points),len(self.environment.x_points),len(self.environment.y_points),len(self.environment.z_points)))
        print("Please relax, this process is going to take \nquite some time.  Lets start! :D")
        print("***************************************************\n***************************************************\n")







    def start(self):
        self.welcome()
        pc_output_list = []
        output_dict = {}
        for z_index, z_point in enumerate(self.environment.z_points):
            for x_index, x_point in enumerate(self.environment.x_points):
                for y_index, y_point in enumerate(self.environment.y_points):
                    if self.sound_mode: # play sound
                        play(self.soundFile)
                    repeat = True
                    while repeat:
                        repeat = False
                        invalid_input = True
                        while invalid_input:
                            invalid_input = False
                            # The next 4 lines are creating the angle graphic using text chars
#                            print("\n\t\t+---+---+---+---+\n\t\t|   |   |   |   |\n\t\t+---+---+---+---+\n\t\t|   |   |   |   |\n\t\t+---+---+---+---+\n\t\t|   |   |   |   |\n\t\t+---+---+---+---+\n\t\t|   |   |   |   |\n\t\t+---+---+---+---+")
                            print("\n\t      +-------------------+\n\t      | +---+---+---+---+ |\n\t      | |   |   |   |   | |\n\t      | +---+---+---+---+ |\n\t      | |   |   |   |   | |\n\t      | +---+---+---+---+ |\n\t      | |   |   |   |   | |\n\t      | +---+---+---+---+ |\n\t      | |   |   |   |   | |\n\t      | +---+---+---+---+ |\n\t      +-------------------+")
                            print("\n     --> [ Next point:",(self.environment.x_points[x_index],self.environment.y_points[y_index],self.environment.z_points[z_index]),"] <-- ")
                            print("\n\t\t\tOR")
                            print("\n\t--> [ Next point:",(x_index,y_index,z_index),"] <-- \n")
    
                            if self.manual_mode:
                                if x_index == 0 and y_index == 0 and z_index == 0:
                                    user = input("Press [s] to start followed by [Enter] to start.")
                                else:
                                    user = input("Press [s] to start or [r] followed by [Enter] to repeat the measurement on this specific point.")
                                if user == "s":
                                    repeat = False
                                    invalid_input = False
                                elif user == "r":
                                    if x_index == 0 and y_index == 0 and z_index == 0:
                                        invalid_input = True
                                        print("You haven't started the measurement yet. Please press [s] to continue.")
                                    else:
                                        repeat = True
                                        invalid_input = False
                                else:
                                    print("Acceptable values are 's' to start and 'r' to repeat.")
                                    invalid_input = True
                                    
#                        if self.propeller_mode:
#                            self.drone.drone.takeoff()
#                            sleep(1)
#                            print('(DRONE): \t TAKING OFF \t [propellers ON]')
#                            sleep(2)
                            
        #                Combine Results
                        pc_result = {'x':x_index,'y':y_index, 'z':z_index, 'x_m':self.environment.x_points[x_index]+self.environment.x_padding,'y_m':self.environment.y_points[y_index]+self.environment.y_padding ,'z_m':self.environment.z_points[z_index]+self.environment.z_padding , 'propellers':self.propellers}    # add the angle
                        pc_result.update(self.ping(count=10))       # "127.0.0.1",'192.168.42.98'
                        pc_result.update(self.signalStrength())     # Retreive wifi statistics
                        pc_result.update(self.bitrate())            # Retrieve bitrate
                        pc_output_list.append(pc_result)
                        output_dict[self.drone.name] = pc_output_list
                        
        #                Save to file
                        if self.savenpy_mode:
                            self.exportNPY(pc_result,x_index,y_index,z_index) 
                        if self.savecsv_mode:
                            self.exportCSV(output_dict,x_index,y_index,z_index) 
                        if self.savejson_mode:
                            self.exportJSON(output_dict,x_index,y_index,z_index)
                        
        #                if self.propeller_mode:
        #                    self.drone.drone.land()
        #                    print('(DRONE): \t LANDING \t [propellers OFF]')
        #                    sleep(1)
        #                    self.drone.drone.land()


class RadiationTracker():
    '''This class is estublishes all the neccessary functionality for the radiation pattern measurements'''
#XXX:
# TODO:
#        Fix the duplicate JSON entry when the user repeats a measurement
#        Implement the propeller functionality for Anafi
    def __init__(self,drone, degrees=360, step=15, radius=1,axis='Yaw',dbg_mode=True,manual_mode=True,savecsv_mode=True,figure_mode=False,sound_mode=True,savejson_mode=True,savenpy_mode=True,welcome_mode=True,propeller_mode=False):
        self.drone = drone
        self.degrees = degrees
        self.step = step
        self.points = np.arange(0,degrees,step).tolist()
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
        
        self.point_data = np.zeros((len(self.output_keys),len(self.points)))
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
                item.update({'angle':self.points[angle_index]})# Add angles
                wifi_output_list.append(item)
            return wifi_output_list
    
    def exportCSV(self,output_dict,angle_index):
        '''Exprot CSV'''
        headers = []
        for item in output_dict[self.drone.name][0].keys():
            headers.append(item)
        with open(self.csvPath, mode='w') as csv_file:
            writer = csv.DictWriter(csv_file, headers)
            writer.writeheader()
            for item in output_dict[self.drone.name]:
                writer.writerow(item)
        print('(CSV): \tSave values until angle ({}{}).'.format(self.points[angle_index],self.degree_sign))
    
    def exportNPY(self,result,angle_index):
       '''Export NPY file '''
#       Store the returned values from every point to a list of numpy arrays
       for i,key in enumerate(self.output_keys):
           self.point_data[i,angle_index] = round(result[key],4)
       np.save(self.npyPath, self.point_data)
       print('(NPY): \tfile for angle {}{} is updated!'.format(self.points[angle_index],self.degree_sign))
    
    def exportJSON(self,output_dict,angle_index):
        '''Export JSON file '''
        with open(self.jsonPath, 'w') as f:
            json.dump(output_dict, f, indent=4)
        print('(JSON):\tSave all values until angle ({}{}).'.format(self.points[angle_index],self.degree_sign))
    
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
                
                if self.propeller_mode:
                    self.drone.drone.takeoff()
                    sleep(1)
                    print('(DRONE): \t TAKING OFF \t [propellers ON]')
                    sleep(2)
                    
#                Combine Results
                pc_result = {'radius':self.radius,'angle':self.points[index], 'axis':self.axis, 'propellers':self.propellers}    # add the angle
                pc_result.update(self.ping(count=10))        # "127.0.0.1",'192.168.42.98'
                pc_result.update(self.signalStrength())     # Retreive wifi statistics
                pc_result.update(self.bitrate())            # Retrieve bitrate
                pc_output_list.append(pc_result)
                output_dict[self.drone.name] = pc_output_list
                
                if self.savenpy_mode:
                    self.exportNPY(pc_result,index) 
                if self.savecsv_mode:
                    self.exportCSV(output_dict,index) 
                if self.savejson_mode:
                    self.exportJSON(output_dict,index)
                
                if self.propeller_mode:
                    self.drone.drone.land()
                    print('(DRONE): \t LANDING \t [propellers OFF]')
                    sleep(1)
                    self.drone.drone.land()
                    
#  ############################
#  ########### MAIN ###########
#  ############################
def main():  
    
    virtual_drone = Drone('Router','192.168.1.1',drone_type='Ardrone')
    virtual_env = Environment(x=12,x_num=5,x_padding=1,y=12,y_num=5,y_padding=1,z=4.5,z_num=4,z_padding=0.5)
#    rad1m = RadiationTracker(virtual_drone,degrees=360,axis='Roll',sound_mode=True,manual_mode=True,propeller_mode=True)
#    rad1m.start()
#    print(rad1m.point_data)
#    
    pinger_1 = SpatialPinger(virtual_drone,virtual_env,sound_mode=False,manual_mode=True)
    pinger_1.start()
#    print(pinger.point_data)    
    

    
#    path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern/NPY/Router_090620_031319_Roll@1m_OFF.npy'
#    test = np.load(path)
#    print(test)
if __name__ == '__main__':
    main()