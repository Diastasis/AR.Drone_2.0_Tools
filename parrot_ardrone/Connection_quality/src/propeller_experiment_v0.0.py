#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
"""
#import struct
import ARDroneLib.ARDroneLib as ARDroneLib
#import ARDroneLib.ARDroneGUI as ARDroneGUI
from time import sleep

drone = ARDroneLib.Drone()

#gui = ARDroneGUI.ControlWindow(default_action=drone.hover)
#gui.add_action("", drone.forward)
#gui.change_text("Hello World !")



#drone.set_config()
drone.takeoff()
sleep(7)
drone.stop()
sleep(10)
drone.stop()
sleep(10)
drone.stop()
sleep(10)
drone.stop()
sleep(10)
#drone.land()
#sleep(5)
#drone.land()
#sleep(5)
#drone.land()
#sleep(5)
#drone.land()
#sleep(5)
#drone.land()