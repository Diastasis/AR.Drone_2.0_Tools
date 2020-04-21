import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.image as mpimg


#================================================
# FLOW:
# Set the path of the files
# Extract the height from the file name
# Extract the drone type from the file name
# Load both NPY files
# Extract the metadata
# Set the titles of the variables 
#================================================
# TODO:
# 1) Some variables contain long values. Make the plots slightly bigger
# 2) Fix the scale of the color bars on all variables
# -- AVGTime ar = 0 - 125 ms | an = 0 - 0.05 ms | dif = 0 - 125 ms
# 3) Create and import the greenhouse layout
# 4) Create a statistics table for each case
# 5) Delete from NPY noiseMW and RSSMW
# 6) Delete from CSV noiseMW and RSSMW




class dataVisualiser:
    
#   Titles used in plots
    varTitles = ('Packets Transmitted (#)', 'Packets Received (#)', 'Packet Loss (#)','Packet Loss (%)',
                'Minimum Time (ms)','Avarage Time (ms)','Maximun Time (ms)', 'Mean Deviation (ms)',
                'Duplicate Packets (#)', 'Duplicate Packets (%)', 'Wifi Readings (#/Point)',
                'Recieved Signal Strenght Index (#/70)', 'Recieved Signal Strenght (%)','Recieved Signal Strenght (dBm)',
                'Noise Level (dBm)','Bitrate (Mb/s)')
    
#   Names of the output files
    outputNames = ('packet_tx','packet_rx','packet_loss_num','packet_loss_perc','min_time','avg_time','max_time',
                   'mean_time','double_packets_num','double_packets_perc','wifi_readings','RSSI','RSS_perc','RSS_dBm',
                   'noise_dbm','bitrate')
    

#    pointSpace = 
    
#   Choose which variables should be plotted 
#            [pkgTx|pkgRx|PkgLossN|PkgLossP|MinTime|AvgTime|MaxTime|MdevTime|PkgDoubleN|PkgDoubleP|Readings|RSSi|RSSp|RSSdbm|noisedbm|bitrate]
    useVar = [False,False,   False,    True,   True,   True,   True,    True,   False,     True,    False,  True,True,  True,  False,    True]
#    barLim = [(0,50),(0,50),(0,50),(0,100),(0,1000),(0,1000),(0,1000),(0,1000),(0,50),    (0,100), (0,10),(0,70),(0,100),(-100,0),(0,-255)(0,300)]
    indices = [i for i, val in enumerate(useVar) if val]
    
    ##   Sel | Variable
## ==================================================
##  0    | Packets Transmitted (#)
##  1    | Packets Received (#)
##  2    | Packet Loss (#)
##  3    | Packet Loss (%)
##  4    | Minimum Time (ms)
##  5    | Avarage Time (ms)
##  6    | Maximun Time (ms)
##  7    | Mean Deviation (ms)
##  8    | Duplicate Packets (#)
##  9    | Duplicate Packets (%)
##  10   | Wifi Readings (#/Point)
##  11   | Recieved Signal Strenght Index (#/70)
##  12   | Recieved Signal Strenght (%)
##  13   | Recieved Signal Strenght (dBm)
##  14   | Noise Level (dBm)
##  15   | Bitrate (Mb/s)
## ------------------------------------------------

#   Define the measured heights 
    layerHeights = (0.5, 1.5, 2.5, 3.5)
    measurementPoint = (10, 10.5, 1) #x,y,z in meters (laptop position)
    
    outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    debug = False
    
    def __init__(self, data, name, comp=False):
        self.dataList = data
        self.droneName = name
        self.objComp = comp
        self.barLim = (0,100)
        if comp:
            self.colormap = "jet"
        else:
            self.colormap="inferno"
        self.groupPlot = False
        if self.debug:
            print("Object "+name+" was created!")
        
#    @classmethod
#    def debugMode(cls, path, name):
#        cls.debug = True
#        print("DEBUGGING MESSAGES ARE ACTIVATED!")
#        return cls(path, name)
    
    @classmethod
    def form_Path(cls, path, name):
        '''Create an object by importing data from an external NPY file'''
        data = np.load(path)
        return cls(data,name)
    
    @classmethod
    def from_Substruction(cls, obj1, obj2, name):
        '''Create a new object by substructing two other objects.'''
        data = obj1.dataList - obj2.dataList
        return cls(data,name,comp=True)

#    @classmethod
#    def from_Addition(cls,obj1,obj2, name):
#        '''Create a new object by adding two other objects.'''
#        data = obj1.dataList + obj2.dataList
#        return cls(data,name,comp=True)
    
    def plotTx(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 0
#        self.barLim = (0,30)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTx was executed!")
        
    def plotRx(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 1
#        self.barLim = (20,26)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotΡx was executed!")
        
    def plotLoss_num(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 2
#        self.barLim = (0,5)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotLoss_num was executed!")

    def plotLoss_perc(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 3
#        self.barLim = (0,15)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotLoss_perc was executed!")
    
    def plotTime_min(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 4
#        self.barLim = (0,1000)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_min was executed!")
    
    def plotTime_avg(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 5
#        self.barLim = (10,125)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_avg was executed!")
        
    def plotTime_max(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 6
#        self.barLim = (0,1000)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_max was executed!")
    
    def plotTime_mdev(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 7
#        self.barLim = (0,1000)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, 80)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_mdev was executed!")
        
    def plotDouble_num(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 8
#        self.barLim = (0,30)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotDouble_num was executed!")
        
    def plotDouble_perc(self, groupPlot=False):
        self.groupPlot = groupPlot
        indx = 9
#        self.barLim = (0,100)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotDouble_perc was executed!")
    
    def plotReadings(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 10
#        self.barLim = (0,10)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotReadings was executed!")
        
    def plotRSSI(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 11
#        self.barLim = (0,80)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotRSSI was executed!")
        
    def plotRSS_perc(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 12
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotRSS_perc was executed!")
    
    def plotRSS_dbm(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 13
#        self.barLim = (-100,0)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotRSS_dbm was executed!")
        
    def plotNoise_dbm(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 14
#        self.barLim = (-250,0)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotNoise_dbm was executed!")
        
    def plotBitrate(self,groupPlot=False):
        self.groupPlot = groupPlot
        indx = 15
#        self.barLim = (0,300)
#        self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupPlot(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
                self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotBitrate was executed!")
        
    def plotAll(self,groupPlot=False):
        self.plotTx(groupPlot)
        self.plotRx(groupPlot)
        self.plotLoss_num(groupPlot)
        self.plotLoss_perc(groupPlot)
        self.plotTime_min(groupPlot)
        self.plotTime_avg(groupPlot)        
        self.plotTime_max(groupPlot)
        self.plotTime_mdev(groupPlot)      
        self.plotDouble_num(groupPlot)
        self.plotDouble_perc(groupPlot)
        self.plotReadings(groupPlot)
        self.plotRSSI(groupPlot)
        self.plotRSS_perc(groupPlot)
        self.plotRSS_dbm(groupPlot)
        self.plotNoise_dbm(groupPlot)
        self.plotBitrate(groupPlot)
        if self.debug:
            print("plotAll was executed!")
    
    def varGroupPlot(self,inputVar,heightList, title, outputName):
        fig= plt.figure(figsize=(8*len(inputVar),10))
        for ii,height in enumerate(heightList):
            ax = fig.add_subplot(1,len(inputVar),ii+1)
            ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
#        plt.suptitle(self.droneName + " ({}m)\n{}".format(height, title))
            plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")
            ax.set_xlim(-1,11)
            ax.set_ylim(11,-1)
        
#        ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
#            img = plt.imread('/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/Greenhouse_Layout.png')
            imgplot = ax.imshow(inputVar[ii], cmap=self.colormap,interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
#            imgplot.set_zorder(1)
#            plt.imshow(img, interpolation='none', alpha=1, zorder=2)

            imgplot.set_clim(self.barLim)        
            mybar = fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.95,pad=0.08)#.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
            if self.objComp:
                mybar.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(inputVar[ii][i][j],2), ha="center", va="center", color="w")
            ax.scatter(self.measurementPoint[0],self.measurementPoint[1], alpha=0.5, s=100, color="b", marker="X")
            plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        plt.show()
        self.savePng(fig,self.outputPath,outputName,"all_")
        if self.debug:
            print("varGroupPlot was exectuted!")
    
    
    
    def varPlot(self,inputVar, height, title, outputName):
        fig= plt.figure(figsize=(10,10))
        ax = fig.add_subplot(1,1,1)
        ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
#        plt.suptitle(self.droneName + " ({}m)\n{}".format(height, title))
        plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
        
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_xlim(-1,11)
        ax.set_ylim(11,-1)
        
#        ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
        img = mpimg.imread('/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/Greenhouse_Layout.png')
        imgplot = ax.imshow(inputVar, cmap=self.colormap,interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
        
        imgplot.set_clim(self.barLim)
#        imgplot.set_label("TEST")
        mybar = fig.colorbar(imgplot, ax=ax, orientation='horizontal',shrink=0.75,pad=0.08)
#        mybar = fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.95,pad=0.08)#.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
        if self.objComp:
                mybar.set_label("AR.Drone 2.0                                                                                                                                          Anafi", labelpad=-55)
        for i in range(11):
            for j in range(11):
                ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                ax.text(j, i, round(inputVar[i][j],2), ha="center", va="center", color="w")
        ax.scatter(self.measurementPoint[0],self.measurementPoint[1], alpha=0.5, s=100, color="b", marker="X")
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        self.savePng(fig,self.outputPath,outputName,height)
        plt.show()
        if self.debug:
            print("varPlot was exectuted!")
    
    def savePng(self,figure,outputPath,outputName,height):
        '''Save a figure to a PNG file on the provided path'''
        figure.savefig(outputPath+"/{}_{}_({}m).png".format(self.droneName,outputName,height), dpi=100)  # results in 160x120 px image
        print("Figure {}_{}_({}m) exported as PNG file.".format(self.droneName,outputName,height))
        if self.debug:
            print("savePng was exectuted!")

#           
#    def stats(self):
##        Stats for every layer:
#        minVal = 
#        maxVal =
#        avgVal =
#        modVal = 
##       Confidence:
#        case5  =
#        case25 = 
#        case50 =
#        case75 = 
#        case95 = 





def  main():
    ardrone_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(full)_(ARDRONE_FIXED_FINAL).npy'
    test = dataVisualiser.form_Path(ardrone_path, "AR.Drone 2.0")
#    test = dataVisualiser.debugMode(ardrone_path, "AR.Drone 2.0")
#    test.plotAll(groupPlot=False)
#    test.plotTx(groupPlot=True)
#    test.plotRx(groupPlot=True)
#    test.plotLoss_num(groupPlot=True)
#    test.plotLoss_perc(groupPlot=True)
#    test.plotTime_min(groupPlot=True)
#    test.plotTime_avg(groupPlot=True)        
#    test.plotTime_max(groupPlot=True)
#    test.plotTime_mdev(groupPlot=True)
#    test.plotDouble_num(groupPlot=True)
#    test.plotDouble_perc(groupPlot=True)
#    test.plotReadings(groupPlot=True)
#    test.plotRSSI(groupPlot=True)
#    test.plotRSS_perc(groupPlot=True)
#    test.plotRSS_dbm(groupPlot=True)
#    test.plotNoise_dbm(groupPlot=True)
#    test.plotBitrate(groupPlot=True)
    
    anafi_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_150126_(full)_(ANAFI_FIXED_FINAL).npy'
    test2 = dataVisualiser.form_Path(anafi_path, "Anafi")
#    test2 = dataVisualiser.debugMode(anafi_path, "Anafi")
#    test2.plotAll(groupPlot=False)
#    test2.plotTx(groupPlot=True)
#    test2.plotRx(groupPlot=True)
#    test2.plotLoss_num(groupPlot=True)
#    test2.plotLoss_perc(groupPlot=True)
#    test2.plotTime_min(groupPlot=True)
#    test2.plotTime_avg(groupPlot=True)        
#    test2.plotTime_max(groupPlot=True)
#    test2.plotTime_mdev(groupPlot=True)
#    test2.plotDouble_num(groupPlot=True)
#    test2.plotDouble_perc(groupPlot=True)
#    test2.plotReadings(groupPlot=True)
#    test2.plotRSSI(groupPlot=True)
#    test2.plotRSS_perc(groupPlot=True)
#    test2.plotRSS_dbm(groupPlot=True)
#    test2.plotNoise_dbm(groupPlot=True)
#    test2.plotBitrate(groupPlot=True)
    
    test3 = dataVisualiser.from_Substruction(test2,test,"Difference between Anafi and Ar.Drone 2.0")
#    test3.plotAll(groupPlot=False)
    test3.plotTx(groupPlot=True)
#    test3.plotRx(groupPlot=True)
#    test3.plotLoss_num(groupPlot=True)
#    test3.plotLoss_perc(groupPlot=True)
#    test3.plotTime_min(groupPlot=True)
#    test3.plotTime_avg(groupPlot=True)        
#    test3.plotTime_max(groupPlot=True)
#    test3.plotTime_mdev(groupPlot=True)
#    test3.plotDouble_num(groupPlot=True)
#    test3.plotDouble_perc(groupPlot=True)
#    test3.plotReadings(groupPlot=True)
#    test3.plotRSSI(groupPlot=True)
#    test3.plotRSS_perc(groupPlot=True)
#    test3.plotRSS_dbm(groupPlot=True)
#    test3.plotNoise_dbm(groupPlot=True)
#    test3.plotBitrate(groupPlot=True)
    
#    nx,ny,nz = (11,11,4)
#    xx = np.linspace(0,10,nx)
#    yy = np.linspace(0,10,ny)
#    zz = np.linspace(0.5,3.5,nz)
#    newFile = np.full((16,4,11,11),fill_value = -999.9)
#    
#    for z in zz:
#        for x in xx:
#            for y in yy:
#                
#    
#
##    xv,yv = np.meshgrid(x,y)
#    xyv = np.meshgrid(x,y)
#    print(xyv)
    
#    z = np.random.rand(11,11)
#    plt.contourf(x,y,z)
#    plt.show()
    
#    print('\nxv:\n',xv)
#    print('\nyv:\n',yv)
#    print('\nzv:',zv)
    








if __name__ == "__main__":
    main()