import numpy as np
import matplotlib.tri as tri
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import axes3d
from matplotlib import cm
import sys
np.set_printoptions(threshold=sys.maxsize)
#np.set_printoptions(threshold=np.inf)
#======================================================
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
## --------------------------------------------------

# XXX:
class Environment():
    '''This class handles the properties of the environment where the measurements took place'''
    def __init__(self, name, x, x_num,  y, y_num, z, z_num,x_offset=0, y_offset=0, z_offset=0, debug=True):
        
        self.name = name
        
        self.x = x
        self.x_num = x_num
        self.x_offset = x_offset # The offset is applied in both ends
        
        self.y = y
        self.y_num = y_num
        self.y_offset = y_offset # The offset is applied in both ends
        
        self.z = z
        self.z_num = z_num
        self.z_offset = z_offset # The offset is applied in both ends
        
        self.widths = np.linspace(self.x_offset,x-x_offset,x_num)
        self.lengths = np.linspace(self.x_offset,y-y_offset,y_num)
        self.heights = np.linspace(z_offset,z,z_num)
        
        self.debug = debug
        if self.debug:
            print('Object: [',self.name,'] is created!')
    
# XXX:
class HeatmapVisualiser():
    '''This class generates heatmaps for every single variable in different layers. It is also possible to compare two class instances and plot heatmaps from their difference.'''
# TODO:
# 1) Add an argument on te plot function to print only one layer
# 2) Fix the scale of the color bars on all variables
# 3) Create and import the greenhouse layout
# 4) Create a statistics table for each case
# 5) Add units in the colorbar label

    outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/SpatialPinger/PLOTS'
    debug = True
    
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

#   Define the measured heights 
#    layerHeights = (0.5, 1.5, 2.5) # Assuming that we have data from three layers
    measurementPoint = (11, 11.5, 1)    # x,y,z in meters (laptop position)
    
    
    def __init__(self, data, environment, name, propellers, comp=False, text=(None,None)):
        self.dataList = data
        self.droneName = name
        self.environment = environment
        self.distances = self.euclideanDist()
        self.propellers = propellers
        self.objComp = comp
        self.barLim = (0,100)
        if comp:
            self.colormap = "jet"
#            In case of comparison keep the names of the objects that are being compared
            self.L_bartext,self.R_bartext = text
        else:
            self.colormap="inferno"
        self.groupPlot = False
        if self.debug:
            print('Object: [',name,'] is created!')
            
    @classmethod
    def from_Path(cls, path, environment, name, propellers):
        '''Create an object by importing data from an external NPY file'''
        data = np.load(path)
        return cls(data,environment,name,propellers)
    
    @classmethod
    def from_Substruction(cls, obj1, obj2,environment):
        '''Create a new object by substructing the data values of two other objects.'''
        data = obj1.dataList - obj2.dataList
        if obj1.droneName == obj2.droneName:
            L_text = 'Propellers '+obj1.propellers
            R_text = 'Propellers '+obj2.propellers
            name = obj1.droneName
            propellers = obj1.propellers+" vs "+obj2.propellers
        elif obj1.propellers == obj1.propellers:
            L_text = obj1.droneName
            R_text = obj2.droneName
            name = obj1.droneName+" vs "+obj2.droneName
            propellers = obj1.propellers
        else:
            raise ValueError("The drones names nor the proppellers value match.")
        return cls(data, environment, name, propellers, comp=True, text=(L_text,R_text))
    
    def euclideanDist(self):
        '''This function returns the euclidean distances from the measurement 
        point (laptop position during the experiments). The output is an array with dimentions (4x11x11).'''
        
#       Create an array with the same dimentions as the originala data filled with the measurementPoint data.
#       POI (point of interest) is the absolute point in space where the laptop was placed
        POI = np.full((self.dataList.shape[1],self.dataList.shape[2],self.dataList.shape[3],3),self.measurementPoint) #(z,x,y,(xyz POI coordinates))      
#        Create and initialize an array with all the x,y,z values of every point
        pointGrid = np.full((self.dataList.shape[1],self.dataList.shape[2],self.dataList.shape[3],3),fill_value = 0.0)
        
        for i,z in enumerate(self.environment.heights):
            for ii,x in enumerate(self.environment.widths):
                for iii,y in enumerate(self.environment.lengths):
                    pointGrid[i,ii,iii] = x,y,z
#        Calculate and return the Euclidean distances between every point end the point of interest
        return np.round(np.sqrt(np.sum((POI-pointGrid)**2,axis=3)),2)


    def heatmapTx(self,groupPlot=False):
        '''Plot transmitted packets (number)'''
        self.groupPlot = groupPlot
        indx = 0
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTx was executed!")
        
    def heatmapRx(self,groupPlot=False):
        '''Plot received packets (number)'''
        self.groupPlot = groupPlot
        indx = 1
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotΡx was executed!")
        
    def heatmapLoss_num(self,groupPlot=False):
        '''Plot packet loss (number)'''
        self.groupPlot = groupPlot
        indx = 2
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotLoss_num was executed!")

    def heatmapLoss_perc(self,groupPlot=False):
        '''Plot packet loss (percentage)'''
        self.groupPlot = groupPlot
        indx = 3
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotLoss_perc was executed!")
    
    def heatmapTime_min(self,groupPlot=False):
        '''Plot packet received min time'''
        self.groupPlot = groupPlot
        indx = 4
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_min was executed!")
    
    def heatmapTime_avg(self,groupPlot=False):
        '''Plot packet received average time'''
        self.groupPlot = groupPlot
        indx = 5
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_avg was executed!")
        
    def heatmapTime_max(self,groupPlot=False):
        '''Plot packet received max time'''
        self.groupPlot = groupPlot
        indx = 6
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_max was executed!")
    
    def heatmapTime_mdev(self,groupPlot=False):
        '''Plot packet received mean diviation time'''
        self.groupPlot = groupPlot
        indx = 7
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotTime_mdev was executed!")
        
    def heatmapDouble_num(self,groupPlot=False):
        '''Plot double packets (number).'''
        self.groupPlot = groupPlot
        indx = 8
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotDouble_num was executed!")
        
    def heatmapDouble_perc(self, groupPlot=False):
        '''Plot double packets (percentage).'''
        self.groupPlot = groupPlot
        indx = 9
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotDouble_perc was executed!")
    
    def heatmapReadings(self,groupPlot=False):
        '''Plot number of readings per point.'''
        self.groupPlot = groupPlot
        indx = 10
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotReadings was executed!")
        
    def heatmapRSSI(self,groupPlot=False):
        '''Plot RSS Index (0-70).'''
        self.groupPlot = groupPlot
        indx = 11
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotRSSI was executed!")
        
    def heatmapRSS_perc(self,groupPlot=False):
        '''Plot RSS (percentage).'''
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
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotRSS_perc was executed!")
    
    def heatmapRSS_dbm(self,groupPlot=False):
        '''Plot RSS (in dBm).'''
        self.groupPlot = groupPlot
        indx = 13
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotRSS_dbm was executed!")
        
    def heatmapNoise_dbm(self,groupPlot=False):
        '''Plot Noise (in dBm).'''
        self.groupPlot = groupPlot
        indx = 14
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotNoise_dbm was executed!")
        
    def heatmapBitrate(self,groupPlot=False):
        '''Plot Bitrate.'''
        self.groupPlot = groupPlot
        indx = 15
        if self.objComp:
            if abs(np.min(self.dataList[indx])) > abs(np.max(self.dataList[indx])):
                self.barLim = (np.min(self.dataList[indx]),abs(np.min(self.dataList[indx])))
            else:
                self.barLim = (-abs(np.max(self.dataList[indx])),abs(np.max(self.dataList[indx])))
        else:
            self.barLim = (np.min(self.dataList[indx])-np.min(self.dataList[indx])*0.1, np.max(self.dataList[indx])+np.max(self.dataList[indx])*0.1)
            
        if self.groupPlot:
            self.varGroupHeatmap(self.dataList[indx],self.environment.heights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.environment.heights):
                self.varHeatmap(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        if self.debug:
            print("plotBitrate was executed!")
        
    def heatmapAll(self,groupPlot=False):
        '''Plot all variables in different figures'''
        self.heatmapTx(groupPlot)
        self.heatmapRx(groupPlot)
        self.heatmapLoss_num(groupPlot)
        self.heatmapLoss_perc(groupPlot)
        self.heatmapTime_min(groupPlot)
        self.heatmapTime_avg(groupPlot)        
        self.heatmapTime_max(groupPlot)
        self.heatmapTime_mdev(groupPlot)      
        self.heatmapDouble_num(groupPlot)
        self.heatmapDouble_perc(groupPlot)
        self.heatmapReadings(groupPlot)
        self.heatmapRSSI(groupPlot)
        self.heatmapRSS_perc(groupPlot)
        self.heatmapRSS_dbm(groupPlot)
        self.heatmapNoise_dbm(groupPlot)
        self.heatmapBitrate(groupPlot)
        if self.debug:
            print("plotAll was executed!")


    def varGroupHeatmap(self,inputVar,heightList, title, outputName):
        '''Plot in one figure all the layers (heights) of a variable'''
        offset = 0
        fig=plt.figure(figsize=(8*len(inputVar),10))
        for ii,height in enumerate(heightList):
            
            ax = fig.add_subplot(1,len(inputVar),ii+1)
            if self.objComp:
                ax.set_title(self.droneName + " propellers {} ({}m)\n{}".format(self.propellers,height,title))#, title))
            else:
                ax.set_title(self.droneName + " propellers {} ({}m)\n{}".format(self.propellers,height,title))#, title))
                
            plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
            
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")          
            ax.set_xlim(-1,self.environment.x+1) # Set a white area between the border and the graph for the x axis
            ax.set_ylim(self.environment.y+1,-1) # Set a white area between the border and the graph for the y axis

            imgplot = ax.imshow(inputVar[ii], cmap=self.colormap,interpolation='gaussian',extent = [0, 12, 12, 0]) # interpolation='bicubic', origin='lower', interpolation='gaussian'
            plt.tight_layout() # This solves the issue of the missing titles in the exported png file
            imgplot.set_clim(self.barLim)        
            mybar = fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.95,pad=0.08)#.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
            if self.objComp:
                mybar.set_label(title, labelpad=-55)
#                mybar.set_label("< "+self.obj1Name+"   "+title+"   "+self.obj2Name+" >", labelpad=-55)
                plt.figtext(0.034+offset,0.17,self.R_bartext)
                plt.figtext(0.28+offset,0.17,self.L_bartext)
                offset += 0.332

            else:
                mybar.set_label(title, labelpad=-55)   
#                plt.figtext(0.5,0.5,'figtext_2')                    
            for i,x in enumerate(self.environment.widths):
                for j,y in enumerate(self.environment.lengths):
                    ax.scatter(x,y, alpha=0.1, s=800, color="w") # cmap='jet' - plot the white circles 
                    ax.text(y, x, round(inputVar[ii][i][j],2), ha="center", va="center", color="w") # - plot the numbers
            ax.scatter(self.measurementPoint[0],self.measurementPoint[1], alpha=0.5, s=100, color="b", marker="X") # - plot laptop position

#            plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        plt.show()
        self.savePng(fig,self.outputPath,outputName,"all_")
        if self.debug:
            print("varGroupHeatmap was exectuted!")
    
    
    
    def varHeatmap(self,inputVar, height, title, outputName):
        '''Plot a graph of a single variable on a given layer (height)'''
        fig= plt.figure(figsize=(10,10))
        ax = fig.add_subplot(1,1,1)
        ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
        plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
        
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_xlim(-1,self.environment.x+1) # Set a white area between the border and the graph for the x axis
        ax.set_ylim(self.environment.y+1,-1) # Set a white area between the border and the graph for the y axis

        imgplot = ax.imshow(inputVar, cmap=self.colormap,interpolation='gaussian',extent = [0, 12, 12, 0]) # interpolation='bicubic', origin='lower', interpolation='gaussian'
        imgplot.set_clim(self.barLim)
        mybar = fig.colorbar(imgplot, ax=ax, orientation='horizontal',shrink=0.75,pad=0.08)
        if self.objComp:
#            mybar.set_label("< "+self.obj1Name+"   "+title+"   "+self.obj2Name+" >", labelpad=-55)
            mybar.set_label(title, labelpad=-55)
            plt.figtext(0.15,0.16,self.R_bartext)
            plt.figtext(0.75,0.16,self.L_bartext)
        else:
            mybar.set_label(title, labelpad=-55)

        for i,x in enumerate(self.environment.widths):
            for j,y in enumerate(self.environment.lengths):
                ax.scatter(x,y, alpha=0.1, s=800, color="w") # cmap='jet' - plot the white circles 
                ax.text(y, x, round(inputVar[i][j],2), ha="center", va="center", color="w") # - plot the numbers
        ax.scatter(self.measurementPoint[0],self.measurementPoint[1], alpha=0.5, s=100, color="b", marker="X") # - plot laptop position
        
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        self.savePng(fig,self.outputPath,outputName,height)
        plt.show()
        if self.debug:
            print("varHeatmap was exectuted!")
    
    def savePng(self,figure,outputPath,outputName,height):
        '''Save a figure to a PNG file on the provided path.'''
        figure.savefig(outputPath+"/{}_{}_({}m).png".format(self.droneName,outputName,height), dpi=100)  # results in 160x120 px image
        print("Figure: {}_{}_({}m) exported as PNG file.".format(self.droneName,outputName,height))
        if self.debug:
            print("savePng was exectuted!")



def newMain():
   
   ############################## 
   ##### GENERATE HEATMATPS #####
   ##############################
   
##   Create an environment object to group the diamentions of the greenhouse    
#    greenhouse = Environment(name='Greenhouse', x=12,x_num=5,x_offset=1,y=12,y_num=5,y_offset=1,z=2.5,z_num=3,z_offset=0.5)
#
#    
#    anafiPath_off = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/SpatialPinger/NPY/Fix_Anafi_160620_143935@3heights_OFF.npy'
#    anafi_off = HeatmapVisualiser.from_Path(anafiPath_off,greenhouse, "Parrot Anafi", "OFF")
#    anafi_off.heatmapAll(groupPlot=True)
#    
#    anafiPath_on = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/SpatialPinger/NPY/Fix_Anafi_160620_143935@3heights_ON.npy'
#    anafi_on = HeatmapVisualiser.from_Path(anafiPath_on,greenhouse, "Parrot Anafi","ON")
##    anafi_on.heatmapAll(groupPlot=True)
#
#    anafi_on_vs_off = HeatmapVisualiser.from_Substruction(anafi_off,anafi_on,greenhouse)
##    anafi_on_vs_off.heatmapAll(groupPlot=True)
#
#
#
#    ardronePath_off = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/SpatialPinger/NPY/Fix_AR.Drone_2.0_160620_114909@3heights_OFF.npy'
#    ardrone_off = HeatmapVisualiser.from_Path(ardronePath_off,greenhouse, 'AR.Drone 2.0','OFF')
##    ardrone_off.heatmapAll(groupPlot=True)
#
#    ardronePath_on = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/SpatialPinger/NPY/Fix_AR.Drone_2.0_160620_114909@3heights_ON.npy'
#    ardrone_on = HeatmapVisualiser.from_Path(ardronePath_on,greenhouse, 'AR.Drone 2.0','ON')
##    ardrone_on.heatmapAll(groupPlot=True)
#
#    ardrone_on_vs_off = HeatmapVisualiser.from_Substruction(ardrone_off,ardrone_on,greenhouse)
##    ardrone_on_vs_off.heatmapAll(groupPlot=True)
#
#
#
#    ardrone_vs_anafi_off = HeatmapVisualiser.from_Substruction(ardrone_off,anafi_off,greenhouse)
##    ardrone_vs_anafi_off.heatmapAll(groupPlot=True)
#
#    ardrone_vs_anafi_on = HeatmapVisualiser.from_Substruction(ardrone_on,anafi_on,greenhouse)
#    ardrone_vs_anafi_on.heatmapAll(groupPlot=False)
    
   
   
   #################################### 
   ##### GENERATE MAX RANGE PLOTS #####
   ####################################
   
   
#    variables = 12
#    # Greenhouse
#    pathAnafi_OFF = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/Anafi_Greenhouse_240620_141100@120.0m_OFF.npy'
#    pathAnafi_ON = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/Anafi_Greenhouse_240620_141120@120.0m_ON.npy'
#    pathArdrone_OFF = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/AR.Drone 2.0_Greenhouse_240620_152747@120.0m_OFF.npy'
#    pathArdrone_ON = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/AR.Drone 2.0_Greenhouse_240620_152738@120.0m_ON.npy'
#    
#    # Field
#    
#    
#    Anafi_OFF = np.load(pathAnafi_OFF)
#    Anafi_ON = np.load(pathAnafi_ON)
#    Ardrone_OFF = np.load(pathArdrone_OFF)
#    Ardrone_ON = np.load(pathArdrone_ON)
#    
#    
#    max_distance = 120
#    step = max_distance/len(Anafi_OFF[variables])
##    print('step: ',step)
#    range_distances = np.arange(step,max_distance,step).tolist() # convert the generated points from numpy to normal list
#    range_distances.append(max(range_distances)+step)                # This is a trick to include the last point  
##    print('range_distances: ',range_distances)
#       
#    
#    fig = plt.figure(figsize=(10,10))
#    plt.plot(range_distances,Anafi_OFF[variables],range_distances,Anafi_ON[variables],range_distances,Ardrone_OFF[variables],range_distances,Ardrone_ON[variables])
#
#
#
#    # Field
#    pathAnafi_OFF = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/Anafi_Field_260620_083042@120.0m_OFF.npy'
#    pathAnafi_ON = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/Anafi_Field_260620_083121@120.0m_ON.npy'
#    pathArdrone_OFF = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/Ardrone_Field_260620_091515@120.0m_OFF.npy'
#    pathArdrone_ON = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RangeFinder/NPY/Ardrone_Field_260620_091529@120.0m_ON.npy'
#    
#    Anafi_OFF = np.load(pathAnafi_OFF)
#    Anafi_ON = np.load(pathAnafi_ON)
#    Ardrone_OFF = np.load(pathArdrone_OFF)
#    Ardrone_ON = np.load(pathArdrone_ON)
#    
#    fig2 = plt.figure(figsize=(10,10))
#    plt.plot(range_distances,Anafi_OFF[variables],range_distances,Anafi_ON[variables],range_distances,Ardrone_OFF[variables],range_distances,Ardrone_ON[variables])
#    
#    

   #################################### 
   ##### GENERATE SPIDER PLOTS ########
   ####################################
   
    variables = 13
    
    angles = np.arange(0,360,15).tolist()
    angles.append(angles[0]) # The plot is a circle, so we need to "complete the loop" and append the start value to the end.
    norm_angles =[]
    # Normalise angles - Convert degrees to radians
    for angle in angles: 
        norm_angles.append(np.deg2rad(angle)) #57.32
    
    # Anafi paths
    pathAnafi_yaw = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern/NPY/Anafi_260620_103824_Yaw@1m_ON.npy'
    pathAnafi_pitch = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern/NPY/Anafi_260620_105313_Pitch@1m_ON.npy'
    pathAnafi_roll = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern/NPY/Anafi_260620_110448_Roll@1m_ON.npy'

    # Ardrone paths
    pathArdrone_yaw = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern/NPY/Ardrone_260620_094950_Yaw@1m_ON.npy'
    pathArdrone_pitch = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern/NPY/Ardrone_260620_101443_Pitch@1m_ON.npy'
    pathArdrone_roll = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/RadiationPattern/NPY/Ardrone_260620_100228_Roll@1m_ON.npy'

    # Anafi data    
    anafi_yaw = np.load(pathAnafi_yaw)
    anafi_pitch = np.load(pathAnafi_pitch)
    ardrone_roll = np.load(pathAnafi_roll)
   
    # Ardrone data
    ardrone_yaw = np.load(pathArdrone_yaw)[variables].tolist()
    ardrone_yaw.append(ardrone_yaw[0]) # The plot is a circle, so we need to "complete the loop" and append the start value to the end.

    ardrone_pitch = np.load(pathArdrone_pitch)[variables].tolist()
    ardrone_pitch.append(ardrone_pitch[0]) # The plot is a circle, so we need to "complete the loop" and append the start value to the end.

    ardrone_roll = np.load(pathArdrone_roll)[variables].tolist()
    ardrone_roll.append(ardrone_roll[0]) # The plot is a circle, so we need to "complete the loop" and append the start value to the end.
    
    datalist = [ardrone_yaw,ardrone_roll,ardrone_pitch]
    axis_name = ['Yaw - Z axis','Roll -  X axis','Pitch - Y axis']
    colors = ['red','green','blue']
    
    for item,label,color in zip(datalist,axis_name,colors):
        fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(polar=True))
        ax.set_title('Parrot Anafi\nRadiation Pattern (1m)\n\n{}'.format(label), y=1.08)
        
        # Custom label 
        description = plt.figtext(0.62,0.48,'Signal Strength (dBm)',fontsize=12,fontstyle='oblique',fontweight='bold')
        description.set_rotation(0)
    
        # Custom styling.
        ax.set_rlabel_position(180 / 180)                           # Change the position angle of the y axis labels
        ax.tick_params(axis='y', labelsize=12, labelcolor='red')    # Change the y-axis labels size.
        ax.grid(color='#AAAAAA')                                    # Change the color of the circular gridlines.
        ax.set_facecolor('#FAFAFA')                                 # Change the background color inside the circle itself.
        ax.spines['polar'].set_color('#222222')                     # Change the color of the outermost gridline (the spine).
        ax.set_ylim(-100, -30)
        
        # Draw the outline of our data.
        ax.set_thetagrids(angles)
        
        ax.plot(norm_angles, item, color=color, linewidth=1,zorder=1)
        ax.fill(norm_angles, item, color=color, alpha=0.15)
        
        #        im = plt.imread('/home/ros/Downloads/Parrot-ANAFI-PNG-Image-Background.png')
        im = plt.imread('/home/ros/Pictures/file_stats_gray.png')
        implot = plt.imshow(im,zorder=2)

        
        plt.show()




   ####################################### 
   ##### GENERATE 3D SPIDER PLOTS ########
   #######################################
















#    ardrone.HeatmapAll(groupPlot=True)
#    ardrone.HeatmapTx(groupPlot=True)
#    ardrone.HeatmapRx(groupPlot=True)
#    ardrone.HeatmapLoss_num(groupPlot=True)
#    ardrone.HeatmapLoss_perc(groupPlot=True)
#    ardrone.HeatmapTime_min(groupPlot=True)
#    ardrone.HeatmapTime_avg(groupPlot=True)        
#    ardrone.HeatmapTime_max(groupPlot=True)
#    ardrone.HeatmapTime_mdev(groupPlot=True)
#    ardrone.HeatmapDouble_num(groupPlot=True)
#    ardrone.HeatmapDouble_perc(groupPlot=True)
#    ardrone.HeatmapReadings(groupPlot=True)
#    ardrone.HeatmapRSSI(groupPlot=True)
#    ardrone.HeatmapRSS_perc(groupPlot=True)
#    ardrone.HeatmapRSS_dbm(groupPlot=True)
#    ardrone.HeatmapNoise_dbm(groupPlot=True)
#    ardrone.HeatmapBitrate(groupPlot=True)
    
#    ardrone_off_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(full)_(ARDRONE_FIXED_FINAL).npy'
#    ardrone = heatmapVisualiser.form_Path(ardrone_path, "AR.Drone 2.0")
#    
#    anafi_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_150126_(full)_(ANAFI_FIXED_FINAL).npy'
#    anafi = heatmapVisualiser.form_Path(anafi_path, "Anafi")
#
#    anafi_vs_ardrone = heatmapVisualiser.from_Substruction(anafi,ardrone,"Anafi vs AR.Drone 2.0")
    
# XXX:
def  main():

##  14   | Noise Level (dBm)
##  15   | Bitrate (Mb/s)
## --------------------------------------------------
    pkgTx_num = 0
    pkgRx_num = 1
    pkgLoss_num = 2
    pgkLoss_prc = 3
    minTime_ms = 4
    avgTime_ms = 5
    maxTime_ms = 6
    meanDev_ms = 7
    doublePkg_num = 8
    doublePkg_prc = 9
    wifiReadings_num = 10
    RSSI_num = 11
    RSS_prc = 12
    RSS_dbm = 13
    noise_dbm = 14
    bitrate_mbs = 15
    
    
    
##### GENERATE HEAT-MAPS FOR ARDRONE #####
    ardrone_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(full)_(ARDRONE_FIXED_FINAL).npy'
    ardrone = heatmapVisualiser.from_Path(ardrone_path, "AR.Drone 2.0")
#    ardrone.heatmapAll(groupPlot=True)
#    ardrone.heatmapTx(groupPlot=True)
#    ardrone.heatmapRx(groupPlot=True)
#    ardrone.heatmapLoss_num(groupPlot=True)
#    ardrone.heatmapLoss_perc(groupPlot=True)
#    ardrone.heatmapTime_min(groupPlot=True)
#    ardrone.heatmapTime_avg(groupPlot=True)        
#    ardrone.heatmapTime_max(groupPlot=True)
#    ardrone.heatmapTime_mdev(groupPlot=True)
#    ardrone.heatmapDouble_num(groupPlot=True)
#    ardrone.heatmapDouble_perc(groupPlot=True)
#    ardrone.heatmapReadings(groupPlot=True)
#    ardrone.heatmapRSSI(groupPlot=True)
#    ardrone.heatmapRSS_perc(groupPlot=True)
#    ardrone.heatmapRSS_dbm(groupPlot=True)
#    ardrone.heatmapNoise_dbm(groupPlot=True)
#    ardrone.heatmapBitrate(groupPlot=True)

##### GENERATE HEAT-MAPS FOR ANAFI #####
    anafi_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_150126_(full)_(ANAFI_FIXED_FINAL).npy'
    anafi = heatmapVisualiser.from_Path(anafi_path, "Anafi")
#    anafi.heatmapAll(groupPlot=True)
#    anafi.heatmapTx(groupPlot=True)
#    anafi.heatmapRx(groupPlot=True)
#    anafi.heatmapLoss_num(groupPlot=True)
#    anafi.heatmapLoss_perc(groupPlot=True)
#    anafi.heatmapTime_min(groupPlot=True)
#    anafi.heatmapTime_avg(groupPlot=True)        
#    anafi.heatmapTime_max(groupPlot=True)
#    anafi.heatmapTime_mdev(groupPlot=True)
#    anafi.heatmapDouble_num(groupPlot=True)
#    anafi.heatmapDouble_perc(groupPlot=True)
#    anafi.heatmapReadings(groupPlot=True)
#    anafi.heatmapRSSI(groupPlot=True)
#    anafi.heatmapRSS_perc(groupPlot=True)
#    anafi.heatmapRSS_dbm(groupPlot=True)
#    anafi.heatmapNoise_dbm(groupPlot=True)
#    anafi.heatmapBitrate(groupPlot=True)

##### COMPARE THE TWO DRONES BY SUBSTRUCTING THEIR INDIVIDUAL VALUES AND PLOTTING HEAT-MAPS #####
    anafi_vs_ardrone = heatmapVisualiser.from_Substruction(anafi,ardrone,"Anafi vs AR.Drone 2.0")
#    anafi_vs_ardrone.heatmapAll(groupPlot=True)
#    anafi_vs_ardrone.heatmapTx(groupPlot=True)
#    anafi_vs_ardrone.heatmapRx(groupPlot=True)
#    anafi_vs_ardrone.heatmapLoss_num(groupPlot=True)
#    anafi_vs_ardrone.heatmapLoss_perc(groupPlot=True)
#    anafi_vs_ardrone.heatmapTime_min(groupPlot=True)
#    anafi_vs_ardrone.heatmapTime_avg(groupPlot=True)        
#    anafi_vs_ardrone.heatmapTime_max(groupPlot=True)
#    anafi_vs_ardrone.heatmapTime_mdev(groupPlot=True)
#    anafi_vs_ardrone.heatmapDouble_num(groupPlot=True)
#    anafi_vs_ardrone.heatmapDouble_perc(groupPlot=True)
#    anafi_vs_ardrone.heatmapReadings(groupPlot=True)
#    anafi_vs_ardrone.heatmapRSSI(groupPlot=True)
#    anafi_vs_ardrone.heatmapRSS_perc(groupPlot=True)
#    anafi_vs_ardrone.heatmapRSS_dbm(groupPlot=True)
#    anafi_vs_ardrone.heatmapNoise_dbm(groupPlot=True)
#    anafi_vs_ardrone.heatmapBitrate(groupPlot=True)

    def overDistancePlot(obj,var):
        '''This function fits the data linearly and the it generates the plots'''
        #    GroupPlot
        sel = var
        polyDegree = 1
        prt = True
        
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(20,30)) #80*len(anafi.dataList[sel])
    # PLOT 1    
        dists1 = obj.distances[0].reshape(1,-1)[0]
        var1 = obj.dataList[sel][0].reshape(1,-1)[0]
        xp1 = np.linspace(dists1.min(),dists1.max(),100)
    
        ax = fig.add_subplot(4,2,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[0], obj.varTitles[sel]+" vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        anPoly1 = np.polyfit(dists1,var1,polyDegree)
        anp1 = np.poly1d(anPoly1)
        plt.plot(dists1,var1,'.', xp1,anp1(xp1),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 2    
        dists2 = obj.distances[1].reshape(1,-1)[0]
        var2 = obj.dataList[sel][1].reshape(1,-1)[0]
        xp2 = np.linspace(dists2.min(),dists2.max(),100)
    
        ax = fig.add_subplot(4,2,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[1], obj.varTitles[sel]+" vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        anPoly2 = np.polyfit(dists2,var2,polyDegree)
        anp2 = np.poly1d(anPoly2)
        plt.plot(dists2,var2,'.', xp2,anp2(xp2),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 3    
        dists3 = obj.distances[2].reshape(1,-1)[0]
        var3 = obj.dataList[sel][2].reshape(1,-1)[0]
        xp3 = np.linspace(dists3.min(),dists3.max(),100)
    
        ax = fig.add_subplot(4,2,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[2], obj.varTitles[sel]+" vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        anPoly3 = np.polyfit(dists3,var3,polyDegree)
        anp3 = np.poly1d(anPoly3)
        plt.plot(dists3,var3,'.', xp3,anp3(xp3),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 4 
        dists4 = obj.distances[3].reshape(1,-1)[0]
        var4 = obj.dataList[sel][3].reshape(1,-1)[0]
        xp4 = np.linspace(dists4.min(),dists4.max(),100)
    
        ax = fig.add_subplot(4,2,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[3], obj.varTitles[sel]+" vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        anPoly4 = np.polyfit(dists4,var4,polyDegree)
        anp4 = np.poly1d(anPoly4)
        plt.plot(dists4,var4,'.', xp4,anp4(xp4),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    
    # PLOT 5  -  All points of all layers fitted in one graph (one line)
        dists5 = obj.distances.reshape(1,-1)[0]
        var5 = obj.dataList[sel].reshape(1,-1)[0]
        xp5 = np.linspace(dists5.min(),dists5.max(),100)
        
        ax = fig.add_subplot(4,2,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", obj.varTitles[sel]+" vs Distance form PC (meters)"))
    #    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])  
           
        anPoly5 = np.polyfit(dists5,var5,polyDegree)
        anp5 = np.poly1d(anPoly5)
        plt.plot(dists5,var5,'.', xp5,anp5(xp5),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    
    # PLOT 6 - Every layer fit (line) is plotted individually in one graph (five lines) 
        ax = fig.add_subplot(4,2,6)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("All", obj.varTitles[sel]+" vs Distance form PC (meters)"))
        
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        plt.plot(dists1,var1,'.', xp1,anp1(xp1),'-', color='cyan', label='0.5m')
        plt.plot(dists2,var2,'.', xp2,anp2(xp2),'-', color='goldenrod', label='1.5m')
        plt.plot(dists3,var3,'.', xp3,anp3(xp3),'-', color='magenta', label='2.5m')
        plt.plot(dists4,var4,'.', xp4,anp4(xp4),'-', color='teal', label='3.5m')
        plt.plot(xp5,anp5(xp5),'-', color='r', label='All Layers')
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_distance_vs_{}.png".format(obj.droneName,obj.outputNames[sel]), dpi=100)  # results in 160x120 px image
                print("Figure: {}_Distance_vs_{} exported as PNG file.".format(obj.droneName,obj.outputNames[sel]))
        
        plt.show()   
    
    
    
    def dbm(obj):
        '''Calculate the Free Space Path Loss (in dBm)'''
        A = -26 # dbm@1m
        n = 2 # Environmental Path Loss exponent (free space = 2)
#        dist = np.sort(obj.distances[0].reshape(1,-1)[0])
#        dist = obj.distances[0].reshape(1,-1)[0]
        dist = np.linspace(1,15,num=121)
        dbm = -10*n*np.log10(dist)+A # ORIGINAL
#        dbm = 10*n*np.log10(dist/1)+25
#        plt.plot(dist,dbm)
#        plt.show()
        return dbm
    
    def dbm2(obj):
        '''Calculate the Free Space Path Loss (in dBm)'''
        n = 2 # Environmental Path Loss exponent (free space = 2)
        dist = np.linspace(1,45,num=121)

#        10*n*np.log10(8/1) # 6db difference
        
        dbm = 10*n*np.log10(dist/1)+26
        return -dbm
    
    
    def overDistanceLogPlot(obj,var):
        '''This function fits the data linearly and the it generates the plots'''
        
    #    GroupPlot
        sel = var
        polyDegree = 1
        prt = True
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
        _dbm = dbm(obj)
        _dbm2 = dbm2(obj)
    
        fig=plt.figure(figsize=(20,30)) #80*len(anafi.dataList[sel])
    # PLOT 1    
        dists1 = obj.distances[0].reshape(1,-1)[0]
        var1 = obj.dataList[sel][0].reshape(1,-1)[0]
#        xp1 = np.linspace(dists1.min(),dists1.max(),100) #ORIGINAL
        xp1 = np.linspace(dists1.min(),dists1.max(),121)
    
        ax = fig.add_subplot(4,2,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[0], obj.varTitles[sel]+" (log) vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
#        anPoly1 = np.polyfit(np.log(dists1),var1,polyDegree)
        anPoly1 = np.polyfit(dists1,var1,polyDegree)
        anp1 = np.poly1d(anPoly1)
        plt.plot(dists1,var1,'.', xp1,anp1(xp1),'-')
#        plt.plot(np.linspace(1,15,num=121),_dbm,color='red')
        plt.hist(dists1)

        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#        ax.set_xscale('log')
    
    # PLOT 2    
        dists2 = obj.distances[1].reshape(1,-1)[0]
        var2 = obj.dataList[sel][1].reshape(1,-1)[0]
#        xp2 = np.linspace(dists2.min(),dists2.max(),100) # ORIGINAL
        xp2 = np.linspace(dists2.min(),dists2.max(),121)
    
        ax = fig.add_subplot(4,2,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[1], obj.varTitles[sel]+" (log) vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        anPoly2 = np.polyfit(dists2,var2,polyDegree)
        anp2 = np.poly1d(anPoly2)
        plt.plot(dists2,var2,'.', xp2,anp2(xp2),'-')
        plt.plot(np.linspace(1,15,num=121),_dbm,color='red')
#        plt.plot(np.linspace(1,15,num=121),_dbm,color='red')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#        ax.set_xscale('log')
    
    # PLOT 3    
        dists3 = obj.distances[2].reshape(1,-1)[0]
        var3 = obj.dataList[sel][2].reshape(1,-1)[0]
        xp3 = np.linspace(dists3.min(),dists3.max(),121)
    
        ax = fig.add_subplot(4,2,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[2], obj.varTitles[sel]+" (log) vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        anPoly3 = np.polyfit(dists3,var3,polyDegree)
        anp3 = np.poly1d(anPoly3)
        plt.plot(dists3,var3,'.', xp3,anp3(xp3),'-')
        plt.plot(np.linspace(1,15,num=121),_dbm,color='red')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#        ax.set_xscale('log')
    
    # PLOT 4 
        dists4 = obj.distances[3].reshape(1,-1)[0]
        var4 = obj.dataList[sel][3].reshape(1,-1)[0]
        xp4 = np.linspace(dists4.min(),dists4.max(),121)
    
        ax = fig.add_subplot(4,2,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[3], obj.varTitles[sel]+" (log) vs Distance form PC (meters)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        anPoly4 = np.polyfit(dists4,var4,polyDegree)
        anp4 = np.poly1d(anPoly4)
        plt.plot(dists4,var4,'.', xp4,anp4(xp4),'-')
        plt.plot(np.linspace(1,15,num=121),_dbm,color='red')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#        ax.set_xscale('log')
    
    
    # PLOT 5  - All points of all layers fitted in one graph (one line)
        dists5 = obj.distances.reshape(1,-1)[0]
        var5 = obj.dataList[sel].reshape(1,-1)[0]
        xp5 = np.linspace(dists5.min(),dists5.max(),121)
        
        ax = fig.add_subplot(4,2,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", obj.varTitles[sel]+" (log) vs Distance form PC (meters)"))
    #    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])  
           
        anPoly5 = np.polyfit(dists5,var5,polyDegree)
        anp5 = np.poly1d(anPoly5)
        plt.plot(dists5,var5,'.', xp5,anp5(xp5),'-')
        plt.plot(np.linspace(1,16,num=121),_dbm,color='red')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#        ax.set_xscale('log')
            
    
    # PLOT 6 - Every layer fit (line) is plotted individually in one graph (five lines) 
        ax = fig.add_subplot(4,2,6)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("All", obj.varTitles[sel]+" (log) vs Distance form PC (meters)"))
        
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance form PC (meters)")
        ax.set_ylabel(obj.varTitles[sel])
        
        plt.plot(dists1,var1,'.', xp1,anp1(xp1),'-', color='cyan', label='0.5m')
        plt.plot(dists2,var2,'.', xp2,anp2(xp2),'-', color='goldenrod', label='1.5m')
        plt.plot(dists3,var3,'.', xp3,anp3(xp3),'-', color='magenta', label='2.5m')
        plt.plot(dists4,var4,'.', xp4,anp4(xp4),'-', color='teal', label='3.5m')
        plt.plot(xp5,anp5(xp5),'-', color='r', label='All Layers')
        plt.legend()
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#        ax.set_xscale('log')
        
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_distance_vs_{}_logfit.png".format(obj.droneName,obj.outputNames[sel]), dpi=100)  # results in 160x120 px image
                print("Figure: {}_Distance_vs_{}_logfit exported as PNG file.".format(obj.droneName,obj.outputNames[sel]))
        
#        ax.set_yscale('log') 
        plt.show()
           
    def overRSSPlot(obj,var):
        '''This function fits the data linearly and the it generates the plots'''
        #    GroupPlot
        sel = var
        polyDegree = 1
        prt = True
        
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(20,30)) #80*len(anafi.dataList[sel])
    # PLOT 1    
        RSS1 = obj.dataList[RSS_dbm][0].reshape(1,-1)[0]
        var1 = obj.dataList[sel][0].reshape(1,-1)[0]
        xp1 = np.linspace(RSS1.min(),RSS1.max(),100)
    
        ax = fig.add_subplot(4,2,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[0], obj.varTitles[sel]+" vs Signal Strength (dbm)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Signal Strength (dbm)")
        ax.set_ylabel(obj.varTitles[sel])
        
#        Fit and plot data
        anPoly1 = np.polyfit(RSS1,var1,polyDegree)
        anp1 = np.poly1d(anPoly1)
        plt.plot(RSS1,var1,'.', xp1,anp1(xp1),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 2    
        RSS2 = obj.dataList[RSS_dbm][1].reshape(1,-1)[0]
        var2 = obj.dataList[sel][1].reshape(1,-1)[0]
        xp2 = np.linspace(RSS2.min(),RSS2.max(),100)
    
        ax = fig.add_subplot(4,2,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[1], obj.varTitles[sel]+" vs Signal Strength (dbm)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Signal Strength (dbm)")
        ax.set_ylabel(obj.varTitles[sel])
        
#        Fit and plot data
        anPoly2 = np.polyfit(RSS2,var2,polyDegree)
        anp2 = np.poly1d(anPoly2)
        plt.plot(RSS2,var2,'.', xp2,anp2(xp2),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 3    
        RSS3 = obj.dataList[RSS_dbm][2].reshape(1,-1)[0]
        var3 = obj.dataList[sel][2].reshape(1,-1)[0]
        xp3 = np.linspace(RSS3.min(),RSS3.max(),100)
    
        ax = fig.add_subplot(4,2,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[2], obj.varTitles[sel]+" vs Signal Strength (dbm)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Signal Strength (dbm)")
        ax.set_ylabel(obj.varTitles[sel])
        
#        Fit and plot data
        anPoly3 = np.polyfit(RSS3,var3,polyDegree)
        anp3 = np.poly1d(anPoly3)
        plt.plot(RSS3,var3,'.', xp3,anp3(xp3),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 4 
        RSS4 = obj.dataList[RSS_dbm][3].reshape(1,-1)[0]
        var4 = obj.dataList[sel][3].reshape(1,-1)[0]
        xp4 = np.linspace(RSS4.min(),RSS4.max(),100)
    
        ax = fig.add_subplot(4,2,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[3], obj.varTitles[sel]+" vs Signal Strength (dbm)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Signal Strength (dbm)")
        ax.set_ylabel(obj.varTitles[sel])

#        Fit and plot data
        anPoly4 = np.polyfit(RSS4,var4,polyDegree)
        anp4 = np.poly1d(anPoly4)
        plt.plot(RSS4,var4,'.', xp4,anp4(xp4),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    
    # PLOT 5  -  All points of all layers fitted in one graph (one line)
        RSS5 = obj.dataList[RSS_dbm].reshape(1,-1)[0]
        var5 = obj.dataList[sel].reshape(1,-1)[0]
        xp5 = np.linspace(RSS5.min(),RSS5.max(),100)
        
        ax = fig.add_subplot(4,2,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", obj.varTitles[sel]+" vs Signal Strength (dbm)"))
    #    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Signal Strength (dbm)")
        ax.set_ylabel(obj.varTitles[sel])  

#        Fit and plot data           
        anPoly5 = np.polyfit(RSS5,var5,polyDegree)
        anp5 = np.poly1d(anPoly5)
        plt.plot(RSS5,var5,'.', xp5,anp5(xp5),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    
    # PLOT 6 - Every layer fit (line) is plotted individually in one graph (five lines) 
        ax = fig.add_subplot(4,2,6)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("All", obj.varTitles[sel]+" vs Signal Strength (dbm)"))
        
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Signal Strength (dbm)")
        ax.set_ylabel(obj.varTitles[sel])
        
        plt.plot(RSS1,var1,'.', xp1,anp1(xp1),'-', color='cyan', label='0.5m')
        plt.plot(RSS2,var2,'.', xp2,anp2(xp2),'-', color='goldenrod', label='1.5m')
        plt.plot(RSS3,var3,'.', xp3,anp3(xp3),'-', color='magenta', label='2.5m')
        plt.plot(RSS4,var4,'.', xp4,anp4(xp4),'-', color='teal', label='3.5m')
        plt.plot(xp5,anp5(xp5),'-', color='r', label='All Layers')
        plt.legend()

#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_distance_vs_{}.png".format(obj.droneName,obj.outputNames[sel]), dpi=100)  # results in 160x120 px image
                print("Figure: {}_RSS_vs_{} exported as PNG file.".format(obj.droneName,obj.outputNames[sel]))
        
        plt.show()
    
    def overBitratePlot(obj,var):
        '''This function fits the data linearly and the it generates the plots'''
        #    GroupPlot
        sel = var
        polyDegree = 1
        prt = True
        
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(20,30)) #80*len(anafi.dataList[sel])
    # PLOT 1    
        BIRTARE1 = obj.dataList[bitrate_mbs][0].reshape(1,-1)[0]
        var1 = obj.dataList[sel][0].reshape(1,-1)[0]
        xp1 = np.linspace(BIRTARE1.min(),BIRTARE1.max(),100)
    
        ax = fig.add_subplot(4,2,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[0], obj.varTitles[sel]+" vs Bitrate (Mb/s)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mb/s)")
        ax.set_ylabel(obj.varTitles[sel])
        
#        Fit and plot data
        anPoly1 = np.polyfit(BIRTARE1,var1,polyDegree)
        anp1 = np.poly1d(anPoly1)
        plt.plot(BIRTARE1,var1,'.', xp1,anp1(xp1),'-')
        
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 2    
        BIRTARE2 = obj.dataList[bitrate_mbs][1].reshape(1,-1)[0]
        var2 = obj.dataList[sel][1].reshape(1,-1)[0]
        xp2 = np.linspace(BIRTARE2.min(),BIRTARE2.max(),100)
    
        ax = fig.add_subplot(4,2,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[1], obj.varTitles[sel]+" vs Bitrate (Mb/s)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mb/s)")
        ax.set_ylabel(obj.varTitles[sel])
        
#        Fit and plot data
        anPoly2 = np.polyfit(BIRTARE2,var2,polyDegree)
        anp2 = np.poly1d(anPoly2)
        plt.plot(BIRTARE2,var2,'.', xp2,anp2(xp2),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 3    
        BIRTARE3 = obj.dataList[bitrate_mbs][2].reshape(1,-1)[0]
        var3 = obj.dataList[sel][2].reshape(1,-1)[0]
        xp3 = np.linspace(BIRTARE3.min(),BIRTARE3.max(),100)
    
        ax = fig.add_subplot(4,2,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[2], obj.varTitles[sel]+" vs Bitrate (Mb/s)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mb/s)")
        ax.set_ylabel(obj.varTitles[sel])
        
#        Fit and plot data
        anPoly3 = np.polyfit(BIRTARE3,var3,polyDegree)
        anp3 = np.poly1d(anPoly3)
        plt.plot(BIRTARE3,var3,'.', xp3,anp3(xp3),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    # PLOT 4 
        BIRTARE4 = obj.dataList[bitrate_mbs][3].reshape(1,-1)[0]
        var4 = obj.dataList[sel][3].reshape(1,-1)[0]
        xp4 = np.linspace(BIRTARE4.min(),BIRTARE4.max(),100)
    
        ax = fig.add_subplot(4,2,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[3], obj.varTitles[sel]+" vs Bitrate (Mb/s)"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mb/s)")
        ax.set_ylabel(obj.varTitles[sel])

#        Fit and plot data
        anPoly4 = np.polyfit(BIRTARE4,var4,polyDegree)
        anp4 = np.poly1d(anPoly4)
        plt.plot(BIRTARE4,var4,'.', xp4,anp4(xp4),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    
    # PLOT 5  -  All points of all layers fitted in one graph (one line)
        BIRTARE5 = obj.dataList[bitrate_mbs].reshape(1,-1)[0]
        var5 = obj.dataList[sel].reshape(1,-1)[0]
        xp5 = np.linspace(BIRTARE5.min(),BIRTARE5.max(),100)
        
        ax = fig.add_subplot(4,2,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", obj.varTitles[sel]+" vs Bitrate (Mb/s)"))
    #    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mb/s)")
        ax.set_ylabel(obj.varTitles[sel])  

#        Fit and plot data           
        anPoly5 = np.polyfit(BIRTARE5,var5,polyDegree)
        anp5 = np.poly1d(anPoly5)
        plt.plot(BIRTARE5,var5,'.', xp5,anp5(xp5),'-')
        
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
    
    # PLOT 6 - Every layer fit (line) is plotted individually in one graph (five lines) 
        ax = fig.add_subplot(4,2,6)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("All", obj.varTitles[sel]+" vs Bitrate (Mbs)"))
        
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mb/s)")
        ax.set_ylabel(obj.varTitles[sel])
        
        plt.plot(BIRTARE1,var1,'.', xp1,anp1(xp1),'-', color='cyan', label='0.5m')
        plt.plot(BIRTARE2,var2,'.', xp2,anp2(xp2),'-', color='goldenrod', label='1.5m')
        plt.plot(BIRTARE3,var3,'.', xp3,anp3(xp3),'-', color='magenta', label='2.5m')
        plt.plot(BIRTARE4,var4,'.', xp4,anp4(xp4),'-', color='teal', label='3.5m')
        plt.plot(xp5,anp5(xp5),'-', color='r', label='All Layers')
        plt.legend()

#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_distance_vs_{}.png".format(obj.droneName,obj.outputNames[sel]), dpi=100)  # results in 160x120 px image
                print("Figure: {}_Bitrate_vs_{} exported as PNG file.".format(obj.droneName,obj.outputNames[sel]))
        
        plt.show()
    
    def timeOverDistancePlot(obj):
        '''This function fits the data linearly and the it generates the plots'''
        polyDegree = 1
        prt = True
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(10,20)) #80*len(anafi.dataList[sel])
        plotLine = np.linspace(0,15,121)
        
    # PLOT 1   
        height = 0
        dists1 = obj.distances[height].reshape(1,-1)[0]
        maxTime1 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime1 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime1 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
    
        ax = fig.add_subplot(5,1,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Distance"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance (meters)")
        ax.set_ylabel("Round-Trip Time (ms)")
        
        
        maxTimeFit1 = np.polyfit(dists1,maxTime1,polyDegree)
        maxTimep1 = np.poly1d(maxTimeFit1)
        plt.plot(plotLine, maxTimep1(plotLine),'--',color='red')
        plt.scatter(dists1,maxTime1,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit1 = np.polyfit(dists1,avgTime1,polyDegree)
        avgTimep1 = np.poly1d(avgTimeFit1)
        plt.plot(plotLine, avgTimep1(plotLine),'-',color='blue')
        plt.scatter(dists1,avgTime1,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit1 = np.polyfit(dists1,minTime1,polyDegree)
        minTimep1 = np.poly1d(minTimeFit1)
        plt.plot(plotLine, minTimep1(plotLine),'--',color='green')
        plt.scatter(dists1,minTime1,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 2  
        height = 1
        
        ax = fig.add_subplot(5,1,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Distance"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance (meters)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        dists2 = obj.distances[height].reshape(1,-1)[0]
        maxTime2 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime2 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime2 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit2 = np.polyfit(dists2,maxTime2,polyDegree)
        maxTimep2 = np.poly1d(maxTimeFit2)
        plt.plot(plotLine, maxTimep2(plotLine),'--',color='red')
        plt.scatter(dists2,maxTime2,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit2 = np.polyfit(dists2,avgTime2,polyDegree)
        avgTimep2 = np.poly1d(avgTimeFit2)
        plt.plot(plotLine, avgTimep2(plotLine),'-',color='blue')
        plt.scatter(dists2,avgTime2,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit2 = np.polyfit(dists2,minTime2,polyDegree)
        minTimep2 = np.poly1d(minTimeFit2)
        plt.plot(plotLine, minTimep2(plotLine),'--',color='green')
        plt.scatter(dists2,minTime2,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 3    
        height = 2
        
        ax = fig.add_subplot(5,1,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Distance"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance (meters)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        dists3 = obj.distances[height].reshape(1,-1)[0]
        maxTime3 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime3 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime3 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit3 = np.polyfit(dists3,maxTime3,polyDegree)
        maxTimep3 = np.poly1d(maxTimeFit3)
        plt.plot(plotLine, maxTimep3(plotLine),'--',color='red')
        plt.scatter(dists3,maxTime3,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit3 = np.polyfit(dists3,avgTime3,polyDegree)
        avgTimep3 = np.poly1d(avgTimeFit3)
        plt.plot(plotLine, avgTimep3(plotLine),'-',color='blue')
        plt.scatter(dists3,avgTime3,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit3 = np.polyfit(dists3,minTime3,polyDegree)
        minTimep3 = np.poly1d(minTimeFit3)
        plt.plot(plotLine, minTimep3(plotLine),'--',color='green')
        plt.scatter(dists3,minTime3,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 4 
        height = 3
        
        ax = fig.add_subplot(5,1,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Distance"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance (meters)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        dists4 = obj.distances[height].reshape(1,-1)[0]
        maxTime4 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime4 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime4 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit4 = np.polyfit(dists4,maxTime4,polyDegree)
        maxTimep4 = np.poly1d(maxTimeFit4)
        plt.plot(plotLine, maxTimep4(plotLine),'--',color='red')
        plt.scatter(dists4,maxTime4,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit4 = np.polyfit(dists4,avgTime4,polyDegree)
        avgTimep4 = np.poly1d(avgTimeFit4)
        plt.plot(plotLine, avgTimep4(plotLine),'-',color='blue')
        plt.scatter(dists4,avgTime4,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit4 = np.polyfit(dists4,minTime4,polyDegree)
        minTimep4 = np.poly1d(minTimeFit4)
        plt.plot(plotLine, minTimep4(plotLine),'--',color='green')
        plt.scatter(dists4,minTime4,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
#    
#    # PLOT 5  -  All points of all layers fitted in one graph (one line)
        height = 3
        
        ax = fig.add_subplot(5,1,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", "Round-Trip Time vs Distance"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Distance (meters)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        dists5 = obj.distances.reshape(1,-1)[0]
        maxTime5 = obj.dataList[maxTime_ms].reshape(1,-1)[0]
        avgTime5 = obj.dataList[avgTime_ms].reshape(1,-1)[0]
        minTime5 = obj.dataList[minTime_ms].reshape(1,-1)[0]
        
        maxTimeFit5 = np.polyfit(dists5,maxTime5,polyDegree)
        maxTimep5 = np.poly1d(maxTimeFit5)
        plt.plot(plotLine, maxTimep5(plotLine),'--',color='red')
        plt.scatter(dists5,maxTime5,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit5 = np.polyfit(dists5,avgTime5,polyDegree)
        avgTimep5 = np.poly1d(avgTimeFit5)
        plt.plot(plotLine, avgTimep5(plotLine),'-',color='blue')
        plt.scatter(dists5,avgTime5,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit5 = np.polyfit(dists5,minTime5,polyDegree)
        minTimep5 = np.poly1d(minTimeFit5)
        plt.plot(plotLine, minTimep5(plotLine),'--',color='green')
        plt.scatter(dists5,minTime5,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_Distance_vs_Times.png".format(obj.droneName), dpi=100)  # results in 160x120 px image
                print("Figure: {}_Distance_vs_Times exported as PNG file.".format(obj.droneName))
        
        plt.show() 
    
    def timeOverRSSPlot(obj):
        '''This function fits the data linearly and the it generates the plots'''
        polyDegree = 1
        prt = True
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(10,20)) #80*len(anafi.dataList[sel])
        plotLine = np.linspace(obj.dataList[RSS_dbm].min(),obj.dataList[RSS_dbm].max(),121)
        
    # PLOT 1   
        height = 0
        RSS1 = obj.dataList[RSS_dbm][height].reshape(1,-1)[0]
        maxTime1 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime1 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime1 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
    
        ax = fig.add_subplot(5,1,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Received Signal Strength"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Received Signal Strength (dbm)")
        ax.set_ylabel("Round-Trip Time (ms)")
        
        
        maxTimeFit1 = np.polyfit(RSS1,maxTime1,polyDegree)
        maxTimep1 = np.poly1d(maxTimeFit1)
        plt.plot(plotLine, maxTimep1(plotLine),'--',color='red')
        plt.scatter(RSS1,maxTime1,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit1 = np.polyfit(RSS1,avgTime1,polyDegree)
        avgTimep1 = np.poly1d(avgTimeFit1)
        plt.plot(plotLine, avgTimep1(plotLine),'-',color='blue')
        plt.scatter(RSS1,avgTime1,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit1 = np.polyfit(RSS1,minTime1,polyDegree)
        minTimep1 = np.poly1d(minTimeFit1)
        plt.plot(plotLine, minTimep1(plotLine),'--',color='green')
        plt.scatter(RSS1,minTime1,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 2  
        height = 1
        
        ax = fig.add_subplot(5,1,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Received Signal Strength"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Received Signal Strength (dbm)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        RSS2 = obj.dataList[RSS_dbm][height].reshape(1,-1)[0]
        maxTime2 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime2 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime2 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit2 = np.polyfit(RSS2,maxTime2,polyDegree)
        maxTimep2 = np.poly1d(maxTimeFit2)
        plt.plot(plotLine, maxTimep2(plotLine),'--',color='red')
        plt.scatter(RSS2,maxTime2,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit2 = np.polyfit(RSS2,avgTime2,polyDegree)
        avgTimep2 = np.poly1d(avgTimeFit2)
        plt.plot(plotLine, avgTimep2(plotLine),'-',color='blue')
        plt.scatter(RSS2,avgTime2,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit2 = np.polyfit(RSS2,minTime2,polyDegree)
        minTimep2 = np.poly1d(minTimeFit2)
        plt.plot(plotLine, minTimep2(plotLine),'--',color='green')
        plt.scatter(RSS2,minTime2,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 3    
        height = 2
        
        ax = fig.add_subplot(5,1,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Received Signal Strength"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Received Signal Strength (dbm)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        RSS3 = obj.dataList[RSS_dbm][height].reshape(1,-1)[0]
        maxTime3 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime3 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime3 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit3 = np.polyfit(RSS3,maxTime3,polyDegree)
        maxTimep3 = np.poly1d(maxTimeFit3)
        plt.plot(plotLine, maxTimep3(plotLine),'--',color='red')
        plt.scatter(RSS3,maxTime3,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit3 = np.polyfit(RSS3,avgTime3,polyDegree)
        avgTimep3 = np.poly1d(avgTimeFit3)
        plt.plot(plotLine, avgTimep3(plotLine),'-',color='blue')
        plt.scatter(RSS3,avgTime3,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit3 = np.polyfit(RSS3,minTime3,polyDegree)
        minTimep3 = np.poly1d(minTimeFit3)
        plt.plot(plotLine, minTimep3(plotLine),'--',color='green')
        plt.scatter(RSS3,minTime3,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 4 
        height = 3
        
        ax = fig.add_subplot(5,1,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Received Signal Strength"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Received Signal Strength (dbm)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        RSS4 = obj.dataList[RSS_dbm][height].reshape(1,-1)[0]
        maxTime4 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime4 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime4 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit4 = np.polyfit(RSS4,maxTime4,polyDegree)
        maxTimep4 = np.poly1d(maxTimeFit4)
        plt.plot(plotLine, maxTimep4(plotLine),'--',color='red')
        plt.scatter(RSS4,maxTime4,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit4 = np.polyfit(RSS4,avgTime4,polyDegree)
        avgTimep4 = np.poly1d(avgTimeFit4)
        plt.plot(plotLine, avgTimep4(plotLine),'-',color='blue')
        plt.scatter(RSS4,avgTime4,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit4 = np.polyfit(RSS4,minTime4,polyDegree)
        minTimep4 = np.poly1d(minTimeFit4)
        plt.plot(plotLine, minTimep4(plotLine),'--',color='green')
        plt.scatter(RSS4,minTime4,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
#    
#    # PLOT 5  -  All points of all layers fitted in one graph (one line)      
        ax = fig.add_subplot(5,1,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", "Round-Trip Time vs Received Signal Strength"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Received Signal Strength (dbm)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        RSS5 = obj.dataList[RSS_dbm].reshape(1,-1)[0]
        maxTime5 = obj.dataList[maxTime_ms].reshape(1,-1)[0]
        avgTime5 = obj.dataList[avgTime_ms].reshape(1,-1)[0]
        minTime5 = obj.dataList[minTime_ms].reshape(1,-1)[0]
        
        maxTimeFit5 = np.polyfit(RSS5,maxTime5,polyDegree)
        maxTimep5 = np.poly1d(maxTimeFit5)
        plt.plot(plotLine, maxTimep5(plotLine),'--',color='red')
        plt.scatter(RSS5,maxTime5,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit5 = np.polyfit(RSS5,avgTime5,polyDegree)
        avgTimep5 = np.poly1d(avgTimeFit5)
        plt.plot(plotLine, avgTimep5(plotLine),'-',color='blue')
        plt.scatter(RSS5,avgTime5,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit5 = np.polyfit(RSS5,minTime5,polyDegree)
        minTimep5 = np.poly1d(minTimeFit5)
        plt.plot(plotLine, minTimep5(plotLine),'--',color='green')
        plt.scatter(RSS5,minTime5,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_RSS_vs_Times.png".format(obj.droneName), dpi=100)  # results in 160x120 px image
                print("Figure: {}_RSS_vs_Times exported as PNG file.".format(obj.droneName))
        
        plt.show()

    def timeOverBitratePlot(obj):
        '''This function fits the data linearly and the it generates the plots'''
        polyDegree = 1
        prt = True
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(10,20)) #80*len(anafi.dataList[sel])
        plotLine = np.linspace(obj.dataList[bitrate_mbs].min(),obj.dataList[bitrate_mbs].max(),121)
        
    # PLOT 1   
        height = 0
        Bitrate1 = obj.dataList[bitrate_mbs][height].reshape(1,-1)[0]
        maxTime1 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime1 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime1 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
    
        ax = fig.add_subplot(5,1,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Bitrate"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mbit/s)")
        ax.set_ylabel("Round-Trip Time (ms)")
        
        
        maxTimeFit1 = np.polyfit(Bitrate1,maxTime1,polyDegree)
        maxTimep1 = np.poly1d(maxTimeFit1)
        plt.plot(plotLine, maxTimep1(plotLine),'--',color='red')
        plt.scatter(Bitrate1,maxTime1,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit1 = np.polyfit(Bitrate1,avgTime1,polyDegree)
        avgTimep1 = np.poly1d(avgTimeFit1)
        plt.plot(plotLine, avgTimep1(plotLine),'-',color='blue')
        plt.scatter(Bitrate1,avgTime1,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit1 = np.polyfit(Bitrate1,minTime1,polyDegree)
        minTimep1 = np.poly1d(minTimeFit1)
        plt.plot(plotLine, minTimep1(plotLine),'--',color='green')
        plt.scatter(Bitrate1,minTime1,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 2  
        height = 1
        
        ax = fig.add_subplot(5,1,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Bitrate"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mbit/s)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        Bitrate2 = obj.dataList[bitrate_mbs][height].reshape(1,-1)[0]
        maxTime2 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime2 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime2 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit2 = np.polyfit(Bitrate2,maxTime2,polyDegree)
        maxTimep2 = np.poly1d(maxTimeFit2)
        plt.plot(plotLine, maxTimep2(plotLine),'--',color='red')
        plt.scatter(Bitrate2,maxTime2,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit2 = np.polyfit(Bitrate2,avgTime2,polyDegree)
        avgTimep2 = np.poly1d(avgTimeFit2)
        plt.plot(plotLine, avgTimep2(plotLine),'-',color='blue')
        plt.scatter(Bitrate2,avgTime2,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit2 = np.polyfit(Bitrate2,minTime2,polyDegree)
        minTimep2 = np.poly1d(minTimeFit2)
        plt.plot(plotLine, minTimep2(plotLine),'--',color='green')
        plt.scatter(Bitrate2,minTime2,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 3    
        height = 2
        
        ax = fig.add_subplot(5,1,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Bitrate"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mbit/s)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        Bitrate3 = obj.dataList[bitrate_mbs][height].reshape(1,-1)[0]
        maxTime3 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime3 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime3 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit3 = np.polyfit(Bitrate3,maxTime3,polyDegree)
        maxTimep3 = np.poly1d(maxTimeFit3)
        plt.plot(plotLine, maxTimep3(plotLine),'--',color='red')
        plt.scatter(Bitrate3,maxTime3,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit3 = np.polyfit(Bitrate3,avgTime3,polyDegree)
        avgTimep3 = np.poly1d(avgTimeFit3)
        plt.plot(plotLine, avgTimep3(plotLine),'-',color='blue')
        plt.scatter(Bitrate3,avgTime3,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit3 = np.polyfit(Bitrate3,minTime3,polyDegree)
        minTimep3 = np.poly1d(minTimeFit3)
        plt.plot(plotLine, minTimep3(plotLine),'--',color='green')
        plt.scatter(Bitrate3,minTime3,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 4 
        height = 3
        
        ax = fig.add_subplot(5,1,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Bitrate"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mbit/s)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        Bitrate4 = obj.dataList[bitrate_mbs][height].reshape(1,-1)[0]
        maxTime4 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime4 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime4 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit4 = np.polyfit(Bitrate4,maxTime4,polyDegree)
        maxTimep4 = np.poly1d(maxTimeFit4)
        plt.plot(plotLine, maxTimep4(plotLine),'--',color='red')
        plt.scatter(Bitrate4,maxTime4,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit4 = np.polyfit(Bitrate4,avgTime4,polyDegree)
        avgTimep4 = np.poly1d(avgTimeFit4)
        plt.plot(plotLine, avgTimep4(plotLine),'-',color='blue')
        plt.scatter(Bitrate4,avgTime4,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit4 = np.polyfit(Bitrate4,minTime4,polyDegree)
        minTimep4 = np.poly1d(minTimeFit4)
        plt.plot(plotLine, minTimep4(plotLine),'--',color='green')
        plt.scatter(Bitrate4,minTime4,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
#    
#    # PLOT 5  -  All points of all layers fitted in one graph (one line)      
        ax = fig.add_subplot(5,1,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", "Round-Trip Time vs Bitrate"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Bitrate (Mbit/s)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        Bitrate5 = obj.dataList[bitrate_mbs].reshape(1,-1)[0]
        maxTime5 = obj.dataList[maxTime_ms].reshape(1,-1)[0]
        avgTime5 = obj.dataList[avgTime_ms].reshape(1,-1)[0]
        minTime5 = obj.dataList[minTime_ms].reshape(1,-1)[0]
        
        maxTimeFit5 = np.polyfit(Bitrate5,maxTime5,polyDegree)
        maxTimep5 = np.poly1d(maxTimeFit5)
        plt.plot(plotLine, maxTimep5(plotLine),'--',color='red')
        plt.scatter(Bitrate5,maxTime5,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit5 = np.polyfit(Bitrate5,avgTime5,polyDegree)
        avgTimep5 = np.poly1d(avgTimeFit5)
        plt.plot(plotLine, avgTimep5(plotLine),'-',color='blue')
        plt.scatter(Bitrate5,avgTime5,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit5 = np.polyfit(Bitrate5,minTime5,polyDegree)
        minTimep5 = np.poly1d(minTimeFit5)
        plt.plot(plotLine, minTimep5(plotLine),'--',color='green')
        plt.scatter(Bitrate5,minTime5,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_Bitrate_vs_Times.png".format(obj.droneName), dpi=100)  # results in 160x120 px image
                print("Figure: {}_Bitrate_vs_Times exported as PNG file.".format(obj.droneName))
        
        plt.show()

    def timeOverDoublepkgPlot(obj):
        '''This function fits the data linearly and the it generates the plots'''
        polyDegree = 1
        prt = True
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(10,20)) #80*len(anafi.dataList[sel])
        plotLine = np.linspace(obj.dataList[doublePkg_num].min(),obj.dataList[doublePkg_num].max(),121)
        
    # PLOT 1   
        height = 0
        DoublePkg1 = obj.dataList[doublePkg_num][height].reshape(1,-1)[0]
        maxTime1 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime1 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime1 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
    
        ax = fig.add_subplot(5,1,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Duplicate Packages"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Duplicate Packages (#)")
        ax.set_ylabel("Round-Trip Time (ms)")
        
        
        maxTimeFit1 = np.polyfit(DoublePkg1,maxTime1,polyDegree)
        maxTimep1 = np.poly1d(maxTimeFit1)
        plt.plot(plotLine, maxTimep1(plotLine),'--',color='red')
        plt.scatter(DoublePkg1,maxTime1,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit1 = np.polyfit(DoublePkg1,avgTime1,polyDegree)
        avgTimep1 = np.poly1d(avgTimeFit1)
        plt.plot(plotLine, avgTimep1(plotLine),'-',color='blue')
        plt.scatter(DoublePkg1,avgTime1,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit1 = np.polyfit(DoublePkg1,minTime1,polyDegree)
        minTimep1 = np.poly1d(minTimeFit1)
        plt.plot(plotLine, minTimep1(plotLine),'--',color='green')
        plt.scatter(DoublePkg1,minTime1,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 2  
        height = 1
        
        ax = fig.add_subplot(5,1,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Duplicate Packages"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Duplicate Packages (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        DoublePkg2 = obj.dataList[doublePkg_num][height].reshape(1,-1)[0]
        maxTime2 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime2 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime2 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit2 = np.polyfit(DoublePkg2,maxTime2,polyDegree)
        maxTimep2 = np.poly1d(maxTimeFit2)
        plt.plot(plotLine, maxTimep2(plotLine),'--',color='red')
        plt.scatter(DoublePkg2,maxTime2,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit2 = np.polyfit(DoublePkg2,avgTime2,polyDegree)
        avgTimep2 = np.poly1d(avgTimeFit2)
        plt.plot(plotLine, avgTimep2(plotLine),'-',color='blue')
        plt.scatter(DoublePkg2,avgTime2,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit2 = np.polyfit(DoublePkg2,minTime2,polyDegree)
        minTimep2 = np.poly1d(minTimeFit2)
        plt.plot(plotLine, minTimep2(plotLine),'--',color='green')
        plt.scatter(DoublePkg2,minTime2,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 3    
        height = 2
        
        ax = fig.add_subplot(5,1,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Duplicate Packages"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Duplicate Packages (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        DoublePkg3 = obj.dataList[doublePkg_num][height].reshape(1,-1)[0]
        maxTime3 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime3 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime3 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit3 = np.polyfit(DoublePkg3,maxTime3,polyDegree)
        maxTimep3 = np.poly1d(maxTimeFit3)
        plt.plot(plotLine, maxTimep3(plotLine),'--',color='red')
        plt.scatter(DoublePkg3,maxTime3,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit3 = np.polyfit(DoublePkg3,avgTime3,polyDegree)
        avgTimep3 = np.poly1d(avgTimeFit3)
        plt.plot(plotLine, avgTimep3(plotLine),'-',color='blue')
        plt.scatter(DoublePkg3,avgTime3,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit3 = np.polyfit(DoublePkg3,minTime3,polyDegree)
        minTimep3 = np.poly1d(minTimeFit3)
        plt.plot(plotLine, minTimep3(plotLine),'--',color='green')
        plt.scatter(DoublePkg3,minTime3,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 4 
        height = 3
        
        ax = fig.add_subplot(5,1,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Duplicate Packages"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Duplicate Packages (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        DoublePkg4 = obj.dataList[doublePkg_num][height].reshape(1,-1)[0]
        maxTime4 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime4 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime4 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit4 = np.polyfit(DoublePkg4,maxTime4,polyDegree)
        maxTimep4 = np.poly1d(maxTimeFit4)
        plt.plot(plotLine, maxTimep4(plotLine),'--',color='red')
        plt.scatter(DoublePkg4,maxTime4,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit4 = np.polyfit(DoublePkg4,avgTime4,polyDegree)
        avgTimep4 = np.poly1d(avgTimeFit4)
        plt.plot(plotLine, avgTimep4(plotLine),'-',color='blue')
        plt.scatter(DoublePkg4,avgTime4,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit4 = np.polyfit(DoublePkg4,minTime4,polyDegree)
        minTimep4 = np.poly1d(minTimeFit4)
        plt.plot(plotLine, minTimep4(plotLine),'--',color='green')
        plt.scatter(DoublePkg4,minTime4,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
#    
#    # PLOT 5  -  All points of all layers fitted in one graph (one line)      
        ax = fig.add_subplot(5,1,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", "Round-Trip Time vs Duplicate Packages"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Duplicate Packages (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        DoublePkg5 = obj.dataList[doublePkg_num].reshape(1,-1)[0]
        maxTime5 = obj.dataList[maxTime_ms].reshape(1,-1)[0]
        avgTime5 = obj.dataList[avgTime_ms].reshape(1,-1)[0]
        minTime5 = obj.dataList[minTime_ms].reshape(1,-1)[0]
        
        maxTimeFit5 = np.polyfit(DoublePkg5,maxTime5,polyDegree)
        maxTimep5 = np.poly1d(maxTimeFit5)
        plt.plot(plotLine, maxTimep5(plotLine),'--',color='red')
        plt.scatter(DoublePkg5,maxTime5,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit5 = np.polyfit(DoublePkg5,avgTime5,polyDegree)
        avgTimep5 = np.poly1d(avgTimeFit5)
        plt.plot(plotLine, avgTimep5(plotLine),'-',color='blue')
        plt.scatter(DoublePkg5,avgTime5,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit5 = np.polyfit(DoublePkg5,minTime5,polyDegree)
        minTimep5 = np.poly1d(minTimeFit5)
        plt.plot(plotLine, minTimep5(plotLine),'--',color='green')
        plt.scatter(DoublePkg5,minTime5,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_Duplicate_Pkg_vs_Times.png".format(obj.droneName), dpi=100)  # results in 160x120 px image
                print("Figure: {}_Duplicate_Pkg_vs_Times exported as PNG file.".format(obj.droneName))
        
        plt.show()

    def timeOverPkgLossPlot(obj):
        '''This function fits the data linearly and the it generates the plots'''
        polyDegree = 1
        prt = True
        outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    
        fig=plt.figure(figsize=(10,20)) #80*len(anafi.dataList[sel])
        plotLine = np.linspace(obj.dataList[pkgLoss_num].min(),obj.dataList[pkgLoss_num].max(),121)
        
    # PLOT 1   
        height = 0
        PkgLoss1 = obj.dataList[pkgLoss_num][height].reshape(1,-1)[0]
        maxTime1 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime1 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime1 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
    
        ax = fig.add_subplot(5,1,1)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Package Loss"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Package Loss (#)")
        ax.set_ylabel("Round-Trip Time (ms)")
        
        
        maxTimeFit1 = np.polyfit(PkgLoss1,maxTime1,polyDegree)
        maxTimep1 = np.poly1d(maxTimeFit1)
        plt.plot(plotLine, maxTimep1(plotLine),'--',color='red')
        plt.scatter(PkgLoss1,maxTime1,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit1 = np.polyfit(PkgLoss1,avgTime1,polyDegree)
        avgTimep1 = np.poly1d(avgTimeFit1)
        plt.plot(plotLine, avgTimep1(plotLine),'-',color='blue')
        plt.scatter(PkgLoss1,avgTime1,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit1 = np.polyfit(PkgLoss1,minTime1,polyDegree)
        minTimep1 = np.poly1d(minTimeFit1)
        plt.plot(plotLine, minTimep1(plotLine),'--',color='green')
        plt.scatter(PkgLoss1,minTime1,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 2  
        height = 1
        
        ax = fig.add_subplot(5,1,2)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Package Loss"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Package Loss (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        PkgLoss2 = obj.dataList[pkgLoss_num][height].reshape(1,-1)[0]
        maxTime2 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime2 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime2 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit2 = np.polyfit(PkgLoss2,maxTime2,polyDegree)
        maxTimep2 = np.poly1d(maxTimeFit2)
        plt.plot(plotLine, maxTimep2(plotLine),'--',color='red')
        plt.scatter(PkgLoss2,maxTime2,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit2 = np.polyfit(PkgLoss2,avgTime2,polyDegree)
        avgTimep2 = np.poly1d(avgTimeFit2)
        plt.plot(plotLine, avgTimep2(plotLine),'-',color='blue')
        plt.scatter(PkgLoss2,avgTime2,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit2 = np.polyfit(PkgLoss2,minTime2,polyDegree)
        minTimep2 = np.poly1d(minTimeFit2)
        plt.plot(plotLine, minTimep2(plotLine),'--',color='green')
        plt.scatter(PkgLoss2,minTime2,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 3    
        height = 2
        
        ax = fig.add_subplot(5,1,3)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Package Loss"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Package Loss (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        PkgLoss3 = obj.dataList[pkgLoss_num][height].reshape(1,-1)[0]
#        PkgLoss3 = np.full_like(PkgLoss3x,fill_value=1,dtype=float) # On layer 3 there is 0 package loss, however 0 is not a valide value
        maxTime3 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime3 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime3 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
    
        if PkgLoss3.all() == 0:
            maxTimeFit3 = np.polyfit(PkgLoss3,maxTime3,0)
        else:
            maxTimeFit3 = np.polyfit(PkgLoss3,maxTime3,polyDegree)
        maxTimep3 = np.poly1d(maxTimeFit3)
        plt.plot(plotLine, maxTimep3(plotLine),'--',color='red')
        plt.scatter(PkgLoss3,maxTime3,alpha=0.2,color='red', label='Max Time')
 
        if PkgLoss3.all() == 0:
            avgTimeFit3 = np.polyfit(PkgLoss3,avgTime3,0)
        else:
            avgTimeFit3 = np.polyfit(PkgLoss3,avgTime3,polyDegree)
        avgTimep3 = np.poly1d(avgTimeFit3)
        plt.plot(plotLine, avgTimep3(plotLine),'-',color='blue')
        plt.scatter(PkgLoss3,avgTime3,alpha=0.2,color='blue', label='Average Time')
        
        if PkgLoss3.all() == 0:
            minTimeFit3 = np.polyfit(PkgLoss3,minTime3,0)
        else:
            minTimeFit3 = np.polyfit(PkgLoss3,minTime3,polyDegree)
        minTimep3 = np.poly1d(minTimeFit3)
        plt.plot(plotLine, minTimep3(plotLine),'--',color='green')
        plt.scatter(PkgLoss3,minTime3,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
    
#    # PLOT 4 
        height = 3
        
        ax = fig.add_subplot(5,1,4)
        ax.set_title(obj.droneName + " ({} m)\n{}".format(obj.layerHeights[height], "Round-Trip Time vs Package Loss"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Package Loss (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        PkgLoss4 = obj.dataList[pkgLoss_num][height].reshape(1,-1)[0]
        maxTime4 = obj.dataList[maxTime_ms][height].reshape(1,-1)[0]
        avgTime4 = obj.dataList[avgTime_ms][height].reshape(1,-1)[0]
        minTime4 = obj.dataList[minTime_ms][height].reshape(1,-1)[0]
        
        maxTimeFit4 = np.polyfit(PkgLoss4,maxTime4,polyDegree)
        maxTimep4 = np.poly1d(maxTimeFit4)
        plt.plot(plotLine, maxTimep4(plotLine),'--',color='red')
        plt.scatter(PkgLoss4,maxTime4,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit4 = np.polyfit(PkgLoss4,avgTime4,polyDegree)
        avgTimep4 = np.poly1d(avgTimeFit4)
        plt.plot(plotLine, avgTimep4(plotLine),'-',color='blue')
        plt.scatter(PkgLoss4,avgTime4,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit4 = np.polyfit(PkgLoss4,minTime4,polyDegree)
        minTimep4 = np.poly1d(minTimeFit4)
        plt.plot(plotLine, minTimep4(plotLine),'--',color='green')
        plt.scatter(PkgLoss4,minTime4,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
#    
#    # PLOT 5  -  All points of all layers fitted in one graph (one line)      
        ax = fig.add_subplot(5,1,5)
        ax.set_title(obj.droneName + " ({} m)\n{}".format("all", "Round-Trip Time vs Package Loss"))
        plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
        ax.set_xlabel("Package Loss (#)")
        ax.set_ylabel("Round-Trip Time (ms)")    
    
        PkgLoss5 = obj.dataList[pkgLoss_num].reshape(1,-1)[0]
        maxTime5 = obj.dataList[maxTime_ms].reshape(1,-1)[0]
        avgTime5 = obj.dataList[avgTime_ms].reshape(1,-1)[0]
        minTime5 = obj.dataList[minTime_ms].reshape(1,-1)[0]
        
        maxTimeFit5 = np.polyfit(PkgLoss5,maxTime5,polyDegree)
        maxTimep5 = np.poly1d(maxTimeFit5)
        plt.plot(plotLine, maxTimep5(plotLine),'--',color='red')
        plt.scatter(PkgLoss5,maxTime5,alpha=0.2,color='red', label='Max Time')
 
        avgTimeFit5 = np.polyfit(PkgLoss5,avgTime5,polyDegree)
        avgTimep5 = np.poly1d(avgTimeFit5)
        plt.plot(plotLine, avgTimep5(plotLine),'-',color='blue')
        plt.scatter(PkgLoss5,avgTime5,alpha=0.2,color='blue', label='Average Time')
        
        minTimeFit5 = np.polyfit(PkgLoss5,minTime5,polyDegree)
        minTimep5 = np.poly1d(minTimeFit5)
        plt.plot(plotLine, minTimep5(plotLine),'--',color='green')
        plt.scatter(PkgLoss5,minTime5,alpha=0.2,color='green', label='Min Time')
        
        plt.legend()
#        Customize the grid (major and minor)
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.2', color='black')
        ax.grid(which='minor', linestyle=':', linewidth='0.2', color='gray')
#    
        plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        if prt:
                fig.savefig(outputPath+"/{}_Pkg_Loss_vs_Times.png".format(obj.droneName), dpi=100)  # results in 160x120 px image
                print("Figure: {}_Pkg_Loss_vs_Times exported as PNG file.".format(obj.droneName))
        
        plt.show()

#    overDistancePlot(ardrone,bitrate_mbs)
#    overDistanceLogPlot(ardrone,avgTime_ms)
#    overRSSPlot(anafi,minTime_ms)
#    overBitratePlot(ardrone,avgTime_ms)
#    dbm(anafi)
#    FSPL(anafi)
#    timeOverDistancePlot(anafi)
#    timeOverDistancePlot(ardrone)
#    timeOverRSSPlot(anafi)
#    timeOverBitratePlot(ardrone)
#    timeOverDoublepkgPlot(ardrone)
    timeOverPkgLossPlot(ardrone)

#
#    for var in range(16):
#        overDistancePlot(anafi,var)
#        overDistanceLogPlot(anafi,var)
##        overDistancePlot(ardrone,var)
































if __name__ == "__main__":
#    main()
    newMain()