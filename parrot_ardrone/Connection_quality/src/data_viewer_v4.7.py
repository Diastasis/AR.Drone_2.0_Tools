import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
np.set_printoptions(threshold=sys.maxsize)
#np.set_printoptions(threshold=np.inf)
#======================================================



class heatmapVisualiser():
    '''This class generates heatmaps for every single variable in different layers. It is also possible to compare two class instances and plot heatmaps from their difference.'''
# TODO:
# 1) Add an argument on te plot function to print only one layer
# 2) Fix the scale of the color bars on all variables
# 3) Create and import the greenhouse layout
# 4) Create a statistics table for each case
# 5) Add units in the colorbar label
    
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
    

    outputPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/plots'
    debug = False
    
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
    layerHeights = (0.5, 1.5, 2.5, 3.5) # Assuming that we have data from four layers
    measurementPoint = (10, 10.5, 1)    # x,y,z in meters (laptop position)
    
    
    
    def __init__(self, data, name, comp=False):
        self.dataList = data
        self.droneName = name
        self.distances = self.euclideanDist()
        self.objComp = comp
        self.barLim = (0,100)
        if comp:
            self.colormap = "jet"
        else:
            self.colormap="inferno"
        self.groupPlot = False
        if self.debug:
            print("Object "+name+" was created!")
            
    @classmethod
    def form_Path(cls, path, name):
        '''Create an object by importing data from an external NPY file'''
        data = np.load(path)
        return cls(data,name)
    
    @classmethod
    def from_Substruction(cls, obj1, obj2, name):
        '''Create a new object by substructing the data values of two other objects.'''
        data = obj1.dataList - obj2.dataList
        return cls(data,name,comp=True)
    
    def euclideanDist(self):
        '''This function returns the euclidean distances from the measurement 
        point (laptop position during the experiments). The output is an array with dimentions (4x11x11).'''
        
        # Create an array with the same dimentions as the originala data filled with the measurementPoint data.
        POI = np.full((4,11,11,3),self.measurementPoint)
#        distances = np.full((4,11,11),fill_value = 0.0)
        xx = np.linspace(0,10,11)   # Creae x axis array
        yy = np.linspace(0,10,11)   # Creae y axis array
        zz = np.linspace(0.5,3.5,4) # Creae z axis array
        
#        Create and initialize an array with all the x,y,z values of every point
        pointGrid = np.full((4,11,11,3),fill_value = 0.0)    
        for i,z in enumerate(zz):
            for ii,y in enumerate(yy):
                for iii,x in enumerate(xx):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
            self.varGroupHeatmap(self.dataList[indx],self.layerHeights,self.varTitles[indx],self.outputNames[indx])
        else:
            for i,height in enumerate(self.layerHeights):
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
        fig=plt.figure(figsize=(8*len(inputVar),10))
        for ii,height in enumerate(heightList):
            ax = fig.add_subplot(1,len(inputVar),ii+1)
            ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
            plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")
            ax.set_xlim(-1,11)
            ax.set_ylim(11,-1)
#            img = plt.imread('/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/Greenhouse_Layout.png')
            imgplot = ax.imshow(inputVar[ii], cmap=self.colormap,interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
            plt.tight_layout() # This solves the issue of the missing titles in the exported png file
            imgplot.set_clim(self.barLim)        
            mybar = fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.95,pad=0.08)#.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
            if self.objComp:
                mybar.set_label("<  AR.Drone 2.0                                                      Anafi  >", labelpad=-55)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(inputVar[ii][i][j],2), ha="center", va="center", color="w")
            ax.scatter(self.measurementPoint[0],self.measurementPoint[1], alpha=0.5, s=100, color="b", marker="X")
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
        ax.set_xlim(-1,11)
        ax.set_ylim(11,-1)
#        img = mpimg.imread('/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/Greenhouse_Layout.png')
        imgplot = ax.imshow(inputVar, cmap=self.colormap,interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
        imgplot.set_clim(self.barLim)
        mybar = fig.colorbar(imgplot, ax=ax, orientation='horizontal',shrink=0.75,pad=0.08)
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
            print("varHeatmap was exectuted!")
    
    def savePng(self,figure,outputPath,outputName,height):
        '''Save a figure to a PNG file on the provided path.'''
        figure.savefig(outputPath+"/{}_{}_({}m).png".format(self.droneName,outputName,height), dpi=100)  # results in 160x120 px image
        print("Figure: {}_{}_({}m) exported as PNG file.".format(self.droneName,outputName,height))
        if self.debug:
            print("savePng was exectuted!")


#===============================================================================
            

            
##    Packets
    def plotDist_vs_Tx(self,other,groupPlot=True):
        indx = 0
        if groupPlot:
            dists = self.distances.reshape(1,-1)
            print("dists:\n",dists)
            print("dists.shape",dists.shape)
            
            
            arRSSI = obj1.dataList[indx][3].reshape(1,-1)
            anRSSI = obj2.dataList[indx][3].reshape(1,-1)
        else:
            raise Exception("Invalid layer number!")
        
        fig=plt.figure(figsize=(8*len(dists),10))
        
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
#            print("mBAR",mybar.get_clim())
            if self.objComp:
                mybar.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(inputVar[ii][i][j],2), ha="center", va="center", color="w")
            ax.scatter(self.measurementPoint[0],self.measurementPoint[1], alpha=0.5, s=100, color="b", marker="X")
            plt.tight_layout() # This solves the issue of the missing titles in the exported png file
        plt.show()
        
        
        
        
        xp = np.linspace(dists.min(),dists.max(),100)
        
        arPoly = np.polyfit(dists,arRSSI,1)
        arp = np.poly1d(arPoly)
        plt.plot(dists,arRSSI,'.',xp,arp(xp),'-')
    
        anPoly = np.polyfit(dists,anRSSI,1)
        anp = np.poly1d(anPoly)
        plt.plot(dists,anRSSI,'.', xp,anp(xp),'-')
        
        plt.show()







def  main():
    ardrone_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(full)_(ARDRONE_FIXED_FINAL).npy'
    ardrone = heatmapVisualiser.form_Path(ardrone_path, "AR.Drone 2.0")
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
    
    anafi_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_150126_(full)_(ANAFI_FIXED_FINAL).npy'
    anafi = heatmapVisualiser.form_Path(anafi_path, "Anafi")
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
    
    anafi_vs_Ardrone = heatmapVisualiser.from_Substruction(anafi,ardrone,"Anafi vs AR.Drone 2.0")
#    anafi_vs_Ardrone.heatmapAll(groupPlot=True)
#    anafi_vs_Ardrone.heatmapTx(groupPlot=True)
#    anafi_vs_Ardrone.heatmapRx(groupPlot=True)
#    anafi_vs_Ardrone.heatmapLoss_num(groupPlot=True)
#    anafi_vs_Ardrone.heatmapLoss_perc(groupPlot=True)
#    anafi_vs_Ardrone.heatmapTime_min(groupPlot=True)
#    anafi_vs_Ardrone.heatmapTime_avg(groupPlot=True)        
#    anafi_vs_Ardrone.heatmapTime_max(groupPlot=True)
#    anafi_vs_Ardrone.heatmapTime_mdev(groupPlot=True)
#    anafi_vs_Ardrone.heatmapDouble_num(groupPlot=True)
#    anafi_vs_Ardrone.heatmapDouble_perc(groupPlot=True)
#    anafi_vs_Ardrone.heatmapReadings(groupPlot=True)
#    anafi_vs_Ardrone.heatmapRSSI(groupPlot=True)
#    anafi_vs_Ardrone.heatmapRSS_perc(groupPlot=True)
#    anafi_vs_Ardrone.heatmapRSS_dbm(groupPlot=True)
#    anafi_vs_Ardrone.heatmapNoise_dbm(groupPlot=True)
#    anafi_vs_Ardrone.heatmapBitrate(groupPlot=True)

#    def overDistancePlot(obj,var):
#        pass

#    GroupPlot
    sel = 1
    polyDegree = 2


    fig=plt.figure(figsize=(40,60)) #80*len(anafi.dataList[sel])
# PLOT 1    
    dists1 = anafi.distances[0].reshape(1,-1)[0]
    var1 = anafi.dataList[sel][0].reshape(1,-1)[0]
    xp1 = np.linspace(dists1.min(),dists1.max(),100)

    ax = fig.add_subplot(4,2,1)
    ax.set_title(anafi.droneName + " ({}m)\n{}".format(anafi.layerHeights[0], "var_title"))
    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
    ax.set_xlabel("Distance form PC (meters)")
    ax.set_ylabel("Variable of interest (units)")
    
    anPoly1 = np.polyfit(dists1,var1,polyDegree)
    anp1 = np.poly1d(anPoly1)
    plt.plot(dists1,var1,'.', xp1,anp1(xp1),'-')

# PLOT 2    
    dists2 = anafi.distances[1].reshape(1,-1)[0]
    var2 = anafi.dataList[sel][1].reshape(1,-1)[0]
    xp2 = np.linspace(dists2.min(),dists2.max(),100)

    ax = fig.add_subplot(4,2,2)
    ax.set_title(anafi.droneName + " ({}m)\n{}".format(anafi.layerHeights[1], "var_title"))
    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
    ax.set_xlabel("Distance form PC (meters)")
    ax.set_ylabel("Variable of interest (units)")
    
    anPoly2 = np.polyfit(dists2,var2,polyDegree)
    anp2 = np.poly1d(anPoly2)
    plt.plot(dists2,var2,'.', xp2,anp2(xp2),'-')

# PLOT 3    
    dists3 = anafi.distances[2].reshape(1,-1)[0]
    var3 = anafi.dataList[sel][2].reshape(1,-1)[0]
    xp3 = np.linspace(dists3.min(),dists3.max(),100)

    ax = fig.add_subplot(4,2,3)
    ax.set_title(anafi.droneName + " ({}m)\n{}".format(anafi.layerHeights[2], "var_title"))
    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
    ax.set_xlabel("Distance form PC (meters)")
    ax.set_ylabel("Variable of interest (units)")
    
    anPoly3 = np.polyfit(dists3,var3,polyDegree)
    anp3 = np.poly1d(anPoly3)
    plt.plot(dists3,var3,'.', xp3,anp3(xp3),'-')

# PLOT 4 
    dists4 = anafi.distances[3].reshape(1,-1)[0]
    var4 = anafi.dataList[sel][3].reshape(1,-1)[0]
    xp4 = np.linspace(dists4.min(),dists4.max(),100)

    ax = fig.add_subplot(4,2,4)
    ax.set_title(anafi.droneName + " ({}m)\n{}".format(anafi.layerHeights[3], "var_title"))
    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
    ax.set_xlabel("Distance form PC (meters)")
    ax.set_ylabel("Variable of interest (units)")
    
    anPoly4 = np.polyfit(dists4,var4,polyDegree)
    anp4 = np.poly1d(anPoly4)
    plt.plot(dists4,var4,'.', xp4,anp4(xp4),'-')


# PLOT 5    All points in one graph
    dists5 = anafi.distances.reshape(1,-1)[0]
    var5 = anafi.dataList[sel].reshape(1,-1)[0]
    xp5 = np.linspace(dists5.min(),dists5.max(),100)
    
    ax = fig.add_subplot(4,2,5)
    ax.set_title(anafi.droneName + " ({}m)\n{}".format("all", "var_title"))
#    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
    ax.set_xlabel("Distance form PC (meters)")
    ax.set_ylabel("Variable of interest (units)")  
       
    anPoly5 = np.polyfit(dists5,var5,2)
    anp5 = np.poly1d(anPoly5)
    plt.plot(dists5,var5,'.', xp5,anp5(xp5),'-')


# PLOT 6 - All layers and fits in one figure

    ax = fig.add_subplot(4,2,6)
    ax.set_title(anafi.droneName + " ({}m)\n{}".format("All", "var_title"))
    
    plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
    ax.set_xlabel("Distance form PC (meters)")
    ax.set_ylabel("Variable of interest (units)")
    
    plt.plot(dists1,var1,'.', xp1,anp1(xp1),'-', color='cyan', label='0.5m')
    plt.plot(dists2,var2,'.', xp2,anp2(xp2),'-', color='goldenrod', label='1.5m')
    plt.plot(dists3,var3,'.', xp3,anp3(xp3),'-', color='magenta', label='2.5m')
    plt.plot(dists4,var4,'.', xp4,anp4(xp4),'-', color='teal', label='3.5m')
    plt.plot(xp5,anp5(xp5),'-', color='r', label='All Layers')
    plt.legend()




    plt.show()





















##   DISTANCE VS :

    
#    def plotDist_vs_Rx(self, layer="all"):
#        pass
#    
#    def plotDist_vs_PkgLoss_num(self, layer="all"):
#        pass
#    
#    def plotDist_vs_PkgLoss_perc(self, layer="all"):
#        pass
#
#    def plotDist_vs_double_num(self, layer="all"):
#        pass
#    
#    def plotDist_vs_double_perc(self, layer="all"):
#        pass
#
#
##    Time
#    def plotDist_vs_Min(self, layer="all"):
#        pass
#
#    def plotDist_vs_Avg(self, layer="all"):
#        pass
#    
#    def plotDist_vs_Max(self, layer="all"):
#        pass
#    
#    def plotDist_vs_Mdev(self, layer="all"):
#        pass
#
#
##   Signal
#    def plotDist_vs_Readings(self, layer="all"):
#        pass
#    
#    def plotDist_vs_RSSI(self, layer="all"):
#        pass
#    
#    def plotDist_vs_RSS_perc(self, layer="all"):
#        pass
#    
#    def plotDist_vs_RSS_dbm(self, layer="all"):
#        pass
#
#    def plotDist_vs_Noise_dbm(self, layer="all"):
#        pass
#
#
##   Bitrate
#    def plotDist_vs_Bitrate(self, layer="all"):
#        pass


    
#    plotDist_vs_Tx()


#    dists = ardrone.distances.reshape(1,-1)
#    arRSSI = ardrone.dataList[12].reshape(1,-1)
#    anRSSI = anafi.dataList[12].reshape(1,-1)
    


#    xp = np.linspace(dists[0].min(),dists[0].max(),100)
#    
#    arPoly = np.polyfit(dists[0],arRSSI[0],3)
#    arp = np.poly1d(arPoly)
#    plt.plot(dists[0],arRSSI[0],'.',xp,arp(xp),'-')
#
#    anPoly = np.polyfit(dists[0],anRSSI[0],3)
#    anp = np.poly1d(anPoly)
#    plt.plot(dists[0],anRSSI[0],'.', xp,anp(xp),'-')
#    
#    plt.show()

#    print("\n",len("AR.Drone 2.0                                                                                             Anafi"))
    




    















if __name__ == "__main__":
    main()