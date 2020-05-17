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
#            
#
#            
###    Packets
#    def plotDist_vs_Tx(self,other,groupPlot=True):
#        indx = 0
#        if groupPlot:
#            dists = self.distances.reshape(1,-1)
#            print("dists:\n",dists)
#            print("dists.shape",dists.shape)
#            
#            
#            arRSSI = obj1.dataList[indx][3].reshape(1,-1)
#            anRSSI = obj2.dataList[indx][3].reshape(1,-1)
#        else:
#            raise Exception("Invalid layer number!")
#        
#        fig=plt.figure(figsize=(8*len(dists),10))
#        
#        for ii,height in enumerate(heightList):
#            ax = fig.add_subplot(1,len(inputVar),ii+1)
#            ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
##        plt.suptitle(self.droneName + " ({}m)\n{}".format(height, title))
#            plt.subplots_adjust(left=0.0, right=0.5, bottom=0.5, top=1.0)
#        
#            ax.set_xlabel("X (meters)")
#            ax.set_ylabel("Y (meters)")
#            ax.set_xlim(-1,11)
#            ax.set_ylim(11,-1)
#        
##        ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
##            img = plt.imread('/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/Greenhouse_Layout.png')
#            imgplot = ax.imshow(inputVar[ii], cmap=self.colormap,interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
##            imgplot.set_zorder(1)
##            plt.imshow(img, interpolation='none', alpha=1, zorder=2)
#
#            imgplot.set_clim(self.barLim)        
#            mybar = fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.95,pad=0.08)#.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
##            print("mBAR",mybar.get_clim())
#            if self.objComp:
#                mybar.set_label("AR.Drone 2.0                                                                                             Anafi", labelpad=-50)
#            for i in range(11):
#                for j in range(11):
#                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
#                    ax.text(j, i, round(inputVar[ii][i][j],2), ha="center", va="center", color="w")
#            ax.scatter(self.measurementPoint[0],self.measurementPoint[1], alpha=0.5, s=100, color="b", marker="X")
#            plt.tight_layout() # This solves the issue of the missing titles in the exported png file
#        plt.show()
#        
#        
#        
#        
#        xp = np.linspace(dists.min(),dists.max(),100)
#        
#        arPoly = np.polyfit(dists,arRSSI,1)
#        arp = np.poly1d(arPoly)
#        plt.plot(dists,arRSSI,'.',xp,arp(xp),'-')
#    
#        anPoly = np.polyfit(dists,anRSSI,1)
#        anp = np.poly1d(anPoly)
#        plt.plot(dists,anRSSI,'.', xp,anp(xp),'-')
#        
#        plt.show()







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

##### GENERATE HEAT-MAPS FOR ANAFI #####
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
    main()