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
                'Recieved Signal Strenght (mW)', 'Noise Level (dBm)', 'Noise Level (mW)','Bitrate (Mb/s)')
    
#   Names of the output files
    outputNames = ('packet_tx','packet_rx','packet_loss_num','packet_loss_perc','min_time','avg_time','max_time',
                   'mean_time','double_packets_num','double_packets_perc','wifi_readings','RSSI','RSS_perc','RSS_dBm',
                   'RSS_mw','noise_dbm','noise_mw','bitrate')
    
#   Choose which variables should be plotted 
#            [pkgTx|pkgRx|PkgLossN|PkgLossP|MinTime|AvgTime|MaxTime|MdevTime|PkgDoubleN|PkgDoubleP|Readings|RSSi|RSSp|RSSdbm|RSSmw|noisedbm|noisemw|bitrate]
    useVar = [False,False,   False,    True,   True,   True,   True,    True,   False,     True,    False,  True,True,  True,False,   False,  False,True]
#    barLim = [(0,50),(0,50),(0,50),(0,100),(0,1000),(0,1000),(0,1000),(0,1000),(0,50),    (0,100), (0,10),(0,70),(0,100),(-100,0),(0,1000),(0,-255),(0,1000),(0,300)]
    indices = [i for i, val in enumerate(useVar) if val]

#   Define the measured heights 
    layerHeights = (0.5, 1.5, 2.5, 3.5)
    
    outputPath = ''
    
    def __init__(self, path, name):
#       Import data from external NPY files        
        self.dataList = np.load(path)
        self.droneName = name
        self.barLim = (0,100)
        print("Object "+name+" was created!")

    def plotTx(self):
        indx = 0
        self.barLim = (0,30)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotTx was executed!")
        
    def plotRx(self):
        indx = 1
        self.barLim = (0,30)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotΡx was executed!")
        
    def plotLoss_num(self):
        indx = 2
        self.barLim = (0,30)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotLoss_num was executed!")

    def plotLoss_perc(self):
        indx = 3
        self.barLim = (0,100)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotLoss_perc was executed!")
    
    def plotTime_min(self):
        indx = 4
        self.barLim = (0,1000)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotTime_min was executed!")
    
    def plotTime_avg(self):
        indx = 5
        self.barLim = (0,1000)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotTime_avg was executed!")
        
    def plotTime_max(self):
        indx = 6
        self.barLim = (0,1000)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotTime_max was executed!")
    
    def plotTime_mdev(self):
        indx = 7
        self.barLim = (0,1000)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotTime_mdev was executed!")
        
    def plotDouble_num(self):
        indx = 8
        self.barLim = (0,30)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotDouble_num was executed!")
        
    def plotDouble_perc(self):
        indx = 9
        self.barLim = (0,100)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotDouble_perc was executed!")
    
    def plotReadings(self):
        indx = 10
        self.barLim = (0,10)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotReadings was executed!")
        
    def plotRSSI(self):
        indx = 11
        self.barLim = (0,80)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotRSSI was executed!")
        
    def plotRSS_perc(self):
        indx = 12
        self.barLim = (0,100)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotRSS_perc was executed!")
    
    def plotRSS_dbm(self):
        indx = 13
        self.barLim = (-100,0)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotRSS_dbm was executed!")
        
    def plotRSS_mw(self):
        indx = 14
        self.barLim = (0,1000)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotRSS_mw was executed!")
        
    def plotNoise_dbm(self):
        indx = 15
        self.barLim = (-250,0)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotNoise_dbm was executed!")
        
    def plotNoise_mw(self):
        indx = 16
        self.barLim = (0,1000)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotNoise_mw was executed!")
        
    def plotBitrate(self):
        indx = 17
        self.barLim = (0,300)
        for i,height in enumerate(self.layerHeights):
            self.varPlot(self.dataList[indx,i],height, self.varTitles[indx], self.outputNames[indx])
        print("plotBitrate was executed!")
        
    def plotAll(self):
        test.plotTx()
        test.plotRx()
        test.plotDouble_num()
        test.plotDouble_perc()
        test.plotLoss_num()
        test.plotLoss_perc()
        test.plotNoise_dbm()
        test.plotNoise_mw()
        test.plotReadings()
        test.plotRSS_dbm()
        test.plotRSS_mw()
        test.plotRSS_perc()
        test.plotRSSI()
        test.plotTime_avg()
        test.plotTime_max()
        test.plotTime_mdev()
        test.plotTime_min()
        test.plotBitrate()
        print("plotAll was executed!")
    
    def varPlot(self,imputVar, height, title, outputName):
        fig= plt.figure(figsize=(10,10))
        ax = fig.add_subplot(1,1,1)
        plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_xlim(-1,11)
        ax.set_ylim(11,-1)
        
        ax.set_title(self.droneName + " ({}m)\n{}".format(height, title))
        img = mpimg.imread("/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/test_layout.png")
        imgplot = ax.imshow(imputVar, cmap="inferno",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
        
        imgplot.set_clim(self.barLim)
        fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.75,pad=0.08)
        for i in range(11):
            for j in range(11):
                ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                ax.text(j, i, round(imputVar[i][j],2), ha="center", va="center", color="w")
#        self.savePng(fig,self.outputPath,outputName)
        print("varPlot, was exectuted!")
        plt.show()
    
    def savePng(self,figure,outputPath,outputName):
        '''Save a figure to a PNG file on the provided path'''
        figure.savefig(outputPath+"/{}.png".format(outputName), dpi=100)  # results in 160x120 px image
    
ardrone_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(full)_(ARDRONE_FIXED).npy'
test = dataVisualiser(ardrone_path, "AR.Drone 2.0")
test.plotAll()
#test.plotTx()
#test.plotRx()
#test.plotDouble_num()
#test.plotDouble_perc()
#test.plotLoss_num()
#test.plotLoss_perc()
#test.plotNoise_dbm()
#test.plotNoise_mw()
#test.plotReadings()
#test.plotRSS_dbm()
#test.plotRSS_mw()
#test.plotRSS_perc()
#test.plotRSSI()
#test.plotTime_avg()
#test.plotTime_max()
#test.plotTime_mdev()
#test.plotTime_min()
#test.plotBitrate()

#    
#    # Difference function
#    def difViz():
#    # Calculate the difference of the values of the two drones
#        difDataList = []
#        for ar,an in zip(arDataList,anDataList):
#            difDataList.append((abs(ar[0]-an[0]),abs(ar[1]-an[1]),abs(ar[2]-an[2]),abs(ar[3]-an[3])))
#        pass
#    
#    
#    for r in indices:
#        oneVarVisual(titles[r], arDataList[r], anDataList[r], difDataList[r], outputNames[r])
