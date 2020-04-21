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
# 4) Take measurements for 2.5m for both drones
# 5) Create a statistics table for each case


# Choose which variables should be plotted 
#        [pkgTx|pkgRx|PkgLossN|PkgLossP|MinTime|AvgTime|MaxTime|MdevTime|PkgBoubleN|PkgDoubleP|Readings|RSSi|RSSp|RSSdbm|RSSmw|noisedbm|noisemw|bitrate]
useVar = [False,False,   False,    True,   True,   True,   True,    True,     False,      True,   False,True,True,  True,False,   False,  False,True]


# AR.Drone: Import data from external NPY files
ardronePath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_300320_152111_(full)_(ARDRONE_FIXED).npy'
arDataList = np.load(ardronePath)

# Anafi: Import data from external NPY files
anafiPath = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_310320_150126_(full)_(ANAFI_FIXED).npy'
anDataList = np.load(anafiPath)


# Calculate the difference of the values of the two drones
difDataList = []
for ar,an in zip(arDataList,anDataList):
    ar0,ar1,ar2,ar3 = ar
    an0,an1,an2,an3 = an
    difDataList.append((abs(ar0-an0),abs(ar1-an1),abs(ar2-an2),abs(ar3-an3)))

# Titles used in plots
titles = ('Packets Transmitted (#)', 'Packets Received (#)', 'Packet Loss (#)','Packet Loss (%)',
            'Minimum Time (ms)','Avarage Time (ms)','Maximun Time (ms)', 'Mean Deviation (ms)',
            'Duplicate Packets (#)', 'Duplicate Packets (%)', 'Wifi Readings (#/Point)',
            'Recieved Signal Strenght Index (#/70)', 'Recieved Signal Strenght (%)','Recieved Signal Strenght (dBm)',
            'Recieved Signal Strenght (mW)', 'Noise Level (dBm)', 'Noise Level (mW)','Bitrate (Mb/s)')

# Names of the output files
outputNames = ('packet_tx','packet_rx','packet_loss_num','packet_loss_perc','min_time','avg_time','max_time',
               'mean_time','double_packets_num','double_packets_perc','wifi_readings','RSSI','RSS_perc','RSS_dBm',
               'RSS_mw','noise_dbm','noise_mw','bitrate')
    

def oneVarVisual(title, arVar, anVar, difVar, outputName):

    layerHeight = [0.5, 1.5, 2.5, 3.5]
    fig= plt.figure(figsize=(30,30))
    for ii in range(12):
        ax = fig.add_subplot(3,4,ii+1)
    #    plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_xlim(-1,11)
        ax.set_ylim(11,-1)
        
        if ii+1 < 5:
            ax.set_title("AR.Drone 2.0 ({}m)\n{}".format(layerHeight[ii], title))
            img = mpimg.imread("/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/test_layout.png")
    #        print(img.shape)
    #        img_x,img_x, _ = img.shape 
            
    #        bin_size = 356 // 200
    #        small_image = img.reshape((1, 200, bin_size, 200, bin_size)).max(4).max(2)
    #        ax.imshow(small_image,zorder=1, alpha=0.2)
            imgplot = ax.imshow(arVar[ii], cmap="inferno",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
    
            imgplot.set_clim(0, 100)
            fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.75,pad=0.08)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(arVar[ii][i][j],2), ha="center", va="center", color="w")
    #                ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
        elif ii+1 < 9:
            ax.set_title("Anafi ({}m)\n{}".format(layerHeight[ii-4], title))
            imgplot = ax.imshow(anVar[ii-4], cmap="inferno",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
            imgplot.set_clim(0, 100)
            fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.76,pad=0.08)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(anVar[ii-4][i][j],2), ha="center", va="center", color="w")
    #                ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
        else:
            ax.set_title("Difference ({}m)\n{}".format(layerHeight[ii-8], title))
            imgplot = ax.imshow(difVar[ii-8], cmap="jet",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
    #        imgplot.set_clim(60, 100)
            fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.76,pad=0.08)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(difVar[ii-8][i][j],2), ha="center", va="center", color="w")
    #                ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
    fig.savefig("{}.png".format(outputName), dpi=100)  # results in 160x120 px image
    plt.show() 


indices = [v for v, val in enumerate(useVar) if val]
for r in indices:
    oneVarVisual(titles[r], arDataList[r], anDataList[r], difDataList[r], outputNames[r])
