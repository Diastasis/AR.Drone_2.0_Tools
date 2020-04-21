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
# 1) Some variables contain values with more than 2 decimal points
# 2) Fix the scale of the color bars on some variables
# -- AVGTime ar = 0 - 125 ms | an = 0 - 0.05 ms | dif = 0 - 125 ms
# 3) Create and import the greenhouse layout
# 4) Take measurements for 2.5m for both drones



# AR.Drone: Import data from external NPY files
ardronePath_1_2 = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_160320_115629_(1.5m)_(AR).npy"
ardroneLayers_1_2 = np.load(ardronePath_1_2)
ardronePath_3 = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_170320_121610_(3.5m)_(AR).npy"
ardroneLayers_3 = np.load(ardronePath_3)

arPkgTx             = (ardroneLayers_1_2[0, 0], ardroneLayers_1_2[0, 1], ardroneLayers_3[0, 0]) # OK
arPkgRx             = (ardroneLayers_1_2[1, 0], ardroneLayers_1_2[1, 1], ardroneLayers_3[1, 0]) # OK
arPkgLoss_num       = (ardroneLayers_1_2[2, 0], ardroneLayers_1_2[2, 1], ardroneLayers_3[2, 0]) # OK
arPkgLoss_perc      = (ardroneLayers_1_2[3, 0], ardroneLayers_1_2[3, 1], ardroneLayers_3[3, 0]) # OK
arTimeMin           = (ardroneLayers_1_2[4, 0], ardroneLayers_1_2[4, 1], ardroneLayers_3[4, 0]) # OK
arTimeAvg           = (ardroneLayers_1_2[5, 0], ardroneLayers_1_2[5, 1], ardroneLayers_3[5, 0]) # OK
arTimeMax           = (ardroneLayers_1_2[6, 0], ardroneLayers_1_2[6, 1], ardroneLayers_3[6, 0]) # OK
arTimeDiv           = (ardroneLayers_1_2[7, 0], ardroneLayers_1_2[7, 1], ardroneLayers_3[7, 0]) # OK
arPkgDouble_num     = (ardroneLayers_1_2[8, 0], ardroneLayers_1_2[8, 1], ardroneLayers_3[8, 0]) # OK
arPkgDouble_perc    = (ardroneLayers_1_2[9, 0], ardroneLayers_1_2[9, 1], ardroneLayers_3[9, 0]) # OK
arWifiReadings      = (ardroneLayers_1_2[10,0], ardroneLayers_1_2[10,1], ardroneLayers_3[10,0]) # OK
arWifiQuality_num   = (ardroneLayers_1_2[11,0], ardroneLayers_1_2[11,1], ardroneLayers_3[11,0]) # OK
arWifiQuality_perc  = (ardroneLayers_1_2[12,0], ardroneLayers_1_2[12,1], ardroneLayers_3[12,0]) # OK
arWifiSig_dbm       = (ardroneLayers_1_2[13,0], ardroneLayers_1_2[13,1], ardroneLayers_3[13,0]) # OK
arWifiSig_mw        = (ardroneLayers_1_2[14,0], ardroneLayers_1_2[14,1], ardroneLayers_3[14,0]) # OK
arWifiNoise_dbm     = (ardroneLayers_1_2[15,0], ardroneLayers_1_2[15,1], ardroneLayers_3[15,0]) # OK
arWifiNoise_mw      = (ardroneLayers_1_2[16,0], ardroneLayers_1_2[16,1], ardroneLayers_3[16,0]) # OK
arWifiBitrate       = (ardroneLayers_1_2[17,0], ardroneLayers_1_2[17,1], ardroneLayers_3[17,0]) # OK

arDataList = [arPkgTx, arPkgRx,arPkgLoss_num, arPkgLoss_perc, arTimeMin, arTimeAvg, arTimeMax, 
              arTimeDiv, arPkgDouble_num, arPkgDouble_perc, arWifiReadings, arWifiQuality_num, 
              arWifiQuality_perc, arWifiSig_dbm, arWifiSig_mw, arWifiNoise_dbm, arWifiNoise_mw, arWifiBitrate]

# Anafi: Import data from external NPY files
anafiPath_3 = "/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_180320_120326_(3.5m)_(ANAFI).npy"
anafiLayers = np.load(anafiPath_3)

anPkgTx             = (anafiLayers[0, 0], anafiLayers[0, 1], anafiLayers[0, 2]) # OK
anPkgRx             = (anafiLayers[1, 0], anafiLayers[1, 1], anafiLayers[1, 2]) # OK
anPkgLoss_num       = (anafiLayers[2, 0], anafiLayers[2, 1], anafiLayers[2, 2]) # OK
anPkgLoss_perc      = (anafiLayers[3, 0], anafiLayers[3, 1], anafiLayers[3, 2]) # OK
anTimeMin           = (anafiLayers[4, 0], anafiLayers[4, 1], anafiLayers[4, 2]) # OK
anTimeAvg           = (anafiLayers[5, 0], anafiLayers[5, 1], anafiLayers[5, 2]) # OK
anTimeMax           = (anafiLayers[6, 0], anafiLayers[6, 1], anafiLayers[6, 2]) # OK
anTimeDiv           = (anafiLayers[7, 0], anafiLayers[7, 1], anafiLayers[7, 2]) # OK
anPkgDouble_num     = (anafiLayers[8, 0], anafiLayers[8, 1], anafiLayers[8, 2]) # OK
anPkgDouble_perc    = (anafiLayers[9, 0], anafiLayers[9, 1], anafiLayers[9, 2]) # OK
anWifiReadings      = (anafiLayers[10,0], anafiLayers[10,1], anafiLayers[10,2]) # OK
anWifiQuality_num   = (anafiLayers[11,0], anafiLayers[11,1], anafiLayers[11,2]) # OK
anWifiQuality_perc  = (anafiLayers[12,0], anafiLayers[12,1], anafiLayers[12,2]) # OK
anWifiSig_dbm       = (anafiLayers[13,0], anafiLayers[13,1], anafiLayers[13,2]) # OK
anWifiSig_mw        = (anafiLayers[14,0], anafiLayers[14,1], anafiLayers[14,2]) # OK
anWifiNoise_dbm     = (anafiLayers[15,0], anafiLayers[15,1], anafiLayers[15,2]) # OK
anWifiNoise_mw      = (anafiLayers[16,0], anafiLayers[16,1], anafiLayers[16,2]) # OK
anWifiBitrate       = (anafiLayers[17,0], anafiLayers[17,1], anafiLayers[17,2]) # OK

anDataList = [anPkgTx, anPkgRx,anPkgLoss_num, anPkgLoss_perc, anTimeMin, anTimeAvg, anTimeMax, 
              anTimeDiv, anPkgDouble_num, anPkgDouble_perc, anWifiReadings, anWifiQuality_num, 
              anWifiQuality_perc, anWifiSig_dbm, anWifiSig_mw, anWifiNoise_dbm, anWifiNoise_mw, anWifiBitrate]

# Calculate the difference of the values of the two drones
difDataList = []
for ar,an in zip(arDataList,anDataList):
    ar1,ar2,ar3 = ar
    an1,an2,an3 = an
    difDataList.append((abs(ar1-an1),abs(ar2-an2),abs(ar3-an3)))


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


#for t,title in enumerate(titles):
#    arLayer1, arLayer2, arLayer3 = arDataList[t]
#    anLayer1, anLayer2, anLayer3 = anDataList[t]
#    difLayer1 = arLayer1 - anLayer1 
#    difLayer2 = arLayer2 - anLayer2
#    difLayer3 = arLayer3 - anLayer3
    
    
def oneVarVisual(title, arVar, anVar, difVar, outputName):

    layerHeight = [0.5, 1.5, 3.5]
    fig= plt.figure(figsize=(30,30))
    for ii in range(9):
        ax = fig.add_subplot(3,3,ii+1)
    #    plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_xlim(-1,11)
        ax.set_ylim(11,-1)
        
        if ii+1 < 4:
            ax.set_title("AR.Drone 2.0 ({}m)\n{}".format(layerHeight[ii], title))
            img = mpimg.imread("/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/test_layout.png")
    #        print(img.shape)
    #        img_x,img_x, _ = img.shape 
            
    #        bin_size = 356 // 200
    #        small_image = img.reshape((1, 200, bin_size, 200, bin_size)).max(4).max(2)
    #        ax.imshow(small_image,zorder=1, alpha=0.2)
            imgplot = ax.imshow(arVar[ii], cmap="inferno",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
    
    #        imgplot.set_clim(60, 100)
            fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.75,pad=0.08)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(arVar[ii][i][j],2), ha="center", va="center", color="w")
    #                ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
        elif ii < 6:
            ax.set_title("Anafi ({}m)\n{}".format(layerHeight[ii-3], title))
            imgplot = ax.imshow(anVar[ii-4], cmap="inferno",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
    #        imgplot.set_clim(60, 100)
            fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.76,pad=0.08)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(anVar[ii-4][i][j],2), ha="center", va="center", color="w")
    #                ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
        else:
            ax.set_title("Difference ({}m)\n{}".format(layerHeight[ii-6], title))
            imgplot = ax.imshow(difVar[ii-6], cmap="jet",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
    #        imgplot.set_clim(60, 100)
            fig.colorbar(imgplot, ax=ax,orientation='horizontal',shrink=0.76,pad=0.08)
            for i in range(11):
                for j in range(11):
                    ax.scatter(i,j, alpha=0.1, s=800, color="w") # cmap='jet'
                    ax.text(j, i, round(difVar[ii-6][i][j],2), ha="center", va="center", color="w")
    #                ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
    fig.savefig("{}.png".format(outputName), dpi=100)  # results in 160x120 px image
    plt.show() 

for v in range(18):
    oneVarVisual(titles[v], arDataList[v], anDataList[v], difDataList[v], outputNames[v])
#
#index = 3            
#print(ardroneLayers.shape)
#for n,layer2 in enumerate(ardroneLayers[13]):
#    ax = fig.add_subplot(2,3,index)
#    ax.set_title("AR.Drone (Layer {})\n{}".format(n, titles[13]))
#    ax.set_xlabel("X (meters)")
#    ax.set_ylabel("Y (meters)")
#    ax.set_xlim(-1,11)
#    ax.set_ylim(11,-1)
#    index += 1
#    ax.imshow(layer2, cmap="inferno",interpolation='gaussian') # interpolation='bicubic', origin='lower', interpolation='gaussian'
#    for i in range(11):
#        for j in range(11):
#            ax.scatter(i,j, alpha=0.1, s=1200, color="w") # cmap='jet'
#            ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
##           ax.text(j, i, anafiLayers[13,2,i, j], ha="center", va="center", color="w")
#            

#bx = fig.add_subplot(1,2,2)
#bx.set_title("Title")
#bx.imshow(anafiLayers[7][0], cmap="hot", interpolation='gaussian') #, interpolation='bicubic'
#bx.set_xlabel("X (meters)")
#bx.set_ylabel("Y (meters)")
#for i in range(12):
#    for j in range(12):
#        bx.scatter(i,j, color='white', alpha='0.6', s=4)
#ax.imshow(anafiLayers[6][0], cmap="hot", interpolation='gaussian') #, interpolation='bicubic'
#plt.imshow(anafiLayers[7][0], cmap="hot",origin='lower', interpolation='gaussian') #, interpolation='bicubic'

#plt.show() 




#def npy2fig(npy_file_path, title_list=titles,exclude=[]):
#    fig_data = np.load(npy_file_path)
##    variables,x,y = fig_data.shape
##    print(fig_data.shape)
##    print(len(fig_data))
##    print(" x:", x, " y:", y, " variables:", variables)
#    print(fig_data.astype(str))
#    print(fig_data.shape)
#    
##    img_graphs(fig_data)
#    
##    axis_X = np.zeros((x))
##    axis_Y = np.zeros((y))
##    print(list(axis_X))
##    print(axis_Y)
#
#
## define the rooms diamensions (x,y,z -> width,length, height)
#width = 5
#w_step = 1
#
#length = 5
#l_step = 1
#
#height = 1
#h_step = 0.5
#
## Pre-calculate how many points required with the given dimentions
#h_points = int((height-h_step)/h_step)
#w_points = int((width-w_step)/w_step)
#l_points = int((length-l_step)/l_step)
#
##   Define the titles of the figures
#titles = ['Packets Transmitted (#)','Packets Received (#)', 
#        'Packet loss (%)','Total Time (ms)', 'Minimum Time (ms)','Maximun Time (ms)',
#        'Avarage Time (ms)','Mean Deviation (ms)', 'Wifi measurements (#/Point)',
#        'Link Quality (#/70)', 'Link Quality (%)','Signal Level (dBm)','Signal Level (mW)',
#        'Noise Level (dBm)', 'Noise Level (mW)']
#
#npy_file_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/PC2DR_050320_180135.npy'
#
#def dict2fig(dictionary, title_list):
#    # Initialiaze all fields to np arrays filled with zeros
##    axis_X = np.zeros((w_points))
##    axis_Y = np.zeros((l_points))
#    pass
#
#
#
#
#
#def img_graphs(fig_data, titles=titles, exclude=[]):
#    """Generate graphs from numpy arrays"""
##   ================================================================================================
#        # Initialiaze all fields to np arrays filled with zeros
#    axX = np.zeros((w_points))
#    axY = np.zeros((l_points))
#    
#    for w in range(w_points):
#        axX[w] = int(w)
#        for l in range(l_points):
#            axY[l] = int(l)
##    fig, ((ax, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 90))
#    fig = plt.figure(figsize=(16,90))
#    fig.tight_layout()
#    
#    for i in range(len(fig_data)):     
#        a = fig.add_subplot(int((len(fig_data)/2)+1), 2, i+1)
#        
#        imgplot = plt.imshow(fig_data[i], interpolation='catrom') # cmap="hot",  cmap='nipy_spectral',interpolation="nearest", "bicubic"
#        a.set_title(titles[i])
#        
#        #   fig, ax = plt.subplots()
##        im = ax.imshow(harvest,interpolation='catrom') # 'gaussian'
#        
#        # We want to show all ticks...
#        a.set_xticks(np.arange(len(axX)))
#        a.set_yticks(np.arange(len(axY)))
#        # ... and label them with the respective list entries
#        a.set_xticklabels(axX)
#        a.set_yticklabels(axY)
#        
#        ## Rotate the tick labels and set their alignment.
#        #plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
#        #         rotation_mode="anchor")
#        text_mode = False
#        if text_mode == True:
#            # Loop over data dimensions and create text annotations.
#            for ii in range(len(axY)):
#                for j in range(len(axX)):
#                    text = a.text(j, ii, fig_data[i][ii, j], ha="center", va="center", color="w")
#        
##        a.set_title("Harvest of local farmers (in tons/year)")
#        
#        plt.show() 
##   ================================================================================================ 
#        
#        fig.suptitle('Level 1', fontsize=16) # +str(level)+"/"+str(l_points)
#        if i == 2:      # Packet loss(%) 
#            plt.colorbar(ticks=[0, 50, 100], orientation='horizontal')
#            imgplot.set_clim(0, 100)  
#        elif i == 9:    # Link Quality (#/70)
#            plt.colorbar(ticks=[0, 35, 70], orientation='horizontal')
#            imgplot.set_clim(0, 70)
#        elif i == 10:   # Link Quality (%) OR Packet loss(%)
#            plt.colorbar(ticks=[0, 50, 100], orientation='horizontal')
#            imgplot.set_clim(0, 100)
#        elif i == 11:   # Link Quality (%) OR Packet loss(%)
#            plt.colorbar(ticks=[-30, -65, -100], orientation='horizontal')
#            imgplot.set_clim(-30, -100)
#        elif i == 13:   # Noise Level (dBm)
#            plt.colorbar(ticks=[-20, -138, -256], orientation='horizontal')
#            imgplot.set_clim(-20, -256)
#        else:           
#            plt.colorbar(ticks=[np.amin(fig_data[i]), ((np.amax(fig_data[i])-np.amin(fig_data[i]))/2)+np.amin(fig_data[i]), np.amax(fig_data[i])], orientation='horizontal')
#            imgplot.set_clim(np.amin(fig_data[i])-(np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02, (np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02+np.amax(fig_data[i]))
#    plt.show()
#    
##  ==========[ Debugging prints ]==========
#    if dbg_mode:
#        print("\n ================  DEBUGGING IS ENABLED  ==================")
#        for i in range(len(fig_data)):
#            print("\n",titles[i],":\n",(len(titles[i])+2)*"-","\n", fig_data[i])
#
##  ==========[ Main() ]==========
#def main():
#    """Main"""
#    npy2fig(npy_file_path, titles)
#
#if __name__ == '__main__':
#    main()