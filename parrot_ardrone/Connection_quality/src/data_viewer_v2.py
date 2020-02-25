import numpy as np
import matplotlib.pyplot as plt

dbg_mode = True

# define the rooms diamensions (x,y,z -> width,length, height)
width = 5
w_step = 1

length = 5
l_step = 1

height = 1
h_step = 0.5

# Pre-calculate how many points required with the given dimentions
h_points = int((height-h_step)/h_step)
w_points = int((width-w_step)/w_step)
l_points = int((length-l_step)/l_step)

#   Define the titles of the figures
titles = ['Packets Transmitted (#)','Packets Received (#)', 
        'Packet loss (%)','Total Time (ms)', 'Minimum Time (ms)','Maximun Time (ms)',
        'Avarage Time (ms)','Mean Deviation (ms)', 'Wifi measurements (#/Point)',
        'Link Quality (#/70)', 'Link Quality (%)','Signal Level (dBm)','Signal Level (mW)',
        'Noise Level (dBm)', 'Noise Level (mW)']

npy_file_path = '/home/ros/parrot2_ws/src/parrot_ardrone/Connection_quality/src/NPY/Ping_(1.0m)_240220_130152.npy'

def dict2fig(dictionary, title_list):
    # Initialiaze all fields to np arrays filled with zeros
#    axis_X = np.zeros((w_points))
#    axis_Y = np.zeros((l_points))
    pass

def nparray2fig(np_array, title_list):
    pass

def npy2fig(npy_file_path, title_list=titles,exclude=[]):
    fig_data = np.load(npy_file_path)
    variables,x,y = fig_data.shape
#    print(fig_data.shape)
#    print(len(fig_data))
    print(" x:", x, " y:", y, " variables:", variables)
    
    img_graphs(fig_data)
    
    axis_X = np.zeros((x))
    axis_Y = np.zeros((y))
    print(list(axis_X))
    print(axis_Y)

def csv2fig(csv_file, title_list):
    pass

def json2fig(csv_file, title_list):
    pass

def img_graphs(fig_data, titles=titles, exclude=[]):
    """Generate graphs from numpy arrays"""
#   ================================================================================================
        # Initialiaze all fields to np arrays filled with zeros
    axX = np.zeros((w_points))
    axY = np.zeros((l_points))
    
    for w in range(w_points):
        axX[w] = int(w)
        for l in range(l_points):
            axY[l] = int(l)
#    fig, ((ax, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 90))
    fig = plt.figure(figsize=(16,90))
    fig.tight_layout()
    
    for i in range(len(fig_data)):     
        a = fig.add_subplot(int((len(fig_data)/2)+1), 2, i+1)
        
        imgplot = plt.imshow(fig_data[i], interpolation='catrom') # cmap="hot",  cmap='nipy_spectral',interpolation="nearest", "bicubic"
        a.set_title(titles[i])
        
        #   fig, ax = plt.subplots()
#        im = ax.imshow(harvest,interpolation='catrom') # 'gaussian'
        
        # We want to show all ticks...
        a.set_xticks(np.arange(len(axX)))
        a.set_yticks(np.arange(len(axY)))
        # ... and label them with the respective list entries
        a.set_xticklabels(axX)
        a.set_yticklabels(axY)
        
        ## Rotate the tick labels and set their alignment.
        #plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
        #         rotation_mode="anchor")
        text_mode = False
        if text_mode == True:
            # Loop over data dimensions and create text annotations.
            for ii in range(len(axY)):
                for j in range(len(axX)):
                    text = a.text(j, ii, fig_data[i][ii, j], ha="center", va="center", color="w")
        
#        a.set_title("Harvest of local farmers (in tons/year)")
        
        plt.show() 
#   ================================================================================================ 
        
        fig.suptitle('Level 1', fontsize=16) # +str(level)+"/"+str(l_points)
        if i == 2:      # Packet loss(%) 
            plt.colorbar(ticks=[0, 50, 100], orientation='horizontal')
            imgplot.set_clim(0, 100)  
        elif i == 9:    # Link Quality (#/70)
            plt.colorbar(ticks=[0, 35, 70], orientation='horizontal')
            imgplot.set_clim(0, 70)
        elif i == 10:   # Link Quality (%) OR Packet loss(%)
            plt.colorbar(ticks=[0, 50, 100], orientation='horizontal')
            imgplot.set_clim(0, 100)
        elif i == 11:   # Link Quality (%) OR Packet loss(%)
            plt.colorbar(ticks=[-30, -65, -100], orientation='horizontal')
            imgplot.set_clim(-30, -100)
        elif i == 13:   # Noise Level (dBm)
            plt.colorbar(ticks=[-20, -138, -256], orientation='horizontal')
            imgplot.set_clim(-20, -256)
        else:           
            plt.colorbar(ticks=[np.amin(fig_data[i]), ((np.amax(fig_data[i])-np.amin(fig_data[i]))/2)+np.amin(fig_data[i]), np.amax(fig_data[i])], orientation='horizontal')
            imgplot.set_clim(np.amin(fig_data[i])-(np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02, (np.amax(fig_data[i])-np.amin(fig_data[i]))*0.02+np.amax(fig_data[i]))
    plt.show()
    
#  ==========[ Debugging prints ]==========
    if dbg_mode:
        print("\n ================  DEBUGGING IS ENABLED  ==================")
        for i in range(len(fig_data)):
            print("\n",titles[i],":\n",(len(titles[i])+2)*"-","\n", fig_data[i])

#  ==========[ Main() ]==========
def main():
    """Main"""
    npy2fig(npy_file_path, titles)

if __name__ == '__main__':
    main()