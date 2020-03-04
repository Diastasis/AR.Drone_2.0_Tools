#include <vector>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "aruco/markerdetector.h"
#include <opencv2/flann.hpp> 

using namespace std;

int main (int argc, char **argv){

    //read the input image
    cv::Mat InImage=cv::imread(argv[1]);
    aruco::MarkerDetector MDetector;
    
    //Let's detect
    vector<aruco::Marker> Markers = MDetector.detect(InImage);
    
    //For each marker draw info and its boundaries
    for (unsigned int i = 0; i< Markers.size(); i++)
    {
        cout <<Markers[i] << endl;
        Markers[i].draw(InImage, cv::Scalar(0,0,255),2);
    } 
    //Show input with augmented info
    cv::namedWindow("in",1);
    cv::imshow("in",InImage);
    while (char(cv::waitKey(0)) != 27); //wait ESC to be pressed
}
