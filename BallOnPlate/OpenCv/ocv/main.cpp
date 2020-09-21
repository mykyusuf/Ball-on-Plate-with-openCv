// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <chrono>
#include <ctime>
/*
TRACK A YELLOW BALL - OBJECT DETECTION METHOD USING COLOR SEPERATION OPEN CV 3.1.0
author - Rachit Gulati
*/

using namespace std;

string exec(string command) {
   char buffer[128];
   string result = "";

   // Open pipe to file
   FILE* pipe = popen(command.c_str(), "r");
   if (!pipe) {
      return "popen failed!";
   }

   // read till end of process:
   while (!feof(pipe)) {

      // use buffer to read and add to result
      if (fgets(buffer, 128, pipe) != NULL)
         result += buffer;
   }

   pclose(pipe);
   return result;
}

int main() {
    

    cv::VideoCapture capWebcam(0);        // declare a VideoCapture object to associate webcam, 0 means use 1st (default) webcam

    
    if (capWebcam.isOpened() == false)     //  To check if object was associated to webcam successfully
    {
        std::cout << "error: Webcam connect unsuccessful\n";    // if not then print error message
        return(0);                                                // and exit program
    }

    cv::Mat imgOriginal;        // Input image
    cv::Mat hsvImg;                // HSV Image
    cv::Mat threshImg;            // Thresh Image

    std::vector<cv::Vec3f> v3fCircles;        // 3 element vector of floats, this will be the pass by reference output of HoughCircles()

    char charCheckForEscKey = 0;
    
    int lowH = 80;                            // Set Hue
    int highH = 120;

    int lowS = 40;                            // Set Saturation
    int highS = 80;

    int lowV = 180;                            // Set Value
    int highV = 255;
    // HUE for YELLOW is 21-30.
    // Adjust Saturation and Value depending on the lighting condition of the environment as well as the surface of the object.

    auto start = std::chrono::system_clock::now();
    int tempx=0;
    int tempy=0;
    int check=1;
    double velocity=0;
    
    while (charCheckForEscKey != 27 && capWebcam.isOpened()) {                // until the Esc is pressed or webcam connection is lost
        
        // Some computation here
        
        bool blnFrameReadSuccessfully = capWebcam.read(imgOriginal);        // get next frame

        
        if (!blnFrameReadSuccessfully || imgOriginal.empty()) {                // if frame read unsuccessfully
            std::cout << "error: frame can't read \n";                        // print error message
            break;                                                            // jump out of loop
        }

        
        imgOriginal.convertTo(imgOriginal, -1,1,1);
        cv::cvtColor(imgOriginal, hsvImg, cv::COLOR_BGR2HSV);                        // Convert Original Image to HSV Thresh Image

        cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), threshImg);

        cv::GaussianBlur(threshImg, threshImg, cv::Size(3, 3), 0);            //Blur Effect
        cv::dilate(threshImg, threshImg, 0);                                // Dilate Filter Effect
        cv::erode(threshImg, threshImg, 0);                                    // Erode Filter Effect

                                                                            // fill circles vector with all circles in processed image
        cv::HoughCircles(threshImg, v3fCircles, cv::HOUGH_GRADIENT, 2, threshImg.rows / 4, 100, 50, 25, 40);  // algorithm for detecting circles

        
        
        for (int i = 0; i < v3fCircles.size(); i++) {                        // for each circle

            //std::cout << "Ball position X = " << v3fCircles[i][0]            // x position of center point of circle
            //    << ",\tY = " << v3fCircles[i][1]                                // y position of center point of circle
            //    << ",\tRadius = " << v3fCircles[i][2] << "\n";                    // radius of circle

            if (check) {
                velocity=(int)v3fCircles[i][0];
                check=0;
            }
            
            
            string tmp="echo ";
            
            if((int)v3fCircles[i][0]/100 != 0){
                tmp+=to_string((int)v3fCircles[i][0]);
            }
            else if((int)v3fCircles[i][0]/10 != 0){
                tmp+="0"+to_string((int)v3fCircles[i][0]);
            }
            else{
                tmp+="00"+to_string((int)v3fCircles[i][0]);
            }
            tmp+=",";
            
            if((int)v3fCircles[i][1]/100 != 0){
                tmp+=to_string((int)v3fCircles[i][1])+">/dev/tty.HC-05-DevB";
            }
            else if((int)v3fCircles[i][1]/10 != 0){
                tmp+="0"+to_string((int)v3fCircles[i][1])+">/dev/tty.HC-05-DevB";
            }
            else{
                tmp+="00"+to_string((int)v3fCircles[i][1])+">/dev/tty.HC-05-DevB";
            }
                
                
            
            auto end = std::chrono::system_clock::now();

            std::chrono::duration<double> elapsed_seconds = end-start;
            
            if(elapsed_seconds.count()>0.01){
                
                    exec(tmp);
                    check=1;
                    cout<<tmp;
                    tempx=v3fCircles[i][0];
                    tempy=v3fCircles[i][1];
                
                start=end;
                
            }
      
            

                                                                                // draw small green circle at center of object detected
            cv::circle(imgOriginal,                                                // draw on original image
                cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),        // center point of circle
                3,                                                                // radius of circle in pixels
                cv::Scalar(0, 255, 0),                                            // draw green
                cv::FILLED);                                                        // thickness

                                                                                // draw red circle around object detected
            cv::circle(imgOriginal,                                                // draw on original image
                cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),        // center point of circle
                (int)v3fCircles[i][2],                                            // radius of circle in pixels
                cv::Scalar(0, 0, 255),                                            // draw red
                3);                                                                // thickness
        }

        // declare windows
        cv::namedWindow("imgOriginal", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("threshImg", cv::WINDOW_AUTOSIZE);

        /* Create trackbars in "threshImg" window to adjust according to object and environment.*/
        cv::createTrackbar("LowH", "threshImg", &lowH, 179);    //Hue (0 - 179)
        cv::createTrackbar("HighH", "threshImg", &highH, 179);

        cv::createTrackbar("LowS", "threshImg", &lowS, 255);    //Saturation (0 - 255)
        cv::createTrackbar("HighS", "threshImg", &highS, 255);

        cv::createTrackbar("LowV", "threshImg", &lowV, 255);    //Value (0 - 255)
        cv::createTrackbar("HighV", "threshImg", &highV, 255);


        cv::imshow("imgOriginal", imgOriginal);                    // show windows
        cv::imshow("threshImg", threshImg);

        charCheckForEscKey = cv::waitKey(1);                    // delay and get key press
    }

    return(0);
}
