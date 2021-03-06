/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]){

  /* INIT VARIABLES AND DATA STRUCTURES */

  // data location
  string dataPath = "../";

  // camera
  string imgBasePath = dataPath + "images/";
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
  string imgFileType = ".png";
  int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
  int imgEndIndex = 9;   // last file index to load
  int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

  // misc
  int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
  // vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
  bool bVis = false;            // visualize results

  /* MAIN LOOP OVER ALL IMAGES */

  vector<string> kptTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
  vector<string> desTypes = {"BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
  ofstream detector_file;
  detector_file.open ("../MP_7_Counts_Keypoints.csv");

  ofstream det_des_matches;
  det_des_matches.open ("../MP_8_Counts_Matched_Keypoints.csv");

  ofstream det_des_time;
  det_des_time.open ("../MP_9_Log_Time.csv");  

  for (auto i = 0; i < kptTypes.size(); i++){
    string detectorType = kptTypes[i];
    detector_file << detectorType;
    bool writeDetector = true;
    for (auto j = 0; j < desTypes.size(); j++){
      vector<DataFrame> dataBuffer;
      string descriptorType = desTypes[j];
      if (descriptorType.compare("ORB") == 0 && detectorType.compare("SIFT") == 0)
        continue;
      det_des_matches << detectorType << "+" << descriptorType;
      det_des_time << detectorType << "+" << descriptorType;
      for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++){
        double t = (double)cv::getTickCount();
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() >= dataBufferSize){
          dataBuffer.pop_back();
        }
        vector<DataFrame>::iterator it = dataBuffer.begin();
        // dataBuffer.push_back(frame);
        dataBuffer.insert(it, frame);

        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0){
          detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0){
          detKeypointsHARRIS(keypoints, imgGray, false);
        }
        else if (detectorType.compare("FAST") == 0){
          detKeypointsFAST(keypoints, imgGray, false);
        }
        else if (detectorType.compare("BRISK") == 0){
          detKeypointsBRISK(keypoints, imgGray, false);
        }
        else if (detectorType.compare("ORB") == 0){
          detKeypointsORB(keypoints, imgGray, false);
        }
        else if (detectorType.compare("AKAZE") == 0){
          detKeypointsAKAZE(keypoints, imgGray, false);
        }
        else if (detectorType.compare("SIFT") == 0){
          detKeypointsSIFT(keypoints, imgGray, false);
        }

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle){
          vector<cv::KeyPoint> raw_kpts = keypoints;
          keypoints.clear();
          for (int i = 0; i < raw_kpts.size(); i++){
            int ptx = raw_kpts[i].pt.x;
            int pty = raw_kpts[i].pt.y;
            int w = vehicleRect.width;
            int h = vehicleRect.height;
            int x = vehicleRect.x;
            int y = vehicleRect.y;
            if (ptx >= x && ptx <= x + w){
              if (pty >= y && pty <= y + h){
                keypoints.push_back(raw_kpts[i]);
              }
            }
          }
          cv::rectangle(dataBuffer.begin()->cameraImg, vehicleRect, cv::Scalar::all(-1));
        }

        // MP7: keypoint numbers.
        if (writeDetector)
          detector_file << ", " << keypoints.size();
        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.begin())->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
    
        cv::Mat descriptors;
        // string descriptorType = desTypes[j]; // BRIEF, ORB, FREAK, AKAZE, SIFT
        if (detectorType.compare("AKAZE") != 0 && descriptorType.compare("AKAZE") == 0){
          continue;
        }
        cout << descriptorType << endl;
        descKeypoints((dataBuffer.begin())->keypoints, (dataBuffer.begin())->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT
    
        // push descriptors for current frame to end of data buffer
        (dataBuffer.begin())->descriptors = descriptors;
    
        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
    

        if (dataBuffer.size() > 1){ // wait until at least two images have been processed
          
    
          /* MATCH KEYPOINT DESCRIPTORS */
    
          vector<cv::DMatch> matches;
          string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
          string descriptorKind = "DES_BINARY"; // DES_BINARY, DES_HOG
          string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
          if (detectorType.compare("SIFT") == 0){
            // SIFT is HOG descriptor.
            descriptorKind = "DES_HOG";
          }
          //// STUDENT ASSIGNMENT
            
          //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
    
          //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
          matchDescriptors((dataBuffer.begin() + 1)->keypoints, (dataBuffer.begin())->keypoints,
                           (dataBuffer.begin() + 1)->descriptors, (dataBuffer.begin())->descriptors,
                           matches, descriptorKind, matcherType, selectorType);
            
          //// EOF STUDENT ASSIGNMENT
            
          // MP8.
          det_des_matches << ", " << matches.size();
          t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
          det_des_time << ", " << 1000*t;
          // store matches in current data frame
          (dataBuffer.begin() + 1)->kptMatches = matches;
          cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
            
          // visualize matches between current and previous image
          bVis = false;
          if (bVis){
            cv::Mat matchImg = ((dataBuffer.begin())->cameraImg).clone();
            cv::drawMatches((dataBuffer.begin() + 1)->cameraImg, (dataBuffer.begin() + 1)->keypoints,
                            (dataBuffer.begin())->cameraImg, (dataBuffer.begin())->keypoints,
                            matches, matchImg,
                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                            vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            string windowName = "Matching keypoints between two camera images";
            cv::namedWindow(windowName, 7);
            cv::imshow(windowName, matchImg);
            cout << "Press key to continue to next image" << endl;
            cv::waitKey(0); // wait for key to be pressed
          }
          bVis = false;
        }        
      }
      det_des_matches << endl;
      det_des_time << endl;
      if (writeDetector)
        detector_file << endl;
      writeDetector = false;
    }
  }

  
  // for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++){
  //   /* LOAD IMAGE INTO BUFFER */

  //   // assemble filenames for current index
  //   ostringstream imgNumber;
  //   imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
  //   string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

  //   // load image from file and convert to grayscale
  //   cv::Mat img, imgGray;
  //   img = cv::imread(imgFullFilename);
  //   cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

  //   //// STUDENT ASSIGNMENT
  //   //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

  //   // push image into data frame buffer
  //   DataFrame frame;
  //   frame.cameraImg = imgGray;
  //   if (dataBuffer.size() >= dataBufferSize){
  //     dataBuffer.pop_back();
  //   }
  //   vector<DataFrame>::iterator it = dataBuffer.begin();
  //   // dataBuffer.push_back(frame);
  //   dataBuffer.insert(it, frame);

  //   //// EOF STUDENT ASSIGNMENT
  //   cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

  //   /* DETECT IMAGE KEYPOINTS */

  //   // extract 2D keypoints from current image
  //   vector<cv::KeyPoint> keypoints; // create empty feature list for current image
  //   string detectorType = "SIFT";

  //   //// STUDENT ASSIGNMENT
  //   //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
  //   //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

  //   if (detectorType.compare("SHITOMASI") == 0){
  //     detKeypointsShiTomasi(keypoints, imgGray, false);
  //   }
  //   else if (detectorType.compare("HARRIS") == 0){
  //     detKeypointsHARRIS(keypoints, imgGray, false);
  //   }
  //   else if (detectorType.compare("FAST") == 0){
  //     detKeypointsFAST(keypoints, imgGray, false);
  //   }
  //   else if (detectorType.compare("BRISK") == 0){
  //     detKeypointsBRISK(keypoints, imgGray, false);
  //   }
  //   else if (detectorType.compare("ORB") == 0){
  //     detKeypointsORB(keypoints, imgGray, false);
  //   }
  //   else if (detectorType.compare("AKAZE") == 0){
  //     detKeypointsAKAZE(keypoints, imgGray, false);
  //   }
  //   else if (detectorType.compare("SIFT") == 0){
  //     detKeypointsSIFT(keypoints, imgGray, false);
  //   }
  //   //// EOF STUDENT ASSIGNMENT

  //   //// STUDENT ASSIGNMENT
  //   //// TASK MP.3 -> only keep keypoints on the preceding vehicle

  //   // only keep keypoints on the preceding vehicle
  //   bool bFocusOnVehicle = true;
  //   cv::Rect vehicleRect(535, 180, 180, 150);
  //   if (bFocusOnVehicle){
  //     vector<cv::KeyPoint> raw_kpts = keypoints;
  //     keypoints.clear();
  //     for (int i = 0; i < raw_kpts.size(); i++){
  //       int ptx = raw_kpts[i].pt.x;
  //       int pty = raw_kpts[i].pt.y;
  //       int w = vehicleRect.width;
  //       int h = vehicleRect.height;
  //       int x = vehicleRect.x;
  //       int y = vehicleRect.y;
  //       if (ptx >= x && ptx <= x + w){
  //         if (pty >= y && pty <= y + h){
  //           keypoints.push_back(raw_kpts[i]);
  //         }
  //       }
  //     }
  //     cv::rectangle(dataBuffer.begin()->cameraImg, vehicleRect, cv::Scalar::all(-1));
  //   }

  //   //// EOF STUDENT ASSIGNMENT

  //   // optional : limit number of keypoints (helpful for debugging and learning)
  //   bool bLimitKpts = false;
  //   if (bLimitKpts){
  //     int maxKeypoints = 50;
    
  //     if (detectorType.compare("SHITOMASI") == 0){ 
  //       // there is no response info, so keep the first 50 as they are sorted in descending quality order
  //       keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
  //     }
  //     cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
  //     cout << " NOTE: Keypoints have been limited!" << endl;
  //   }

  //   // push keypoints and descriptor for current frame to end of data buffer
  //   (dataBuffer.begin())->keypoints = keypoints;
  //   cout << "#2 : DETECT KEYPOINTS done" << endl;

  //   /* EXTRACT KEYPOINT DESCRIPTORS */

  //   //// STUDENT ASSIGNMENT
  //   //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
  //   //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

  //   cv::Mat descriptors;
  //   string descriptorType = "SIFT"; // BRIEF, ORB, FREAK, AKAZE, SIFT
  //   descKeypoints((dataBuffer.begin())->keypoints, (dataBuffer.begin())->cameraImg, descriptors, descriptorType);
  //   //// EOF STUDENT ASSIGNMENT

  //   // push descriptors for current frame to end of data buffer
  //   (dataBuffer.begin())->descriptors = descriptors;

  //   cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

  //   if (dataBuffer.size() > 1){ // wait until at least two images have been processed
      

  //     /* MATCH KEYPOINT DESCRIPTORS */

  //     vector<cv::DMatch> matches;
  //     string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
  //     string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
  //     string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
  //     if (detectorType.compare("SIFT") == 0){
  //       // SIFT is HOG descriptor.
  //       descriptorType = "DES_HOG";
  //     }
  //     //// STUDENT ASSIGNMENT
        
  //     //// TASK MP.5 -> add FLANN matching in file matching2D.cpp

  //     //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
  //     matchDescriptors((dataBuffer.begin() + 1)->keypoints, (dataBuffer.begin())->keypoints,
  //                      (dataBuffer.begin() + 1)->descriptors, (dataBuffer.begin())->descriptors,
  //                      matches, descriptorType, matcherType, selectorType);
        
  //     //// EOF STUDENT ASSIGNMENT
        
  //     // store matches in current data frame
  //     (dataBuffer.begin() + 1)->kptMatches = matches;
  //     cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
        
  //     // visualize matches between current and previous image
  //     bVis = true;
  //     if (bVis){
  //       cv::Mat matchImg = ((dataBuffer.begin())->cameraImg).clone();
  //       cv::drawMatches((dataBuffer.begin() + 1)->cameraImg, (dataBuffer.begin() + 1)->keypoints,
  //                       (dataBuffer.begin())->cameraImg, (dataBuffer.begin())->keypoints,
  //                       matches, matchImg,
  //                       cv::Scalar::all(-1), cv::Scalar::all(-1),
  //                       vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  //       string windowName = "Matching keypoints between two camera images";
  //       cv::namedWindow(windowName, 7);
  //       cv::imshow(windowName, matchImg);
  //       cout << "Press key to continue to next image" << endl;
  //       cv::waitKey(0); // wait for key to be pressed
  //     }
  //     bVis = false;
  //   }
  // } // eof loop over all images
  
  return 0;
}
