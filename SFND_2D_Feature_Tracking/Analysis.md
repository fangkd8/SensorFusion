# Statistics

## 1. MP7

Mean valus of different keypoints are shown in the table.
| SHITOMASI | HARRIS | FAST  | BRISK | ORB | AKAZE | SIFT |
| --------- |:------:|:-----:| :----:|:---:|:-----:|:----:|
| 119       | 108.4  | 413.6 |272.2|115 | 166.6 | 138|

Full result is shown in csv file.

## 2. MP8

Full result is shown in csv file.

## 3. MP9

Full result is shown in csv file.

## Analysis

Please run statistics.py to check the mean values.

Since tracking means both real-time and accuracy, in order to select appropriate combination
of Keypoints detector with Descriptor extractor, we need to consider both aspects. 

In considering both, the top 3 choices are:

|Keypoint + Descriptor|Mean Tracking Keypoints Number|Mean Time(ms)|
|:-------------------:|:----------------------------:|:-----------:|
|FAST + BRIEF         |245.77                        |10.908       | 
|FAST + ORB           |231.11                        |11.792       | 
|FAST + SIFT          |232.11                        |25.740       | 
