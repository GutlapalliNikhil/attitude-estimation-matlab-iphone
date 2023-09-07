# Attitude Estimation of iPhone using MATLAB Code

## Overview

This MATLAB code is designed for attitude estimation using data collected from an iPhone's accelerometer and gyroscope sensors. It implements various attitude estimation techniques, including accelerometer-only estimation, gyroscope-only estimation, a complementary filter, and a Kalman filter. The code processes the sensor data and provides estimates of roll and pitch angles.

## Requirements

* MATLAB (R2016b or later recommended)

* A dataset containing accelerometer and gyroscope measurements (provided as a MAT file)

## Usage

#### Data Preparation: 

Ensure you have the sensor data in the required format. The code assumes that accelerometer and gyroscope measurements are loaded from a MAT file. You may need to adjust the data loading code (load('/MATLAB Drive/MobileSensorData/onlypitch.mat')) to point to your dataset.

#### Run the Code: Execute the MATLAB script containing the provided code. The script will process the sensor data and perform the following:

* Calculate roll and pitch angles using accelerometer data.

* Estimate roll and pitch angles using gyroscope data.

* Implement a complementary filter for attitude estimation.

* Implement a Kalman filter for attitude estimation.

## Visualize Results: 

The code generates plots that visualize the estimated roll and pitch angles using different estimation techniques. You can view these plots to analyze the performance of each method.

## Configuration

You can adjust various parameters in the code to fine-tune the estimation algorithms, such as the value of alpha for the complementary filter or the matrices A, B, C, P, Q, and R for the Kalman filter. Be sure to understand the impact of these changes on your specific dataset and application.
