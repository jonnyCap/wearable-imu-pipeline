### GitHub "About" Section
A real-time Human Activity Recognition (HAR) pipeline for wearable devices, processing IMU accelerometer data for screen-glance gesture detection and algorithmic step counting.

---

### The One-Liner (For Portfolios or Resumes)
An end-to-end wearable sensor pipeline that translates raw accelerometer data into real-time step counts and posture recognition.

---

### The README.md Introduction

# Wearable IMU Pipeline

This repository implements a real-time Human Activity Recognition (HAR) system for wearable devices. The project focuses on processing raw Inertial Measurement Unit (IMU) data to achieve two main goals:

* **Gesture Recognition:** A simple windowing algorithm buffering accelerometer data to detect "glance at screen" arm postures using thresholding and hysteresis.
* **Step Detection & Counting:** A signal processing pipeline that utilizes offline frequency analysis (FFT/Welch's method) to design and apply real-time filters for accurate step counting during walking activities.