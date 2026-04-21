## Goals
This exercise will be the first step into the domain of human activity recognition, aiming for gesture recognition (the glance at the wearable's screen) and step detection (in a walking activity).

First, you will implement a simple windowing approach to cut incoming sensor data into smaller chunks and then analyze the accelerometer data to detect the gesture/posture of reading the device's screen: lifting the arm and holding the arm in front of the chest to read the display.

Then, using recorded data samples from different activities (e.g., walking, standing still, stepping quickly), you will determine the filter parameters to detect walking accurately. Once these parameters are determined, you will apply them in real-time to filter incoming IMU sensor data on the wearable device and recognize walking activity effectively.

## Task Description
**(GO FOR THE FULL POINTS VERSION ONLY)**

### Glance on Screen Detection (3 points)
For the wearable device, implement a windowing approach (Tutorial 7, no overlap necessary here) that buffers 250 ms of accelerometer data, sampled, e.g., at 20 Hz, and then analyzes the signals from the x, y, and z axes to detect the gesture/posture of reading the device's screen. Keep the approach simple (KISS design principle), so aggregate the buffered data, e.g., using the mean, and use simple thresholds (ideally with error margin and hysteresis). Show a simple text or illustration on the screen when the gesture/posture is detected (and clear it afterward) to demonstrate the working feature in the video submission.

### Step Detection & Counting (6 points) 
In this task, you will develop a step-counting algorithm that combines the windowing concept (Task 1) with filters to clean the raw signals obtained from the sensor.

**Data Acquisition**
First, implement software that allows you to stream the accelerometer measurements to your computer, and then record and save them into a *.csv file. We recommend a sampling rate of at least 20 Hz. With this tool, record at least 60 seconds of data with the device worn at either wrist, repeating a cycle of 15s walking and 15s standing twice (so walk, stand, walk, and stand).

**Offline Analysis**
Use the provided Jupyter Notebook to analyze the data (based on Tutorial 6). You can use the FFT (fast Fourier transform) or Welch's method (average of multiple FFTs on data segments) to obtain the frequency spectrum or periodogram, respectively. Based on your insights, decide on a suitable aggregation technique as well as filters and filter parameters (e.g., cutoff frequency) to preprocess the raw signals. With the target application of step counting in mind, use the provided Python functions to test your concept and configuration.

**Wearable Implementation & Step Detection**
Finally, you will transfer your system concept to the wearable device. Use the provided Python script to calculate the filter coefficients for implementing the filters (Tutorial 6), enabling real-time signal processing on the wearable. Keep in mind that you can peek into the black box and test your filter implementation by streaming the signals to your computer at any time. Eventually, implement the rest of the concept by implementing aggregation, windowing (Tutorial 7, Task 1), and step feature detection. Base your implementation on realistic and feasible assumptions on the step activity (e.g., step frequency and interval) to determine, e.g., window size, overlap, and threshold levels.

* **Version A (1.0 / 3.0 points):** Detect whether the user is walking or not walking.
* **Version B (3.0 / 3.0 points):** Count the number of steps the user is performing.

### Display Results
Display the results:
* **Version A (0.25 / 1.0 points):** Stream the step count to a computer terminal.
* **Version B (0.5 / 1.0 points):** Display the live count directly on the wearable screen.
* **Version C (1.0 / 1.0 points):** Show the count only when the reading gesture (Task 1) is detected.

### Reflection (1 point)
a) First, write a brief reflection about your experience with the tasks. Cover your gained knowledge, insights, and identified topics where you need further clarification or faced challenges.

b) Afterward, write down your thoughts, in particular on the following aspects:
* Which values did you obtain for cutoff frequencies?
* How would these cutoff frequencies have to be adjusted to detect different activities, e.g., running?