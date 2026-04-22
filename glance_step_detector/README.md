# Glance + Step Detection

This project implements the full assignment pipeline for the wearable IMU task:

- 20 Hz accelerometer sampling
- 250 ms glance windowing with hysteresis
- Resultant magnitude aggregation for walking detection
- Real-time filtering with a 0.5 Hz high-pass and 6.0 Hz low-pass biquad cascade
- Peak-based step counting with a refractory interval and adaptive threshold
- LCD output that shows the count only while the glance gesture is active

## Signal Flow

The firmware starts with a short stationary calibration phase and then processes each IMU sample in a fixed 20 Hz loop:

1. Read raw x/y/z accelerometer values from the MPU6050.
2. Aggregate the axes into a resultant magnitude.
3. Remove the baseline offset and run the band-pass filter.
4. Count steps from local maxima above an adaptive threshold.
5. Collect 250 ms windows and classify the glance posture from the mean x/y/z vector with hysteresis.
6. Show the step count on the LCD only while the glance posture is active.

## Default Tuning

The current constants are intentionally conservative so the firmware remains stable for the demo recording:

- Sample rate: 20 Hz
- Glance window: 5 samples
- Glance enter distance: 0.22 g from the calibrated baseline posture
- Glance exit distance: 0.14 g from the calibrated baseline posture
- Step refractory interval: 350 ms
- Step threshold multiplier: 2.2x the running noise floor
- Band-pass cutoffs: 0.5 Hz and 6.0 Hz

## Display Behavior

If the ST7789 LCD initializes successfully on the board, the app clears the screen when glance detection is inactive and shows a glance banner plus the current step count when the gesture is active. If the display cannot be initialized, the same state is still reported over serial logging.

## Build

From the `glance_step_detector` directory:

```bash
idf.py set-target esp32
idf.py build
```

If your board uses a different ESP32 family, keep the same source and only adjust the LCD pins in `main/hello_world_main.c` if needed.

## Notes for Reflection

For the assignment write-up, the current filter cutoffs are 0.5 Hz and 6.0 Hz. For faster activities such as running, the high-pass can stay near the same value, while the low-pass and refractory interval usually need to increase and decrease, respectively, to follow the higher cadence.
