# Glance + Step Detection

This project implements the full assignment pipeline for the wearable IMU task in `main/`:

- 20 Hz accelerometer sampling
- 250 ms glance windowing with two-window hysteresis
- Resultant magnitude step detection through a 0.7 Hz high-pass and 4.0 Hz low-pass biquad cascade
- Step activity gating with a 0.18 g enter threshold, a 0.12 g exit threshold, and a 300 ms refractory interval
- LCD output that shows the count only while the glance gesture is active

## Signal Flow

The firmware processes each IMU sample in a fixed 20 Hz loop:

1. Initialize the IMU and, when present on the board, enable the AXP192 power rails used by the display and sensor stack.
2. Read raw x/y/z accelerometer values from the MPU6050.
3. Compute the resultant magnitude and filter it with a biquad high-pass and low-pass cascade tuned for walking cadence.
4. Detect step events from the filtered magnitude envelope using a rising threshold, a falling threshold, and a refractory interval.
5. Collect 250 ms windows and classify the glance posture from the mean x/y/z vector plus window stability energy.
6. Apply hysteresis to the glance state so the LCD only shows the step count while the gesture is active.

## Default Tuning

The current constants in `main/main.c` are intentionally conservative so the firmware remains stable for the demo recording:

- Sample rate: 20 Hz
- Glance window: 5 samples
- Glance stability energy enter threshold: 0.035
- Glance stability energy exit threshold: 0.055
- Glance enter thresholds: mean z > 0.70 g, mean x > 0.18 g, |mean y| < 0.45 g, magnitude between 0.80 g and 1.20 g
- Glance exit thresholds: mean z < 0.60 g, mean x < 0.06 g, |mean y| > 0.60 g, magnitude outside 0.75 g to 1.25 g
- Glance hold windows: 2 to enter, 2 to exit
- Step refractory interval: 300 ms
- Step activity thresholds: 0.18 g enter, 0.12 g exit
- Step filter cutoffs: 0.7 Hz high-pass and 4.0 Hz low-pass

## Display Behavior

If the ST7789 LCD initializes successfully on the board, the app clears the screen when glance detection is inactive and shows a glance banner plus the current step count when the gesture is active. If the display cannot be initialized, the same state is still reported over serial logging.

The display code is only compiled for `CONFIG_IDF_TARGET_ESP32`, and the board-specific LCD wiring uses GPIO 13, 15, 5, 23, and 18 for clock, MOSI, CS, DC, and reset respectively.

## Build

From the `glance_step_detector` directory:

```bash
idf.py set-target esp32
idf.py build
```

If your board uses a different ESP32 family, keep the same source and only adjust the LCD pins in `main/main.c` if needed.